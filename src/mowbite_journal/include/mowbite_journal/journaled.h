#ifndef MOWBITE_JOURNAL_JOURNALED_H
#define MOWBITE_JOURNAL_JOURNALED_H

#include <cstddef>
#include <cstdint>
#include <string>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include "mowbite_journal/codec.h"
#include "mowbite_journal/format.h"
#include "mowbite_journal/journal.h"

namespace mowbite_journal {

// The methods of a core that change it, in a fixed order: the number of a method is its place, so new ones are only
// appended. Specialized next to each core:
//   template <> struct mowbite_journal::CoreTraits<Core> {
//     static constexpr const char* kName = "mcu";
//     using Config = CoreConfig;
//     MOWBITE_JOURNAL_METHODS(&Core::A, &Core::B);
//     static json View(const Core& core);  // optional: a few values written whenever a call changed them
//   };
template <typename Core>
struct CoreTraits;

template <auto... M>
struct MethodList {
  static constexpr size_t kSize = sizeof...(M);
};

#define MOWBITE_JOURNAL_METHODS(...)                     \
  using Methods = ::mowbite_journal::MethodList<__VA_ARGS__>; \
  static constexpr const char* kMethodText = #__VA_ARGS__

template <typename F>
struct MethodTraits;
template <typename R, typename C, typename... A>
struct MethodTraits<R (C::*)(A...)> {
  using Result = R;
  using Class = C;
  using Params = std::tuple<A...>;
};
template <typename F>
struct MethodTraits<const F> : MethodTraits<F> {};

namespace detail {

template <auto V>
struct Value {};

template <auto M, auto... Ms>
constexpr size_t IndexOf(MethodList<Ms...>) {
  constexpr bool same[] = {std::is_same_v<Value<M>, Value<Ms>>...};
  for (size_t i = 0; i < sizeof...(Ms); ++i) {
    if (same[i]) return i;
  }
  return sizeof...(Ms);
}

template <size_t I, auto M0, auto... Ms>
struct Nth {
  static constexpr auto kValue = Nth<I - 1, Ms...>::kValue;
};
template <auto M0, auto... Ms>
struct Nth<0, M0, Ms...> {
  static constexpr auto kValue = M0;
};

template <size_t I, typename List>
struct MethodAt;
template <size_t I, auto... Ms>
struct MethodAt<I, MethodList<Ms...>> {
  static constexpr auto kValue = Nth<I, Ms...>::kValue;
};

template <typename T>
struct IsOut : std::false_type {};
template <typename T>
struct IsOut<T*> : std::true_type {};

template <typename Core, typename = void>
struct HasView : std::false_type {};
template <typename Core>
struct HasView<Core, std::void_t<decltype(CoreTraits<Core>::View(std::declval<const Core&>()))>> : std::true_type {};

// Names of the methods from the text of the macro: "&Core::A, &Core::B" gives A and B.
std::vector<std::string> MethodNames(const char* text);

template <typename F>
std::string Signature() {
  std::string text = __PRETTY_FUNCTION__;
  const size_t at = text.find("F = ");
  return at == std::string::npos ? text : text.substr(at + 4, text.find_first_of(";]", at) - at - 4);
}

template <typename List>
struct Table;
template <auto... Ms>
struct Table<MethodList<Ms...>> {
  static std::vector<std::string> Signatures() {
    return {Signature<decltype(Ms)>()...};
  }
};

// A FNV-1a hash over the names and signatures, so that replay refuses a journal of another table.
std::string TableChecksum(const std::vector<std::string>& names, const std::vector<std::string>& signatures);

inline Out& CallScratch() {
  thread_local Out out;
  return out;
}

}  // namespace detail

// The definition of a core's stream: name, config and method table.
template <typename Core>
json CoreDefinition(const typename CoreTraits<Core>::Config& config, const json& extra) {
  using Traits = CoreTraits<Core>;
  const auto names = detail::MethodNames(Traits::kMethodText);
  const auto signatures = detail::Table<typename Traits::Methods>::Signatures();
  json methods = json::array();
  for (size_t i = 0; i < names.size(); ++i) methods.push_back({{"name", names[i]}, {"signature", signatures[i]}});
  return {{"name", Traits::kName},
          {"kind", "core"},
          {"config", config},
          {"methods", methods},
          {"table", detail::TableChecksum(names, signatures)},
          {"extra", extra}};
}

// A core whose every change goes through the journal: changing methods only by Call<&Core::Method>(args...), which
// writes the arguments, the result and the out parameters in the order of the calls; reading only by View(). Without a
// journal Call just calls. Calls of one core come one at a time, under the lock of the node that owns it.
template <typename Core>
class Journaled {
 public:
  using Traits = CoreTraits<Core>;
  using Config = typename Traits::Config;

  Journaled(Journal* journal, const Config& config, const json& extra = json::object())
      : core_(config), journal_(journal) {
    if (journal_) stream_ = journal_->Define(CoreDefinition<Core>(config, extra));
  }

  template <auto M, typename... A>
  decltype(auto) Call(A&&... args) {
    using T = MethodTraits<decltype(M)>;
    using Params = typename T::Params;
    using R = typename T::Result;
    static_assert(std::is_same_v<typename T::Class, Core>, "a method of another class");
    constexpr size_t kIndex = detail::IndexOf<M>(typename Traits::Methods());
    static_assert(kIndex < Traits::Methods::kSize, "the method is not in the journal table of its core");
    static_assert(sizeof...(A) == std::tuple_size_v<Params>, "every argument has to be given, also the defaults");
    static_assert(!Pod<std::decay_t<R>>::value, "a result is compared byte for byte: no plain struct with padding");
    if (!journal_) return (core_.*M)(std::forward<A>(args)...);
    Out& out = detail::CallScratch();
    out.Clear();
    out.PutU32(0);
    WriteArgs<Params>(out, std::make_index_sequence<sizeof...(A)>(), args...);
    out.SetU32(0, static_cast<uint32_t>(out.size() - 4));
    if constexpr (std::is_void_v<R>) {
      (core_.*M)(std::forward<A>(args)...);
      Finish<Params>(out, kIndex, std::make_index_sequence<sizeof...(A)>(), args...);
    } else {
      R result = (core_.*M)(std::forward<A>(args)...);
      Write<std::decay_t<R>>(out, result);
      Finish<Params>(out, kIndex, std::make_index_sequence<sizeof...(A)>(), args...);
      return result;
    }
  }

  const Core& View() const {
    return core_;
  }

 private:
  template <typename Params, size_t... I, typename... A>
  static void WriteArgs(Out& out, std::index_sequence<I...>, const A&... args) {
    (WriteArg<std::tuple_element_t<I, Params>>(out, args), ...);
  }

  // An out parameter only says whether it was given; its value follows the result.
  template <typename P, typename A>
  static void WriteArg(Out& out, const A& arg) {
    if constexpr (detail::IsOut<P>::value) {
      const uint8_t given = arg != nullptr;
      out.Put(&given, 1);
    } else {
      Write<std::decay_t<P>>(out, arg);
    }
  }

  template <typename Params, size_t... I, typename... A>
  void Finish(Out& out, size_t index, std::index_sequence<I...>, const A&... args) {
    (WriteOut<std::tuple_element_t<I, Params>>(out, args), ...);
    journal_->Append(stream_, static_cast<uint16_t>(index), out.data(), out.size());
    if constexpr (detail::HasView<Core>::value) {
      Out& view = detail::CallScratch();
      view.Clear();
      Write(view, Traits::View(core_));
      if (view.size() != last_view_.size() || !std::equal(last_view_.begin(), last_view_.end(), view.data())) {
        last_view_.assign(view.data(), view.data() + view.size());
        journal_->Append(stream_, kViewMethod, view.data(), view.size());
      }
    }
  }

  template <typename P, typename A>
  static void WriteOut(Out& out, const A& arg) {
    if constexpr (detail::IsOut<P>::value && !std::is_null_pointer_v<A>) {
      if (arg != nullptr) Write<std::remove_pointer_t<P>>(out, *arg);
    }
  }

  Core core_;
  Journal* journal_;
  uint16_t stream_ = 0;
  std::vector<uint8_t> last_view_;
};

}  // namespace mowbite_journal

#endif  // MOWBITE_JOURNAL_JOURNALED_H
