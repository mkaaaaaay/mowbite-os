#ifndef MOWBITE_JOURNAL_ANONYMIZE_H
#define MOWBITE_JOURNAL_ANONYMIZE_H

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include "mowbite_journal/journaled.h"

namespace mowbite_journal {

// What a copy of a journal without its place gets instead: a fixed place, the wall clock moved by whole days, a boot ID
// of zeros and, unless kept, raw streams as zeros of their size.
struct AnonymizeOptions {
  double latitude = 52.0;
  double longitude = 9.0;
  // Whole days keep the time of day, a multiple of 7 also the weekday the schedule works with.
  int shift_days = -1001;
  std::string boot_id = "00000000-0000-0000-0000-000000000000";
  // Raw streams (bytes on a port) may carry the wall clock in their frames.
  bool keep_raw = false;

  int64_t shift_s() const {
    return int64_t{shift_days} * 86400;
  }
  // A date of the calendar moved by shift_days.
  void ShiftDate(int* year, int* month, int* day) const;
  // Every date YYYY-MM-DD in the text moved by shift_days.
  std::string ShiftDates(const std::string& text) const;
  // The same in every string and key of the value, and numbers under one of unix_keys (unix s, above 0) moved; for
  // json and ordered_json.
  template <typename Json>
  void ShiftWallIn(Json* value, const std::vector<std::string>& unix_keys = {}) const {
    if (value->is_string()) {
      *value = ShiftDates(value->template get<std::string>());
    } else if (value->is_array()) {
      for (auto& item : *value) ShiftWallIn(&item, unix_keys);
    } else if (value->is_object()) {
      Json moved = Json::object();
      for (auto it = value->begin(); it != value->end(); ++it) {
        Json item = it.value();
        const bool wall = std::find(unix_keys.begin(), unix_keys.end(), it.key()) != unix_keys.end();
        if (wall && item.is_number_integer() && item.template get<int64_t>() > 0) {
          item = item.template get<int64_t>() + shift_s();
        } else if (wall && item.is_number_float() && item.template get<double>() > 0) {
          item = item.template get<double>() + static_cast<double>(shift_s());
        } else {
          ShiftWallIn(&item, unix_keys);
        }
        moved[ShiftDates(it.key())] = std::move(item);
      }
      *value = std::move(moved);
    }
  }
};

// The calls of a core that carry a place or the wall clock, and how a copy without its place changes them. The table
// stands next to the method table of the core, as an optional member of its traits:
//   template <typename A>
//   static void Anonymize(A& a) {
//     a.template Args<&Core::OnDatum>([](const auto& o, double& latitude, double& longitude) { ... });
//     a.template Result<&Core::TakeOutput>([](const auto& o, Output& result) { ... });
//     a.Config([](const auto& o, Config& config) { ... });
//   }
// Args gets the options and every argument as the replay holds it; Result gets the options, the result and then the
// arguments after Args.
template <typename Core>
class Anonymizer {
 public:
  using Traits = CoreTraits<Core>;
  using Methods = typename Traits::Methods;
  using CoreConfig = typename Traits::Config;

  static const Anonymizer& Get() {
    static const Anonymizer anonymizer;
    return anonymizer;
  }

  template <auto M, typename F>
  void Args(F f) {
    constexpr size_t kIndex = detail::IndexOf<M>(Methods());
    static_assert(kIndex < Methods::kSize, "the method is not in the journal table of its core");
    using Tuple = typename detail::StoredArgs<typename MethodTraits<decltype(M)>::Params>::Type;
    args_[kIndex] = [f](const AnonymizeOptions& options, void* args) {
      std::apply([&](auto&... arg) { f(options, arg...); }, *static_cast<Tuple*>(args));
    };
  }

  template <auto M, typename F>
  void Result(F f) {
    constexpr size_t kIndex = detail::IndexOf<M>(Methods());
    static_assert(kIndex < Methods::kSize, "the method is not in the journal table of its core");
    using T = MethodTraits<decltype(M)>;
    static_assert(!std::is_void_v<typename T::Result>, "a method without a result");
    using R = std::decay_t<typename T::Result>;
    using Tuple = typename detail::StoredArgs<typename T::Params>::Type;
    results_[kIndex] = [f](const AnonymizeOptions& options, void* result, void* args) {
      std::apply([&](auto&... arg) { f(options, *static_cast<R*>(result), arg...); }, *static_cast<Tuple*>(args));
    };
  }

  template <typename F>
  void Config(F f) {
    config_ = [f](const AnonymizeOptions& options, CoreConfig& config) { f(options, config); };
  }

  // Whether the table names the method.
  bool Covers(uint16_t method) const {
    return method < Methods::kSize && (args_[method] || results_[method]);
  }

  // The call record with its places and wall clock replaced into out; false when the method has neither.
  bool Rewrite(uint16_t method, const uint8_t* data, size_t size, const AnonymizeOptions& options, Out* out) const {
    if (!Covers(method)) return false;
    In in(data, size);
    RewriteTable(std::make_index_sequence<Methods::kSize>())[method](*this, options, in, out);
    return true;
  }

  json RewriteConfig(const json& config, const AnonymizeOptions& options) const {
    if (!config_) return config;
    CoreConfig value = config.get<CoreConfig>();
    config_(options, value);
    return json(value);
  }

 private:
  template <typename C, typename = void>
  struct HasTable : std::false_type {};
  template <typename C>
  struct HasTable<C, std::void_t<decltype(CoreTraits<C>::Anonymize(std::declval<Anonymizer<C>&>()))>>
      : std::true_type {};

  Anonymizer() {
    if constexpr (HasTable<Core>::value) Traits::Anonymize(*this);
  }

  using RewriteFn = void (*)(const Anonymizer&, const AnonymizeOptions&, In&, Out*);

  template <size_t... I>
  static const std::array<RewriteFn, sizeof...(I)>& RewriteTable(std::index_sequence<I...>) {
    static const std::array<RewriteFn, sizeof...(I)> table = {&Anonymizer::RewriteAt<I>...};
    return table;
  }

  template <size_t I>
  static void RewriteAt(const Anonymizer& self, const AnonymizeOptions& options, In& in, Out* out) {
    constexpr auto M = detail::MethodAt<I, Methods>::kValue;
    using T = MethodTraits<decltype(M)>;
    using Params = typename T::Params;
    using R = typename T::Result;
    constexpr size_t N = std::tuple_size_v<Params>;
    const uint32_t args_size = in.GetU32();
    In args_in(in.Take(args_size), args_size);
    auto args = detail::ReadArgs<Params>(args_in, std::make_index_sequence<N>());
    if (self.args_[I]) self.args_[I](options, &args);
    out->Clear();
    out->PutU32(0);
    detail::WriteArgs<Params>(*out, args, std::make_index_sequence<N>());
    out->SetU32(0, static_cast<uint32_t>(out->size() - 4));
    if constexpr (!std::is_void_v<R>) {
      if (self.results_[I]) {
        auto result = Read<std::decay_t<R>>(in);
        self.results_[I](options, &result, &args);
        Write<std::decay_t<R>>(*out, result);
      }
    }
    const size_t left = in.left();
    out->Put(in.Take(left), left);
  }

  std::array<std::function<void(const AnonymizeOptions&, void*)>, Methods::kSize> args_;
  std::array<std::function<void(const AnonymizeOptions&, void*, void*)>, Methods::kSize> results_;
  std::function<void(const AnonymizeOptions&, CoreConfig&)> config_;
};

}  // namespace mowbite_journal

#endif  // MOWBITE_JOURNAL_ANONYMIZE_H
