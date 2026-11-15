#ifndef MOWBITE_JOURNAL_REPLAY_H
#define MOWBITE_JOURNAL_REPLAY_H

#include <array>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <ostream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "mowbite_journal/anonymize.h"
#include "mowbite_journal/journaled.h"
#include "mowbite_journal/reader.h"

namespace mowbite_journal {

struct Mismatch {
  std::string what;
  json expected;
  json got;
};

// A core rebuilt from its definition, fed with its calls.
class CoreReplay {
 public:
  virtual ~CoreReplay() = default;
  // Calls the method with the recorded arguments and compares result and out parameters byte for byte.
  virtual std::optional<Mismatch> Apply(uint16_t method, const uint8_t* data, size_t size) = 0;
  // The view the core has now against a recorded one.
  virtual std::optional<Mismatch> CheckView(const uint8_t* data, size_t size) = 0;
  // {"args": [...], "result": ..., "out": [...]} of a call record, without calling anything.
  virtual json Decode(uint16_t method, const uint8_t* data, size_t size) const = 0;
  virtual json DecodeView(const uint8_t* data, size_t size) const = 0;
  // The call record with its places and wall clock replaced (Anonymizer); false when the method has neither.
  virtual bool Anonymize(uint16_t method, const uint8_t* data, size_t size, const AnonymizeOptions& options,
                         Out* out) const = 0;
  virtual json AnonymizeConfig(const json& config, const AnonymizeOptions& options) const = 0;
};

namespace detail {

template <typename P>
decltype(auto) Pass(Stored<P>& stored) {
  if constexpr (IsOut<P>::value) {
    return stored.given ? &stored.value : static_cast<P>(nullptr);
  } else if constexpr (std::is_lvalue_reference_v<P>) {
    static_assert(std::is_const_v<std::remove_reference_t<P>>, "a core method takes no non-const reference");
    return static_cast<P>(stored);
  } else {
    return std::move(stored);
  }
}

}  // namespace detail

template <typename Core>
class Replayer : public CoreReplay {
 public:
  using Traits = CoreTraits<Core>;
  using Methods = typename Traits::Methods;
  using Config = typename Traits::Config;

  // A journal plays if its method table is the start of this one: new methods only come at the end.
  explicit Replayer(const json& definition) : core_(definition.at("config").get<Config>()) {
    const auto names = detail::MethodNames(Traits::kMethodText);
    const auto signatures = detail::Table<Methods>::Signatures();
    const size_t n = definition.at("methods").size();
    const std::string table = definition.at("table").get<std::string>();
    if (n > names.size() || table != detail::TableChecksum({names.begin(), names.begin() + n},
                                                           {signatures.begin(), signatures.begin() + n})) {
      throw std::runtime_error("method table " + table + " of the journal is not the start of " +
                               detail::TableChecksum(names, signatures) + " here: another build of " + Traits::kName);
    }
    UpdateView();
  }

  std::optional<Mismatch> Apply(uint16_t method, const uint8_t* data, size_t size) override {
    if (method >= Methods::kSize) return Mismatch{"method " + std::to_string(method) + " is not in the table", {}, {}};
    In in(data, size);
    auto mismatch = ApplyTable(std::make_index_sequence<Methods::kSize>())[method](this, in);
    UpdateView();
    return mismatch;
  }

  std::optional<Mismatch> CheckView(const uint8_t* data, size_t size) override {
    if (size == view_.size() && std::equal(view_.begin(), view_.end(), data)) return std::nullopt;
    return Mismatch{"view", DecodeView(data, size), DecodeView(view_.data(), view_.size())};
  }

  json Decode(uint16_t method, const uint8_t* data, size_t size) const override {
    if (method >= Methods::kSize) throw std::runtime_error("method " + std::to_string(method) + " not in the table");
    In in(data, size);
    return DecodeTable(std::make_index_sequence<Methods::kSize>())[method](in);
  }

  bool Anonymize(uint16_t method, const uint8_t* data, size_t size, const AnonymizeOptions& options,
                 Out* out) const override {
    return Anonymizer<Core>::Get().Rewrite(method, data, size, options, out);
  }

  json AnonymizeConfig(const json& config, const AnonymizeOptions& options) const override {
    return Anonymizer<Core>::Get().RewriteConfig(config, options);
  }

  json DecodeView(const uint8_t* data, size_t size) const override {
    if constexpr (detail::HasView<Core>::value) {
      In in(data, size);
      return json(Read<decltype(Traits::View(std::declval<const Core&>()))>(in));
    } else {
      return Hex(data, size);
    }
  }

 private:
  using ApplyFn = std::optional<Mismatch> (*)(Replayer*, In&);
  using DecodeFn = json (*)(In&);

  template <size_t... I>
  static const std::array<ApplyFn, sizeof...(I)>& ApplyTable(std::index_sequence<I...>) {
    static const std::array<ApplyFn, sizeof...(I)> table = {&Replayer::ApplyAt<I>...};
    return table;
  }
  template <size_t... I>
  static const std::array<DecodeFn, sizeof...(I)>& DecodeTable(std::index_sequence<I...>) {
    static const std::array<DecodeFn, sizeof...(I)> table = {&Replayer::DecodeAt<I>...};
    return table;
  }

  template <size_t I>
  static std::optional<Mismatch> ApplyAt(Replayer* self, In& in) {
    constexpr auto M = detail::MethodAt<I, Methods>::kValue;
    using T = MethodTraits<decltype(M)>;
    using Params = typename T::Params;
    using R = typename T::Result;
    constexpr size_t N = std::tuple_size_v<Params>;
    const uint32_t args_size = in.GetU32();
    In args_in(in.Take(args_size), args_size);
    auto args = detail::ReadArgs<Params>(args_in, std::make_index_sequence<N>());
    Out& out = detail::CallScratch();
    out.Clear();
    self->Invoke<M, Params, R>(args, out, std::make_index_sequence<N>());
    const size_t left = in.left();
    const uint8_t* recorded = in.Take(left);
    if (left == out.size() && std::equal(recorded, recorded + left, out.data())) return std::nullopt;
    In expected(recorded, left);
    In got(out.data(), out.size());
    Mismatch mismatch{"result", DecodeTail<Params, R>(expected, std::make_index_sequence<N>(), args),
                      DecodeTail<Params, R>(got, std::make_index_sequence<N>(), args)};
    return mismatch;
  }

  template <auto M, typename Params, typename R, typename Tuple, size_t... I>
  void Invoke(Tuple& args, Out& out, std::index_sequence<I...>) {
    if constexpr (std::is_void_v<R>) {
      (core_.*M)(detail::Pass<std::tuple_element_t<I, Params>>(std::get<I>(args))...);
    } else {
      R result = (core_.*M)(detail::Pass<std::tuple_element_t<I, Params>>(std::get<I>(args))...);
      Write<std::decay_t<R>>(out, result);
    }
    (WriteOut<std::tuple_element_t<I, Params>>(out, std::get<I>(args)), ...);
  }

  template <typename P, typename S>
  static void WriteOut(Out& out, const S& slot) {
    if constexpr (detail::IsOut<P>::value) {
      if (slot.given) Write(out, slot.value);
    }
  }

  // Result and out parameters as JSON; broken bytes give their hex.
  template <typename Params, typename R, typename Tuple, size_t... I>
  static json DecodeTail(In& in, std::index_sequence<I...>, const Tuple& args) {
    const uint8_t* start = in.here();
    const size_t size = in.left();
    try {
      json tail = json::object();
      if constexpr (!std::is_void_v<R>) tail["result"] = json(Read<std::decay_t<R>>(in));
      json outs = json::array();
      (ReadOutJson<std::tuple_element_t<I, Params>>(in, Given<std::tuple_element_t<I, Params>>(std::get<I>(args)),
                                                     outs),
       ...);
      if (!outs.empty()) tail["out"] = outs;
      return tail;
    } catch (const std::exception&) {
      return Hex(start, size);
    }
  }

  template <typename P, typename S>
  static bool Given(const S& slot) {
    if constexpr (detail::IsOut<P>::value) {
      return slot.given;
    } else {
      return false;
    }
  }

  template <typename P>
  static void ReadOutJson(In& in, bool given, json& outs) {
    if constexpr (detail::IsOut<P>::value) {
      if (given) outs.push_back(json(Read<std::remove_pointer_t<P>>(in)));
    }
  }

  template <size_t I>
  static json DecodeAt(In& in) {
    constexpr auto M = detail::MethodAt<I, Methods>::kValue;
    using T = MethodTraits<decltype(M)>;
    using Params = typename T::Params;
    using R = typename T::Result;
    constexpr size_t N = std::tuple_size_v<Params>;
    const uint32_t args_size = in.GetU32();
    In args_in(in.Take(args_size), args_size);
    auto args = detail::ReadArgs<Params>(args_in, std::make_index_sequence<N>());
    json decoded = {{"args", ArgsJson<Params>(args, std::make_index_sequence<N>())}};
    decoded.update(DecodeTail<Params, R>(in, std::make_index_sequence<N>(), args));
    return decoded;
  }

  template <typename Params, typename Tuple, size_t... I>
  static json ArgsJson(const Tuple& args, std::index_sequence<I...>) {
    json array = json::array();
    (array.push_back(ArgJson<std::tuple_element_t<I, Params>>(std::get<I>(args))), ...);
    return array;
  }

  template <typename P, typename S>
  static json ArgJson(const S& slot) {
    if constexpr (detail::IsOut<P>::value) {
      return slot.given ? json("out") : json(nullptr);
    } else {
      return json(slot);
    }
  }

  void UpdateView() {
    if constexpr (detail::HasView<Core>::value) {
      Out& view = detail::CallScratch();
      view.Clear();
      Write(view, Traits::View(core_));
      view_.assign(view.data(), view.data() + view.size());
    }
  }

  Core core_;
  std::vector<uint8_t> view_;
};

// The cores a tool knows, by the name in their definition.
using ReplayFactory = std::function<std::unique_ptr<CoreReplay>(const json& definition)>;
using ReplayRegistry = std::map<std::string, ReplayFactory>;

template <typename Core>
void Register(ReplayRegistry& registry) {
  registry[CoreTraits<Core>::kName] = [](const json& definition) {
    return std::make_unique<Replayer<Core>>(definition);
  };
}

enum class CheckOutcome { kSame = 0, kDiffers = 1, kNotReplayable = 2, kError = 3 };
// The worse of two: a difference over an error over a run that cannot be replayed.
CheckOutcome Worse(CheckOutcome a, CheckOutcome b);

struct CheckReport {
  CheckOutcome outcome = CheckOutcome::kSame;
  std::vector<std::string> lines;
};

// Replays every core of every run in dir (a journal_dir or one run) from the start of its process and compares.
CheckReport CheckRuns(const std::string& dir, const ReplayRegistry& registry);

// A copy of one run in out_dir (created, without segments yet) with every place and the wall clock of its cores
// replaced as their Anonymizer tables say, the head moved and its boot ID replaced, raw streams zeroed unless kept.
// A core the registry does not know stops it: its calls could not be read.
bool AnonymizeRun(const std::string& run_dir, const std::string& out_dir, const ReplayRegistry& registry,
                  const AnonymizeOptions& options, std::string* error);

struct ExportFilter {
  std::string stream;
  std::string method;
};

// One JSON line per record that passes the filter.
bool ExportJsonl(const std::string& dir, const ReplayRegistry& registry, const ExportFilter& filter, std::ostream& out,
                 std::string* error);
// The records of one method of one stream as a table: time, then the leaves of their JSON as columns.
bool ExportCsv(const std::string& dir, const ReplayRegistry& registry, const ExportFilter& filter, std::ostream& out,
               std::string* error);

}  // namespace mowbite_journal

#endif  // MOWBITE_JOURNAL_REPLAY_H
