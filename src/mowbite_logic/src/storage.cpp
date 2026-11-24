#include "mowbite_logic/logic_core.h"

#include <utility>

namespace mowbite_logic {

Reason LogicCore::StorageProblem() const {
  if (unwritable_file_.empty()) return {};
  const std::string text = unwritable_file_ + " in data_dir cannot be written: nothing starts until it can";
  return Reason("STORAGE_NOT_WRITABLE", text).With("file", unwritable_file_);
}

void LogicCore::SetUnwritableFile(const std::string& file) {
  if (file == unwritable_file_) return;
  const bool begins = unwritable_file_.empty();
  unwritable_file_ = file;
  SetDue<ActionsChanged>(true);
  if (!begins) {
    if (file.empty()) Notify("data_dir can be written again: starts are allowed");
    return;
  }
  json details = json::object();
  PutReason(&details, "STORAGE", StorageProblem());
  AddEvent("STORAGE", std::move(details));
}

void LogicCore::AbandonStart(const std::string& file) {
  SetUnwritableFile(file);
  const Reason why = StorageProblem();
  if (state_ != State::kIdle) RunEnds("REFUSED", why.code);
  switch (state_) {
    case State::kIdle: return;
    case State::kMowing:
      RefuseStart(why);
      SetState(State::kIdle);
      return;
    case State::kUndocking:
      if (undocking_.undock_for() == DockCheck::kDrive) {
        RefuseDrive(why);
      } else {
        RefuseStart(why);
      }
      EndUndocking(std::nullopt, why);
      return;
    case State::kDriving:
      RefuseDrive(why);
      OnDriveEnded(false, why);
      // The node drops the drive it was planning; its end finds the core IDLE.
      CancelDrive(why);
      return;
    case State::kDocking:
      RefuseDrive(why);
      EndDocking(why);
      return;
    case State::kCalibrating:
      calibration_.RefuseCalibration(CoreAccess(*this), why);
      calibration_.EndCalibration(CoreAccess(*this), std::nullopt, true);
      return;
    case State::kSetup:
      setup_.EndSetup(CoreAccess(*this), why.message);
      return;
    case State::kAreaRecording: {
      json details = {{"action", "start_area_recording"}};
      PutReason(&details, "RECORDING_REFUSED", why);
      AddEvent("RECORDING_REFUSED", std::move(details));
      SetState(State::kIdle);
    }
  }
}

void LogicCore::SetDataRollback(const json& note) {
  const bool was_open = DataRollbackOpen();
  const json before = std::exchange(data_rollback_, note.is_object() ? note : json());
  if (data_rollback_.is_object() && !before.is_object()) {
    const bool open = DataRollbackOpen();
    json details = {{"what", "restored"}};
    for (const char* key : {"from", "to", "backup", "backup_of"}) {
      if (data_rollback_.contains(key)) details[key] = data_rollback_[key];
    }
    details["confirm"] = open;
    details["severity"] = open ? "warning" : "info";
    details["resume"] = open ? "user" : "auto";
    AddEvent("DATA_ROLLBACK", std::move(details));
  } else if (before.is_object() && !data_rollback_.is_object()) {
    AddEvent("DATA_ROLLBACK", {{"what", "confirmed"}, {"severity", "info"}, {"resume", "auto"}});
  }
  if (was_open != DataRollbackOpen()) SetDue<ActionsChanged>(true);
}

void LogicCore::SetHold(int64_t until) {
  if (until == hold_until_) return;
  hold_until_ = until;
  SetDue<ActionsChanged>(true);
  WatchAnnouncement();
}

bool LogicCore::Held() const {
  return hold_until_ > 0 || hw_.hold;
}

void LogicCore::SetRestartHold(bool held) {
  if (held == restart_hold_) return;
  restart_hold_ = held;
  SetDue<ActionsChanged>(true);
  UpdateEmergency(false);
}

void LogicCore::ReportRestart(const RestartReport& report) {
  json details = {{"cause", report.cause},
                  {"detail", report.detail},
                  {"crashed_at", report.crashed_at > 0 ? json(report.crashed_at) : json()},
                  {"state_before", report.state_before},
                  {"job_id_before", report.job_id_before.empty() ? json() : json(report.job_id_before)},
                  {"hold", report.hold},
                  {"stalled", report.stalled}};
  // The hold judges by cause; the reason tells a node the watch ended the OS for apart from a crash.
  const std::string reason = report.stalled.is_null() ? report.cause : "NODE_STALLED";
  PutReason(&details, "UNEXPECTED_RESTART", Reason(reason, report.detail));
  AddEvent("UNEXPECTED_RESTART", std::move(details));
}

}  // namespace mowbite_logic
