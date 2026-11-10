#include "mowbite_logic/logic_core.h"

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
  if (file.empty()) {
    Notify("data_dir can be written again: starts are allowed");
    return;
  }
  if (!begins) return;
  json details = json::object();
  PutReason(&details, "STORAGE", StorageProblem());
  AddEvent("STORAGE", std::move(details));
}

void LogicCore::AbandonStart(const std::string& file) {
  SetUnwritableFile(file);
  const Reason why = StorageProblem();
  switch (state_) {
    case State::kMowing:
      RefuseStart(why);
      SetState(State::kIdle);
      return;
    case State::kUndocking:
      if (undock_for_ == DockCheck::kDrive) {
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
      RefuseCalibration(why);
      EndCalibration(std::nullopt, true);
      return;
    case State::kAreaRecording: {
      json details = {{"action", "start_area_recording"}};
      PutReason(&details, "RECORDING_REFUSED", why);
      AddEvent("RECORDING_REFUSED", std::move(details));
      SetState(State::kIdle);
      return;
    }
    case State::kIdle:
    default: return;
  }
}

}  // namespace mowbite_logic
