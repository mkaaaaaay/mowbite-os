#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "mowbite_logic/logic_core.h"

namespace mowbite_logic {

const LogicConfig& CoreFacts::config() const {
  return read_->config_;
}

State CoreFacts::state() const {
  return read_->state_;
}

bool CoreFacts::emergency() const {
  return read_->emergency_;
}

const HardwareSnapshot& CoreFacts::hw() const {
  return read_->hw_;
}

const std::optional<PoseSnapshot>& CoreFacts::pose() const {
  return read_->pose_;
}

const Reason& CoreFacts::drivable_problem() const {
  return read_->drivable_problem_;
}

bool CoreFacts::DockContact() const {
  return read_->DockContact();
}

Reason CoreFacts::Busy() const {
  return read_->Busy();
}

Reason CoreFacts::HoldProblem() const {
  return read_->HoldProblem();
}

Reason CoreFacts::Unsafe() const {
  return read_->Unsafe();
}

Reason CoreFacts::AccuracyReason(float accuracy, double limit) const {
  return read_->AccuracyReason(accuracy, limit);
}

bool CoreFacts::Held() const {
  return read_->Held();
}

double CoreFacts::clock_offset() const {
  return read_->clock_offset_;
}

std::string CoreFacts::InvalidRun(const ScheduleRun& run) const {
  return read_->InvalidRun(run);
}

Reason CoreFacts::ScheduleProblem(const StartRequest& request, const ScheduleRun& run, double now_s) const {
  return read_->ScheduleProblem(request, run, now_s);
}

bool CoreFacts::SunsetWanted() const {
  return read_->SunsetWanted();
}

bool CoreFacts::clock_safe() const {
  return read_->clock_safe_;
}

const std::string& CoreFacts::clock_reason() const {
  return read_->clock_reason_;
}

const std::optional<std::pair<double, double>>& CoreFacts::datum() const {
  return read_->datum_;
}

const std::optional<DockPose>& CoreFacts::dock() const {
  return read_->dock_;
}

bool CoreFacts::charge_stop() const {
  return read_->charge_stop_;
}

const StartRequest& CoreFacts::start_request() const {
  return read_->mowing_.start_request();
}

double CoreFacts::DockedHeading() const {
  return read_->DockedHeading();
}

bool CoreFacts::ContactsBack() const {
  return read_->ContactsBack();
}

bool CoreFacts::LeavesForwards() const {
  return read_->LeavesForwards();
}

const char* CoreFacts::LeaveWords() const {
  return read_->LeaveWords();
}

Reason CoreFacts::RequestMowProblem(const StartRequest& request) const {
  return read_->RequestMowProblem(request);
}

Reason CoreFacts::DriveProblem() const {
  return read_->DriveProblem();
}

Reason CoreFacts::MoveProblem() const {
  return read_->MoveProblem();
}

double CoreFacts::DockAccuracyLimit() const {
  return read_->DockAccuracyLimit();
}

Reason CoreFacts::UndockEndProblem(double x, double y) const {
  return read_->UndockEndProblem(x, y);
}

bool CoreFacts::CanDock() const {
  return read_->CanDock();
}

bool CoreFacts::PoseGood() const {
  return read_->PoseGood();
}

Reason CoreFacts::PositionProblem() const {
  return read_->PositionProblem();
}

const std::vector<Ring>& CoreFacts::drivable() const {
  return read_->drivable_;
}

const std::vector<Ring>& CoreFacts::obstacles() const {
  return read_->obstacles_;
}

const Reason& CoreFacts::mow_map_problem() const {
  return read_->mow_map_problem_;
}

Reason CoreFacts::StartProblem(const std::vector<std::string>& tasks) const {
  return read_->StartProblem(tasks);
}

const std::string& CoreFacts::JobId() const {
  return read_->JobId();
}

bool CoreFacts::HomeReachable() const {
  return read_->HomeReachable();
}

EffectQueue& CoreAccess::effects() {
  return core_->effects_;
}

std::string& CoreAccess::client() {
  return core_->client_;
}

HomeWatch& CoreAccess::home() {
  return core_->home_.watch();
}

std::string CoreAccess::AddEvent(const std::string& type, json details) {
  return core_->AddEvent(type, std::move(details));
}

void CoreAccess::Notify(std::string line) {
  core_->Notify(std::move(line));
}

void CoreAccess::SetState(State state) {
  core_->SetState(state);
}

void CoreAccess::CommandCancels() {
  core_->CommandCancels();
}

void CoreAccess::RefuseStart(const Reason& why) {
  core_->RefuseStart(why);
}

void CoreAccess::RefuseDrive(const Reason& why) {
  core_->RefuseDrive(why);
}

void CoreAccess::RunEnds(const std::string& code, const std::string& detail) {
  core_->RunEnds(code, detail);
}

void CoreAccess::FinishRun() {
  core_->FinishRun();
}

void CoreAccess::SetRunPhase(RunPhase phase) {
  core_->SetRunPhase(phase);
}

void CoreAccess::BeginDrive(double x, double y) {
  core_->BeginDrive(x, y);
}

void CoreAccess::StartMowing() {
  core_->StartMowing();
}

void CoreAccess::CancelDrive(const Reason& reason) {
  core_->CancelDrive(reason);
}

std::string CoreAccess::NewId() {
  return core_->NewId();
}

void CoreAccess::CheckpointNow() {
  core_->CheckpointNow();
}

void CoreAccess::BeginDockCheck(DockCheck purpose) {
  core_->BeginDockCheck(purpose);
}

void CoreAccess::AreasPassedOver(const std::vector<JobArea>& areas) {
  core_->AreasPassedOver(areas);
}

void CoreAccess::EndJob(bool finished, bool home, const Reason& why) {
  core_->EndJob(finished, home, why);
}

const ObstaclePart& CoreFacts::obstacle_part() const {
  return read_->obstacle_part_;
}

ObstaclePart& CoreAccess::obstacle_part() {
  return core_->obstacle_part_;
}

}  // namespace mowbite_logic
