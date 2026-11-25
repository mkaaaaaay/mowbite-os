#ifndef MOWBITE_API_INTERNAL_H
#define MOWBITE_API_INTERNAL_H

#include <map>
#include <set>
#include <string>

#include "mowbite_api/catalog.h"

namespace mowbite_api {

// The events and groups of reasons of the setup program (setup.cpp), listed by Events() and Refusals().
EventEntry SetupStepEvent();
EventEntry SetupProgramEvent();
EventEntry SetupRefused();
EventEntry SetupBlocked();

// The reasons of class symptom by event or group, "*" for all of it; every other reason is a cause.
const std::map<std::string, std::set<std::string>>& ReasonSymptoms();

}  // namespace mowbite_api

#endif  // MOWBITE_API_INTERNAL_H
