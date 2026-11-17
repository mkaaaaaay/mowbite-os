#ifndef MOWBITE_API_INTERNAL_H
#define MOWBITE_API_INTERNAL_H

#include "mowbite_api/catalog.h"

namespace mowbite_api {

// The events and groups of reasons of the setup program (setup.cpp), listed by Events() and Refusals().
EventEntry SetupStepEvent();
EventEntry SetupProgramEvent();
EventEntry SetupRefused();
EventEntry SetupBlocked();

}  // namespace mowbite_api

#endif  // MOWBITE_API_INTERNAL_H
