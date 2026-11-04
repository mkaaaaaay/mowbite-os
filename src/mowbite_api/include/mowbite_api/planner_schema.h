#ifndef MOWBITE_API_PLANNER_SCHEMA_H
#define MOWBITE_API_PLANNER_SCHEMA_H

#include "mowbite_api/catalog.h"

namespace mowbite_api {

// The planner's part of the schema (share/mowbite_planner_os/api/planner.schema.json of the planner found at build
// time): {version, events, topics}; null when the build found none.
Json PlannerPart();

// The schema with this part of the planner instead of the one found at build time.
Json SchemaWithPlanner(const Json& planner);

}  // namespace mowbite_api

#endif  // MOWBITE_API_PLANNER_SCHEMA_H
