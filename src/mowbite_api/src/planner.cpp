#include "mowbite_api/planner_schema.h"
#include "planner_schema_text.h"

namespace mowbite_api {

Json PlannerPart() {
  static const Json part = sizeof(kPlannerSchemaText) > 1 ? Json::parse(kPlannerSchemaText) : Json();
  return part;
}

}  // namespace mowbite_api
