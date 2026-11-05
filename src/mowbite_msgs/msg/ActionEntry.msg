# One action of an ActionList; the app sends "<node_prefix>/<id>" on action.
string id
string name
bool enabled
# Only start_mowing while enabled is false: the reason of MOWING_REFUSED a start would get now, else empty.
string reason
