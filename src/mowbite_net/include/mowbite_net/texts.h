#ifndef MOWBITE_NET_TEXTS_H
#define MOWBITE_NET_TEXTS_H

#include <map>
#include <string>

namespace mowbite_net {

// Values put into a text: {n} the name of the mower, and the others its text names.
using Vars = std::map<std::string, std::string>;

// The text of this id in lang (de or en) with the vars filled in; the id itself for an unknown one.
std::string Say(const std::string& lang, const std::string& id, const Vars& vars = {});

// The name of a reason of an event as the catalog gives it in lang, else message, else the code in small letters.
std::string ReasonName(const std::string& lang, const std::string& event, const std::string& code,
                       const std::string& message = "");

}  // namespace mowbite_net

#endif  // MOWBITE_NET_TEXTS_H
