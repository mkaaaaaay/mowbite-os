// mowbite_setup_check <image folder>: the step list as JSON on stdin and the images it names, by the rules the node
// holds them to. Prints what is wrong, one line each; exit 1 with a problem.
#include <iostream>
#include <iterator>
#include <string>

#include "mowbite_setup/steps.h"

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "usage: mowbite_setup_check <image folder> < steps.json\n";
    return 2;
  }
  const std::string text((std::istreambuf_iterator<char>(std::cin)), std::istreambuf_iterator<char>());
  std::string why;
  const auto list = mowbite_setup::ParseSteps(mowbite_setup::json::parse(text, nullptr, false), &why);
  if (!list) {
    std::cout << why << "\n";
    return 1;
  }
  int problems = 0;
  for (const std::string& name : list->Images()) {
    std::string bytes;
    const std::string wrong = mowbite_setup::LoadImage(argv[1], name, &bytes);
    if (wrong.empty()) continue;
    std::cout << name << ": " << wrong << "\n";
    ++problems;
  }
  return problems == 0 ? 0 : 1;
}
