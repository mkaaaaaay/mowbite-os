#ifndef TRON_VISION_DYNAMIC_LIBRARY_H
#define TRON_VISION_DYNAMIC_LIBRARY_H

#include <dlfcn.h>

#include <string>

namespace tron_vision {

// dlopen of a vendor library that is never part of the build; nullptr with the reason in `error`.
inline void* OpenLibrary(const std::string& path, std::string& error) {
  void* library = dlopen(path.c_str(), RTLD_NOW | RTLD_LOCAL);
  if (library == nullptr) {
    const char* e = dlerror();  // NOLINT(concurrency-mt-unsafe): libraries are loaded on one thread
    error = "cannot load " + path + (e != nullptr ? std::string(": ") + e : std::string());
  }
  return library;
}

template <typename Fn>
bool Symbol(void* library, const char* name, Fn& fn, std::string& error) {
  fn = reinterpret_cast<Fn>(dlsym(library, name));
  if (fn == nullptr) {
    error = std::string("symbol ") + name + " missing";
    return false;
  }
  return true;
}

}  // namespace tron_vision

#endif  // TRON_VISION_DYNAMIC_LIBRARY_H
