#pragma once
#include <stdexcept>

#define GB_STRINGIFY_(x) #x
#define GB_STRINGIFY(x) GB_STRINGIFY_(x)

/**
 * Always throws std::logic_error, tagging file & line.
 * Usage: UNIMPLEMENTED(); // never returns
 */

#define UNIMPLEMENTED() \
  throw std::logic_error("UNIMPLEMENTED: " __FILE__ ":" GB_STRINGIFY(__LINE__))