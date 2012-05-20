/**
 *  \file compatibility/nullptr.h
 *  \brief Provide a nullptr keyword analog.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */

#ifndef IMPCOMPATIBILITY_NULLPTR_H
#define IMPCOMPATIBILITY_NULLPTR_H

#include "compatibility_config.h"

namespace IMP {
#ifdef IMP_DOXYGEN
/** The C++0x standard adds the nullptr keyword to get around a variety of
    problems with NULL. We provide an emulation within the IMP namespace when
    it is not available.

    Use "nullptr" in code; the compiler will use our IMP::nullptr emulation
    automatically on systems that don't provide a native nullptr implementation.

    If you are not in the IMP namespace, use the IMP_NULLPTR macro rather
    than asking for "IMP::nullptr". The latter does not work with some compilers
    (e.g. MSVC, which gets confused because nullptr is a keyword).
*/
const std::nullptr_t nullptr;

#else

#if IMP_DEFINE_NULLPTR

#if !defined(SWIG)

#if __GNUC__ && __GNUC__==4 && __GNUC_MINOR__>=6
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wc++0x-compat"
#endif


struct nullptr_t {
  template <class O>
  operator O*() const {
    return static_cast<O*>(NULL);
  }
  /*template <class O, class C>
  operator O C::*() const {
    return static_cast<const O*>(NULL);
    }*/
};
template <class O>
inline bool operator==(O *a, nullptr_t o) {
  return a == static_cast<O*>(o);
}
template <class O>
inline bool operator!=(O *a, nullptr_t o) {
  return a != static_cast<O*>(o);
}
extern IMPCOMPATIBILITYEXPORT const nullptr_t nullptr;

#if __GNUC__ && __GNUC__==4 && __GNUC_MINOR__>=6
#pragma GCC diagnostic pop
#endif

#else
extern const void * const nullptr;
#endif //SWIG
#define IMP_NULLPTR IMP::nullptr
#else  // IMP_DEFINE_NULLPTR
#define IMP_NULLPTR nullptr
#endif // IMP_DEFINE_NULLPTR
#endif // IMP_DOXYGEN

}

#endif  /* IMPCOMPATIBILITY_NULLPTR_H */
