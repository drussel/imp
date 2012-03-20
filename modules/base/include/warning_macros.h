/**
 *  \file IMP/base/warning_macros.h
 *  \brief Various general useful macros for IMP.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPBASE_WARNING_MACROS_H
#define IMPBASE_WARNING_MACROS_H

#include "base_config.h"
#include "internal/eat.h"

#define IMP_REQUIRE_SEMICOLON_CLASS(Name)
#define IMP_REQUIRE_SEMICOLON_NAMESPACE
/*
#define IMP_REQUIRE_SEMICOLON_CLASS(Name)                       \
  IMP_NO_DOXYGEN(IMP_NO_SWIG(struct semicolon_##Name##_helper_struct{}))
#define IMP_REQUIRE_SEMICOLON_NAMESPACE void dummy_f()
*/


/** Disable unused variable warning for a variable.
 */
#define IMP_UNUSED(variable) IMP::base::internal::eat(variable)

#ifndef IMP_DOXYGEN
#ifdef __GNUC__
#define IMP_STRINGIFY(x) #x
#define IMP_WARN_PREPROCESS(msg) _Pragma(IMP_STRINGIFY(message #msg))

//#if __GNUC_PREREQ(4,2)
#define IMP_GCC_DISABLE_WARNING(name)\
_Pragma(IMP_STRINGIFY(GCC diagnostic ignored name))

/*#else
#define IMP_GCC_DISABLE_WARNING(name)
#endif*/

#else
#define IMP_GCC_DISABLE_WARNING(name)
#endif

#endif


#endif  /* IMPBASE_WARNING_MACROS_H */
