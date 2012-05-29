/**
 *  \file internal/deprecation.h
 *  \brief Macros to mark a class as deprecated.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPKERNEL_INTERNAL_DEPRECATION_H
#define IMPKERNEL_INTERNAL_DEPRECATION_H

#include "../kernel_config.h"

IMP_BEGIN_INTERNAL_NAMESPACE

IMPEXPORT bool get_print_deprecation_message(std::string name);

IMPEXPORT void set_printed_deprecation_message(std::string name, bool tr);

IMP_END_INTERNAL_NAMESPACE

#endif /* IMPKERNEL_INTERNAL_DEPRECATION_H */
