/**
 *  \file internal/utility.h
 *  \brief Various useful utilities
 *
 *  Copyright 2007-2013 IMP Inventors. All rights reserved.
 */

#ifndef IMPKERNEL_INTERNAL_STATIC_H
#define IMPKERNEL_INTERNAL_STATIC_H

#include <IMP/kernel/kernel_config.h>
#include <IMP/base/map.h>
#include <IMP/base/Pointer.h>
#include <IMP/kernel/Undecorator.h>




IMPKERNEL_BEGIN_INTERNAL_NAMESPACE

extern IMPKERNELEXPORT
base::map<std::string, base::Pointer<Undecorator> > undecorators;


IMPKERNEL_END_INTERNAL_NAMESPACE

#endif  /* IMPKERNEL_INTERNAL_STATIC_H */
