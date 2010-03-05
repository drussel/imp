/**
 *  \file GroupnameModifier.cpp   \brief A function on Particles.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 *
 */

#include "IMP/GroupnameModifier.h"
#include "IMP/internal/utility.h"

IMP_BEGIN_NAMESPACE

namespace {
  unsigned int next_index=0;
}

GroupnameModifier::GroupnameModifier(std::string name):
  Object(internal::make_object_name(name, next_index++)){
}

IMP_END_NAMESPACE
