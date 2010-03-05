/**
 *  \file Restraint.cpp   \brief Abstract base class for all restraints.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 *
 */

#include "IMP/container_base.h"
#include "IMP/internal/utility.h"

IMP_BEGIN_NAMESPACE

namespace {
  unsigned int restraint_index=0;
}

Container::Container(std::string name):
  Object(internal::make_object_name(name, restraint_index++))
{
}

IMP_END_NAMESPACE
