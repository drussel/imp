/**
 *  \file SingletonModifier.cpp   \brief A function on Particles.
 *
 *  WARNING This file was generated from NAMEModifier.cc
 *  in tools/maintenance/container_templates/kernel
 *  by tools/maintenance/make-container.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#include "IMP/SingletonModifier.h"
#include "IMP/internal/utility.h"
#include "IMP/ModelObject.h"

IMP_BEGIN_NAMESPACE

SingletonModifier::SingletonModifier(std::string name):
  Object(name){
}

IMP_INPUTS_DEF(SingletonModifier);
IMP_OUTPUTS_DEF(SingletonModifier);

IMP_END_NAMESPACE
