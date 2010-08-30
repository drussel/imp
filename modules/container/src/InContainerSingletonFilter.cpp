/**
 *  \file SingletonFilter.cpp   \brief Filter for particle.
 *
 *  This file is generated by a script (core/tools/make-filter).
 *  Do not edit directly.
 *
 *  Copyright 2007-2010 Sali Lab. All rights reserved.
 *
 */

#include "IMP/container/InContainerSingletonFilter.h"

IMPCONTAINER_BEGIN_NAMESPACE


InContainerSingletonFilter
::InContainerSingletonFilter(SingletonContainer *c,
                             std::string name): SingletonFilter(name), c_(c){}

ParticlesTemp InContainerSingletonFilter
::get_input_particles(Particle*) const {
  // not quite right, but...
  return ParticlesTemp();
}
ContainersTemp InContainerSingletonFilter
::get_input_containers(Particle*) const {
  return ContainersTemp(1,c_);
}

void InContainerSingletonFilter::do_show(std::ostream &out) const {
  out << "InContainerSingletonFilter on " << *c_ << std::endl;
}

IMPCONTAINER_END_NAMESPACE
