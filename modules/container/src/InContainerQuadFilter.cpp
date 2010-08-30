/**
 *  \file QuadFilter.cpp   \brief Filter for particle_quad.
 *
 *  This file is generated by a script (core/tools/make-filter).
 *  Do not edit directly.
 *
 *  Copyright 2007-2010 Sali Lab. All rights reserved.
 *
 */

#include "IMP/container/InContainerQuadFilter.h"

IMPCONTAINER_BEGIN_NAMESPACE


InContainerQuadFilter
::InContainerQuadFilter(QuadContainer *c,
                             std::string name): QuadFilter(name), c_(c){}

ParticlesTemp InContainerQuadFilter
::get_input_particles(const ParticleQuad&) const {
  // not quite right, but...
  return ParticlesTemp();
}
ContainersTemp InContainerQuadFilter
::get_input_containers(const ParticleQuad&) const {
  return ContainersTemp(1,c_);
}

void InContainerQuadFilter::do_show(std::ostream &out) const {
  out << "InContainerQuadFilter on " << *c_ << std::endl;
}

IMPCONTAINER_END_NAMESPACE
