/**
 *  \file ChildrenParticleRefiner.cpp
 *  \brief Return the hierarchy children of a particle.
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 */

#include <IMP/core/ChildrenParticleRefiner.h>

#include <IMP/core/HierarchyDecorator.h>

IMPCORE_BEGIN_NAMESPACE

ChildrenParticleRefiner
::ChildrenParticleRefiner(HierarchyTraits traits): traits_(traits)
{
}


bool ChildrenParticleRefiner::get_can_refine(Particle *p) const
{
  if (!core::HierarchyDecorator::is_instance_of(p, traits_)) return false;
  return core::HierarchyDecorator(p, traits_).get_number_of_children() != 0;

}

Particles ChildrenParticleRefiner::get_refined(Particle *p) const
{
  IMP_assert(get_can_refine(p), "Trying to refine the unrefinable");
  core::HierarchyDecorator d(p, traits_);
  Particles ps(d.get_number_of_children());
  for (unsigned int i=0; i< d.get_number_of_children(); ++i){
    ps[i]= d.get_child(i).get_particle();
  }
  return ps;
}



void ChildrenParticleRefiner::cleanup_refined(Particle *,
                                              Particles &,
                                              DerivativeAccumulator *) const
{
  // This space left intentionally blank
}

void ChildrenParticleRefiner::show(std::ostream &out) const
{
  out << "ChildrenParticleRefiner" << std::endl;
}

IMPCORE_END_NAMESPACE
