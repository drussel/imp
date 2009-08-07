/**
 *  \file ChildrenRefiner.cpp
 *  \brief Return the hierarchy children of a particle.
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 */

#include <IMP/core/ChildrenRefiner.h>

#include <IMP/core/Hierarchy.h>

IMPCORE_BEGIN_NAMESPACE

ChildrenRefiner
::ChildrenRefiner(HierarchyTraits traits): traits_(traits)
{
}


bool ChildrenRefiner::get_can_refine(Particle *p) const
{
  if (!core::Hierarchy::particle_is_instance(p, traits_)) return false;
  return core::Hierarchy(p, traits_).get_number_of_children() != 0;

}

Particle* ChildrenRefiner::get_refined(Particle *p, unsigned int i) const
{
  IMP_assert(get_can_refine(p), "Trying to refine the unrefinable");
  core::Hierarchy d(p, traits_);
  return d.get_child(i).get_particle();
}


unsigned int ChildrenRefiner::get_number_of_refined(Particle *p) const
{
  IMP_assert(get_can_refine(p), "Trying to refine the unrefinable");
  core::Hierarchy d(p, traits_);
  return d.get_number_of_children();
}

const ParticlesTemp ChildrenRefiner::get_refined(Particle *p) const {
    Hierarchy hd(p, traits_);
    ParticlesTemp ret(hd.get_number_of_children());
    for (unsigned int i=0; i< ret.size(); ++i) {
      ret[i]= hd.get_child(i);
    }
    return ret;
  }


void ChildrenRefiner::show(std::ostream &out) const
{
  out << "ChildrenRefiner" << std::endl;
}

IMPCORE_END_NAMESPACE
