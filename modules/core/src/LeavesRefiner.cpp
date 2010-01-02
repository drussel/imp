/**
 *  \file LeavesRefiner.cpp
 *  \brief Return the hierarchy children of a particle.
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 */

#include <IMP/core/LeavesRefiner.h>

#include <IMP/core/Hierarchy.h>

IMPCORE_BEGIN_NAMESPACE

LeavesRefiner
::LeavesRefiner(HierarchyTraits traits):
  Refiner("LeavesRefiner%d"), traits_(traits)
{
}


bool LeavesRefiner::get_can_refine(Particle *p) const
{
  if (!core::Hierarchy::particle_is_instance(p, traits_)) return false;
  if (core::Hierarchy(p, traits_).get_number_of_children() != 0) {
    cache_[p]=get_leaves(core::Hierarchy(p, traits_));
    return true;
  } else {
    return false;
  }

}

Particle* LeavesRefiner::get_refined(Particle *p, unsigned int i) const
{
  // force filling of the cache, yeah, its not good organization
  get_can_refine(p);
  IMP_INTERNAL_CHECK(get_can_refine(p), "Trying to refine the unrefinable");
  return cache_[p][i];
}

unsigned int LeavesRefiner::get_number_of_refined(Particle *p) const
{
  // force filling of the cache, yeah, its not good organization
  get_can_refine(p);
  IMP_INTERNAL_CHECK(get_can_refine(p), "Trying to refine the unrefinable");
  return cache_[p].size();
}



const ParticlesTemp LeavesRefiner::get_refined(Particle *p) const{
  // force filling of the cache, yeah, its not good organization
  get_can_refine(p);
  IMP_INTERNAL_CHECK(get_can_refine(p), "Trying to refine the unrefinable");
  return ParticlesTemp(cache_[p].begin(), cache_[p].end());
}

namespace {
  struct Yes {
    bool operator()(Particle *p) {
      return true;
    }
  };
}

ParticlesTemp LeavesRefiner::get_input_particles(Particle *p) const {
  return ParticlesTemp();
}

ContainersTemp LeavesRefiner::get_input_containers(Particle *p) const {
  GenericHierarchiesTemp t=get_all_descendants(Hierarchy(p, traits_));
  return ContainersTemp(t.begin(), t.end());
}


void LeavesRefiner::show(std::ostream &out) const
{
  out << "LeavesRefiner" << std::endl;
}

IMPCORE_END_NAMESPACE
