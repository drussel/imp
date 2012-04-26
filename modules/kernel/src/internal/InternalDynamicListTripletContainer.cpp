/**
 *  \file DynamicListTripletContainer.cpp
 *  \brief A list of ParticleTripletsTemp.
 *
 *  This file is generated by a script (internal/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#include "IMP/internal/InternalDynamicListTripletContainer.h"
#include "IMP/TripletModifier.h"
#include <IMP/base/check_macros.h>
#include <IMP/compatibility/set.h>
#include <algorithm>


IMP_BEGIN_INTERNAL_NAMESPACE

InternalDynamicListTripletContainer
::InternalDynamicListTripletContainer(Container *m,
                                        std::string name):
    P(m->get_model(), name), scope_(m) {
}


InternalDynamicListTripletContainer
::InternalDynamicListTripletContainer(Container *m,
                                        const char *name):
    P(m->get_model(), name), scope_(m) {
}


void InternalDynamicListTripletContainer::do_show(std::ostream &out) const {
  IMP_CHECK_OBJECT(this);
  out << get_number_of_particle_triplets()
      << " Triplets." << std::endl;
}



void InternalDynamicListTripletContainer
::remove_particle_triplets(const ParticleTripletsTemp &c) {
  if (c.empty()) return;
  get_model()->reset_dependencies();
  ParticleIndexTriplets cp= IMP::internal::get_index(c);
  remove_from_list(cp);
  IMP_IF_CHECK(base::USAGE) {
    for (unsigned int i=0; i< c.size(); ++i) {
      IMP_USAGE_CHECK(IMP::internal::is_valid(c[i]),
                    "Passed Triplet cannot be nullptr (or None)");
    }
  }
}

bool InternalDynamicListTripletContainer::
check_list(const ParticleIndexes& cp) const {
  ParticleIndexes app
    = IMP::internal::get_index(scope_->get_all_possible_particles());

  compatibility::set<ParticleIndex> all(app.begin(),
                                    app.end());
  for (unsigned int i=0; i< cp.size(); ++i) {
    IMP_USAGE_CHECK(all.find(cp[i]) != all.end(),
                    "Particle " << cp[i]
                    << " is not in the list of all possible particles");
  }
  return true;
}

ParticlesTemp
InternalDynamicListTripletContainer::get_all_possible_particles() const {
  return scope_->get_all_possible_particles();
}

void InternalDynamicListTripletContainer::do_before_evaluate() {
}


ParticlesTemp
InternalDynamicListTripletContainer::get_input_particles() const {
  return ParticlesTemp();
}

ContainersTemp
InternalDynamicListTripletContainer::get_input_containers() const {
  return ContainersTemp();
}


ParticleIndexTriplets
InternalDynamicListTripletContainer::get_all_possible_indexes() const {
  IMP_NOT_IMPLEMENTED;
  return ParticleIndexTriplets();
}


IMP_END_INTERNAL_NAMESPACE
