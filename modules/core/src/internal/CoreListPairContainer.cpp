/**
 *  \file ListPairContainer.cpp   \brief A list of ParticlePairs.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 *
 */

#include "IMP/core/internal/CoreListPairContainer.h"
#include "IMP/PairModifier.h"
#include "IMP/PairScore.h"
#include <IMP/core/internal/pair_helpers.h>
#include <algorithm>


IMPCORE_BEGIN_INTERNAL_NAMESPACE

namespace {
  CoreListPairContainer* get_list(PairContainer *g) {
    return dynamic_cast<CoreListPairContainer*>(g);
  }
}

IMP_ACTIVE_CONTAINER_DEF(CoreListPairContainer);

CoreListPairContainer
::CoreListPairContainer(bool):
  internal::ListLikePairContainer(){}


CoreListPairContainer
::CoreListPairContainer(std::string name):
  internal::ListLikePairContainer(name){}


CoreListPairContainer
::CoreListPairContainer(const char *name):
  internal::ListLikePairContainer(name){}


void CoreListPairContainer::do_show(std::ostream &out) const {
  IMP_CHECK_OBJECT(this);
  out << get_number_of_particle_pairs()
      << " particle_pairs." << std::endl;
}



void CoreListPairContainer::set_particle_pairs(ParticlePairsTemp sc) {
  if (!get_has_model() && !get_is_added_or_removed_container()
      && !sc.empty()) {
    set_model(IMP::internal::get_model(sc[0]));
  }
  update_list(sc);
}


void CoreListPairContainer::clear_particle_pairs() {
  ParticlePairsTemp t;
  update_list(t);
}


void CoreListPairContainer::add_particle_pair(const ParticlePair& vt) {
  IMP_USAGE_CHECK(IMP::internal::is_valid(vt),
                  "Passed ParticlePair cannot be NULL (or None)");

  if (!get_has_model() && !get_is_added_or_removed_container()) {
    set_model(IMP::internal::get_model(vt));
  }
  add_to_list(vt);
  IMP_USAGE_CHECK(get_is_added_or_removed_container()
                  || !get_removed_pairs_container()
                  ->get_contains(vt),
                  "You cannot remove and add the same item in one time step.");
}

void
CoreListPairContainer::add_particle_pairs(const ParticlePairsTemp &c) {
  if (c.empty()) return;
  if (!get_has_model() && !get_is_added_or_removed_container()) {
    set_model(IMP::internal::get_model(c[0]));
  }
  ParticlePairsTemp cp= c;
  add_to_list(cp);
  IMP_IF_CHECK(USAGE) {
    for (unsigned int i=0; i< c.size(); ++i) {
      IMP_USAGE_CHECK(IMP::internal::is_valid(c[i]),
                    "Passed ParticlePair cannot be NULL (or None)");
      IMP_USAGE_CHECK(get_is_added_or_removed_container()
                      || !get_removed_pairs_container()
                      ->get_contains(c[i]),
            "You cannot remove and add the same item in one time step.");

    }
  }
}


ContainersTemp CoreListPairContainer::get_input_containers() const {
  return ContainersTemp();
}


void CoreListPairContainer::do_before_evaluate() {
  internal::ListLikePairContainer::do_before_evaluate();
}

void CoreListPairContainer::do_after_evaluate() {
  internal::ListLikePairContainer::do_after_evaluate();
}

ParticlesTemp CoreListPairContainer::get_state_input_particles() const {
  return ParticlesTemp();
}

ContainersTemp CoreListPairContainer::get_state_input_containers() const {
  return ContainersTemp();
}

IMPCORE_END_INTERNAL_NAMESPACE
