/**
 *  \file ListSingletonContainer.cpp   \brief A list of Particles.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 *
 */

#include "IMP/core/internal/CoreListSingletonContainer.h"
#include "IMP/SingletonModifier.h"
#include "IMP/SingletonScore.h"
#include <IMP/core/internal/singleton_helpers.h>
#include <algorithm>


IMPCORE_BEGIN_INTERNAL_NAMESPACE

namespace {
  CoreListSingletonContainer* get_list(SingletonContainer *g) {
    return dynamic_cast<CoreListSingletonContainer*>(g);
  }
}

IMP_ACTIVE_CONTAINER_DEF(CoreListSingletonContainer);

CoreListSingletonContainer
::CoreListSingletonContainer():
  internal::ListLikeSingletonContainer(){}


CoreListSingletonContainer
::CoreListSingletonContainer(Model *m, std::string name):
  internal::ListLikeSingletonContainer(m, name){
  initialize_active_container(m);
}


CoreListSingletonContainer
::CoreListSingletonContainer(Model *m, const char *name):
  internal::ListLikeSingletonContainer(m, name){
  initialize_active_container(m);
}


void CoreListSingletonContainer::do_show(std::ostream &out) const {
  IMP_CHECK_OBJECT(this);
  out << get_number_of_particles()
      << " particles." << std::endl;
}



void CoreListSingletonContainer::set_particles(ParticlesTemp sc) {
  update_list(sc);
}


void CoreListSingletonContainer::clear_particles() {
  ParticlesTemp t;
  update_list(t);
}


void CoreListSingletonContainer::add_particle(Particle* vt) {
  IMP_USAGE_CHECK(IMP::internal::is_valid(vt),
                  "Passed Particle cannot be NULL (or None)");
  add_to_list(vt);
  IMP_USAGE_CHECK(!get_has_added_and_removed_containers()
                  || !get_removed_container()
                  ->get_contains(vt),
                  "You cannot remove and add the same item in one time step.");
}

void
CoreListSingletonContainer::add_particles(const ParticlesTemp &c) {
  if (c.empty()) return;
  ParticlesTemp cp= c;
  add_to_list(cp);
  IMP_IF_CHECK(USAGE) {
    for (unsigned int i=0; i< c.size(); ++i) {
      IMP_USAGE_CHECK(IMP::internal::is_valid(c[i]),
                    "Passed Particle cannot be NULL (or None)");
      IMP_USAGE_CHECK(!get_has_added_and_removed_containers()
                      || !get_removed_container()
                      ->get_contains(c[i]),
            "You cannot remove and add the same item in one time step.");

    }
  }
}

void CoreListSingletonContainer
::remove_particles(const ParticlesTemp &c) {
  if (c.empty()) return;
  ParticlesTemp cp= c;
  remove_from_list(cp);
  IMP_IF_CHECK(USAGE) {
    for (unsigned int i=0; i< c.size(); ++i) {
      IMP_USAGE_CHECK(IMP::internal::is_valid(c[i]),
                    "Passed Particle cannot be NULL (or None)");
    }
  }
}


void CoreListSingletonContainer::do_before_evaluate() {
  internal::ListLikeSingletonContainer::do_before_evaluate();
}

void CoreListSingletonContainer::do_after_evaluate() {
  internal::ListLikeSingletonContainer::do_after_evaluate();
}

ParticlesTemp CoreListSingletonContainer::get_state_input_particles() const {
  return ParticlesTemp();
}

ContainersTemp CoreListSingletonContainer::get_state_input_containers() const {
  return ContainersTemp();
}

bool CoreListSingletonContainer::get_contained_particles_changed() const {
  return !get_added()->get_access().empty()
    || !get_removed()->get_access().empty();
}
ParticlesTemp CoreListSingletonContainer::get_contained_particles() const {
  return IMP::internal::flatten(get_access());
}


IMPCORE_END_INTERNAL_NAMESPACE
