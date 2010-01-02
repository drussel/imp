/**
 *  \file ListSingletonContainer.cpp   \brief A list of Particles.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 *
 */

#include "IMP/core/ListSingletonContainer.h"
#include "IMP/SingletonModifier.h"
#include "IMP/SingletonScore.h"
#include <IMP/core/internal/singleton_helpers.h>
#include <algorithm>


IMPCORE_BEGIN_NAMESPACE

namespace {
  ListSingletonContainer* get_list(SingletonContainer *g) {
    return dynamic_cast<ListSingletonContainer*>(g);
  }
}

IMP_ACTIVE_CONTAINER_DEF(ListSingletonContainer);

ListSingletonContainer
::ListSingletonContainer(bool):
  internal::ListLikeSingletonContainer("added or removed list"){}

ListSingletonContainer
::ListSingletonContainer(const Particles &ps,
                         std::string name):
  internal::ListLikeSingletonContainer(name)
{
  if (ps.empty()) return;
  for (unsigned int i=0; i< ps.size(); ++i) {
    IMP_USAGE_CHECK(IMP::internal::is_valid(ps[i]),
                    "Passed Particle cannot be NULL (or None)",
                    UsageException);
    IMP_USAGE_CHECK(IMP::internal::get_model(ps[i])
                    == IMP::internal::get_model(ps[0]),
                    "All particles in container must have the same model. "
                    << "Particle " << IMP::internal::get_name(ps[i])
                    << " does not.",
                    Particle*Exception);
  }
  set_particles(ps);
}

ListSingletonContainer
::ListSingletonContainer(std::string name):
  internal::ListLikeSingletonContainer(name){
}

ListSingletonContainer
::ListSingletonContainer(const char *name):
  internal::ListLikeSingletonContainer(name){
}

void ListSingletonContainer::show(std::ostream &out) const {
  IMP_CHECK_OBJECT(this);
  out << "ListSingletonContainer with " << get_number_of_particles()
      << " particles." << std::endl;
}



void ListSingletonContainer::set_particles(ParticlesTemp sc) {
  if (!get_has_model() && !get_is_added_or_removed_container()
      && !sc.empty()) {
    set_model(IMP::internal::get_model(sc[0]));
  }
  update_list(sc);
}


void ListSingletonContainer::clear_particles() {
  ParticlesTemp t;
  update_list(t);
}


void ListSingletonContainer::add_particle(Particle* vt) {
  IMP_USAGE_CHECK(IMP::internal::is_valid(vt),
                  "Passed Particle cannot be NULL (or None)",
                  UsageException);

  if (!get_has_model() && !get_is_added_or_removed_container()) {
    set_model(IMP::internal::get_model(vt));
  }
  add_to_list(vt);
  IMP_USAGE_CHECK(get_is_added_or_removed_container()
                  || !get_removed_singletons_container()
                  ->get_contains(vt),
                  "You cannot remove and add the same item in one time step.",
                  Particle*Exception);
}

void
ListSingletonContainer::add_particles(const ParticlesTemp &c) {
  if (c.empty()) return;
  if (!get_has_model() && !get_is_added_or_removed_container()) {
    set_model(IMP::internal::get_model(c[0]));
  }
  ParticlesTemp cp= c;
  add_to_list(cp);
  IMP_IF_CHECK(USAGE) {
    for (unsigned int i=0; i< c.size(); ++i) {
      IMP_USAGE_CHECK(IMP::internal::is_valid(c[i]),
                    "Passed Particle cannot be NULL (or None)",
                    UsageException);
      IMP_USAGE_CHECK(get_is_added_or_removed_container()
                      || !get_removed_singletons_container()
                      ->get_contains(c[i]),
            "You cannot remove and add the same item in one time step.",
                      Particle*Exception);

    }
  }
}


ContainersTemp ListSingletonContainer::get_input_containers() const {
  return ContainersTemp();
}


void ListSingletonContainer::do_before_evaluate() {
  internal::ListLikeSingletonContainer::do_before_evaluate();
}

void ListSingletonContainer::do_after_evaluate() {
  internal::ListLikeSingletonContainer::do_after_evaluate();
}

ParticlesTemp ListSingletonContainer::get_state_input_particles() const {
  return ParticlesTemp();
}

ContainersTemp ListSingletonContainer::get_state_input_containers() const {
  return ContainersTemp();
}

IMPCORE_END_NAMESPACE
