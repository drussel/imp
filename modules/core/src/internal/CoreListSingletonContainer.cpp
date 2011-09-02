/**
 *  \file ListSingletonContainer.cpp   \brief A list of ParticlesTemp.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 *
 */

#include "IMP/core/internal/CoreListSingletonContainer.h"
#include "IMP/SingletonModifier.h"
#include "IMP/SingletonScore.h"
#include <algorithm>


IMPCORE_BEGIN_INTERNAL_NAMESPACE


IMP_ACTIVE_CONTAINER_DEF(CoreListSingletonContainer,);


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
      << " Singletons." << std::endl;
}



void CoreListSingletonContainer
::remove_particles(const ParticlesTemp &c) {
  if (c.empty()) return;
  ParticleIndexes cp= IMP::internal::get_index(c);
  remove_from_list(cp);
  IMP_IF_CHECK(USAGE) {
    for (unsigned int i=0; i< c.size(); ++i) {
      IMP_USAGE_CHECK(IMP::internal::is_valid(c[i]),
                    "Passed Singleton cannot be nullptr (or None)");
    }
  }
}

ParticleIndexes
CoreListSingletonContainer::get_all_possible_indexes() const {
    return get_indexes();
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

IMPCORE_END_INTERNAL_NAMESPACE
