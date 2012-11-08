/**
 *  \file DynamicListSingletonContainer.cpp
 *  \brief A list of ParticlesTemp.
 *
 *  This file is generated by a script (internal/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#include "IMP/internal/InternalDynamicListSingletonContainer.h"
#include "IMP/SingletonModifier.h"
#include <IMP/base/check_macros.h>
#include <IMP/compatibility/set.h>
#include <algorithm>


IMP_BEGIN_INTERNAL_NAMESPACE

InternalDynamicListSingletonContainer
::InternalDynamicListSingletonContainer(Container *m,
                                        std::string name):
    P(m->get_model(), name), scope_(m) {
}


InternalDynamicListSingletonContainer
::InternalDynamicListSingletonContainer(Container *m,
                                        const char *name):
    P(m->get_model(), name), scope_(m) {
}


void InternalDynamicListSingletonContainer::do_show(std::ostream &out) const {
  IMP_CHECK_OBJECT(this);
  out << get_access()
      << " Singletons." << std::endl;
}
void InternalDynamicListSingletonContainer::add(ParticleIndex vt) {
  ParticleIndexes cur;
  swap(cur);
  cur.push_back(vt);
  swap(cur);
}
void InternalDynamicListSingletonContainer
::add(const ParticleIndexes &c) {
  if (c.empty()) return;
  ParticleIndexes cur;
  swap(cur);
  cur+=c;
  swap(cur);
}

void InternalDynamicListSingletonContainer::set(ParticleIndexes cp) {
  swap(cp);
}
void InternalDynamicListSingletonContainer::clear() {
  ParticleIndexes t;
  swap(t);
}
bool InternalDynamicListSingletonContainer::
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
InternalDynamicListSingletonContainer::get_all_possible_particles() const {
  return scope_->get_all_possible_particles();
}

void InternalDynamicListSingletonContainer::do_before_evaluate() {
}


ParticlesTemp
InternalDynamicListSingletonContainer::get_input_particles() const {
  return ParticlesTemp();
}

ContainersTemp
InternalDynamicListSingletonContainer::get_input_containers() const {
  return ContainersTemp();
}


ParticleIndexes
InternalDynamicListSingletonContainer::get_range_indexes() const {
  return get_indexes();
}


IMP_END_INTERNAL_NAMESPACE
