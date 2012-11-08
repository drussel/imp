/**
 *  \file DynamicListQuadContainer.cpp
 *  \brief A list of ParticleQuadsTemp.
 *
 *  This file is generated by a script (internal/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#include "IMP/internal/InternalDynamicListQuadContainer.h"
#include "IMP/QuadModifier.h"
#include <IMP/base/check_macros.h>
#include <IMP/compatibility/set.h>
#include <algorithm>


IMP_BEGIN_INTERNAL_NAMESPACE

InternalDynamicListQuadContainer
::InternalDynamicListQuadContainer(Container *m,
                                        std::string name):
    P(m->get_model(), name), scope_(m) {
}


InternalDynamicListQuadContainer
::InternalDynamicListQuadContainer(Container *m,
                                        const char *name):
    P(m->get_model(), name), scope_(m) {
}


void InternalDynamicListQuadContainer::do_show(std::ostream &out) const {
  IMP_CHECK_OBJECT(this);
  out << get_access()
      << " Quads." << std::endl;
}
void InternalDynamicListQuadContainer::add(const ParticleIndexQuad& vt) {
  ParticleIndexQuads cur;
  swap(cur);
  cur.push_back(vt);
  swap(cur);
}
void InternalDynamicListQuadContainer
::add(const ParticleIndexQuads &c) {
  if (c.empty()) return;
  ParticleIndexQuads cur;
  swap(cur);
  cur+=c;
  swap(cur);
}

void InternalDynamicListQuadContainer::set(ParticleIndexQuads cp) {
  swap(cp);
}
void InternalDynamicListQuadContainer::clear() {
  ParticleIndexQuads t;
  swap(t);
}
bool InternalDynamicListQuadContainer::
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
InternalDynamicListQuadContainer::get_all_possible_particles() const {
  return scope_->get_all_possible_particles();
}

void InternalDynamicListQuadContainer::do_before_evaluate() {
}


ParticlesTemp
InternalDynamicListQuadContainer::get_input_particles() const {
  return ParticlesTemp();
}

ContainersTemp
InternalDynamicListQuadContainer::get_input_containers() const {
  return ContainersTemp();
}


ParticleIndexQuads
InternalDynamicListQuadContainer::get_range_indexes() const {
  return get_indexes();
}


IMP_END_INTERNAL_NAMESPACE
