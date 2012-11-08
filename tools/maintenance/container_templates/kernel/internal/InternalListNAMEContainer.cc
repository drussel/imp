/**
 *  \file ListCLASSNAMEContainer.cpp   \brief A list of PLURALVARIABLETYPE.
 *
 *  This file is generated by a script (internal/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#include "IMP/internal/InternalListCLASSNAMEContainer.h"
#include "IMP/CLASSNAMEModifier.h"
#include <IMP/base/check_macros.h>
#include <algorithm>


IMP_BEGIN_INTERNAL_NAMESPACE

InternalListCLASSNAMEContainer
::InternalListCLASSNAMEContainer(Model *m, std::string name):
  P(m, name){
}


InternalListCLASSNAMEContainer
::InternalListCLASSNAMEContainer(Model *m, const char *name):
  P(m, name){
}
void InternalListCLASSNAMEContainer::add(PASSINDEXTYPE vt) {
  get_model()->clear_caches();
  PLURALINDEXTYPE cur;
  swap(cur);
  cur.push_back(vt);
  swap(cur);
}
void InternalListCLASSNAMEContainer
::add(const PLURALINDEXTYPE &c) {
  if (c.empty()) return;
  get_model()->clear_caches();
  PLURALINDEXTYPE cur;
  swap(cur);
  cur+=c;
  swap(cur);
}
void InternalListCLASSNAMEContainer::set(PLURALINDEXTYPE cp) {
  get_model()->clear_caches();
  swap(cp);
}
void InternalListCLASSNAMEContainer::clear() {
  get_model()->clear_caches();
  PLURALINDEXTYPE t;
  swap(t);
}
void InternalListCLASSNAMEContainer::remove(PASSINDEXTYPE vt) {
  get_model()->clear_caches();
  PLURALINDEXTYPE t;
  swap(t);
  t.erase(std::remove(t.begin(), t.end(), vt), t.end());
  swap(t);
}
void InternalListCLASSNAMEContainer::do_show(std::ostream &out) const {
  IMP_CHECK_OBJECT(this);
  out << get_access()
      << " CLASSNAMEs." << std::endl;
}


ParticlesTemp
InternalListCLASSNAMEContainer::get_all_possible_particles() const {
  return IMP::internal::flatten(get());
}

PLURALINDEXTYPE
InternalListCLASSNAMEContainer::get_range_indexes() const {
  return get_indexes();
}

void InternalListCLASSNAMEContainer::do_before_evaluate() {
}


ParticlesTemp
InternalListCLASSNAMEContainer::get_input_particles() const {
  return ParticlesTemp();
}

ContainersTemp
InternalListCLASSNAMEContainer::get_input_containers() const {
  return ContainersTemp();
}

IMP_END_INTERNAL_NAMESPACE
