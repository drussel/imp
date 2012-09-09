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


void InternalListCLASSNAMEContainer::do_show(std::ostream &out) const {
  IMP_CHECK_OBJECT(this);
  out << get_number_of_FUNCTIONNAMEs()
      << " CLASSNAMEs." << std::endl;
}



void InternalListCLASSNAMEContainer
::remove_FUNCTIONNAMEs(const PLURALVARIABLETYPE &c) {
  if (c.empty()) return;
  get_model()->clear_caches();
  PLURALINDEXTYPE cp= IMP::internal::get_index(c);
  remove_from_list(cp);
  IMP_IF_CHECK(base::USAGE) {
    for (unsigned int i=0; i< c.size(); ++i) {
      IMP_USAGE_CHECK(IMP::internal::is_valid(c[i]),
                    "Passed CLASSNAME cannot be nullptr (or None)");
    }
  }
}

ParticlesTemp
InternalListCLASSNAMEContainer::get_all_possible_particles() const {
  return IMP::internal::flatten(get());
}

PLURALINDEXTYPE
InternalListCLASSNAMEContainer::get_all_possible_indexes() const {
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
