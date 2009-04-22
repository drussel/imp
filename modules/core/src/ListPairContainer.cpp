/**
 *  \file ListPairContainer.cpp   \brief A list of ParticlePairs.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 *
 */

#include "IMP/core/ListPairContainer.h"
#include <algorithm>


IMPCORE_BEGIN_NAMESPACE

ListPairContainer::ListPairContainer(const ParticlePairs &ps){
  sorted_=false;
  set_particle_pairs(ps);
  set_is_editing(false);
}

IMP_LIST_IMPL(ListPairContainer, ParticlePair,
              particle_pair, ParticlePair,, {
                if (sorted_) std::sort(particle_pairs_begin(),
                                       particle_pairs_end());
              },);


void ListPairContainer::set_is_editing(bool tf) {
  if (tf== !sorted_) return;
  else {
    sorted_=!tf;
    if (sorted_) {
      std::sort(particle_pairs_begin(), particle_pairs_end());
    }
  }
}


bool
ListPairContainer::get_contains_particle_pair(ParticlePair vt) const {
  IMP_CHECK_OBJECT(this);
  return std::binary_search(particle_pairs_begin(), particle_pairs_end(), vt);
}

void ListPairContainer::show(std::ostream &out) const {
  IMP_CHECK_OBJECT(this);
  out << "ListPairContainer with " << get_number_of_particle_pairs()
      << " particle_pairs." << std::endl;
}

IMPCORE_END_NAMESPACE
