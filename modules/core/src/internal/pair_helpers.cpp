/**
 *  \file ListPairContainer.cpp   \brief A list of ParticlePairsTemp.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 *
 */

#include <IMP/core/internal/pair_helpers.h>
#include <IMP/PairModifier.h>
#include <IMP/PairScore.h>

IMPCORE_BEGIN_INTERNAL_NAMESPACE

void ListLikePairContainer
::do_show(std::ostream &out) const {
  out << "contains " << data_.size() << std::endl;
}



IMPCORE_END_INTERNAL_NAMESPACE
