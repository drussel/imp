/**
 *  \file IMP/atom/BondedPairFilter.h
 *  \brief A fake container for bonds
 *
 *  Copyright 2007-2013 IMP Inventors. All rights reserved.
 */

#ifndef IMPATOM_BONDED_PAIR_FILTER_H
#define IMPATOM_BONDED_PAIR_FILTER_H

#include <IMP/atom/atom_config.h>
#include "bond_decorators.h"
#include <IMP/PairPredicate.h>
#include <IMP/pair_macros.h>
IMPATOM_BEGIN_NAMESPACE

//! A filter for bonds.
/** This is to be used with a core::ClosePairsScoreState to exclude all
    bonded pairs.
    \ingroup bond
    \see Bonded
 */
class IMPATOMEXPORT BondedPairFilter : public PairPredicate
{
public:
  //! no arguments
  BondedPairFilter();

  IMP_PAIR_PREDICATE(BondedPairFilter);
};

IMP_OBJECTS(BondedPairFilter,BondedPairFilters);

IMPATOM_END_NAMESPACE

#endif  /* IMPATOM_BONDED_PAIR_FILTER_H */
