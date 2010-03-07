/**
 *  \file atom/BondedPairFilter.h
 *  \brief A fake container for bonds
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 */

#ifndef IMPATOM_BONDED_PAIR_FILTER_H
#define IMPATOM_BONDED_PAIR_FILTER_H

#include "atom_config.h"
#include "bond_decorators.h"
#include <IMP/PairFilter.h>

IMPATOM_BEGIN_NAMESPACE

//! A filter for bonds.
/** This is to be used with a core::ClosePairsScoreState to exclude all
    bonded pairs.
    \ingroup bond
    \see Bonded
 */
class IMPATOMEXPORT BondedPairFilter : public PairFilter
{
public:
  //! no arguments
  BondedPairFilter();

  IMP_PAIR_FILTER(BondedPairFilter);
};

IMP_OBJECTS(BondedPairFilter);

IMPATOM_END_NAMESPACE

#endif  /* IMPATOM_BONDED_PAIR_FILTER_H */
