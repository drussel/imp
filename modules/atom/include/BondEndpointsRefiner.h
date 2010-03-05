/**
 *  \file atom/BondEndpointsRefiner.h
 *  \brief Return the endpoints of a bond.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 */

#ifndef IMPATOM_BOND_ENDPOINTS_REFINER_H
#define IMPATOM_BOND_ENDPOINTS_REFINER_H

#include "atom_config.h"

#include <IMP/Refiner.h>

IMPATOM_BEGIN_NAMESPACE

//! Return the endpoints of a bond.
/**
 \ingroup bond
 \see Bond
 */
class IMPATOMEXPORT BondEndpointsRefiner : public Refiner
{
public:
  //! no arguments
  BondEndpointsRefiner();

  IMP_REFINER(BondEndpointsRefiner);
};

IMPATOM_END_NAMESPACE

#endif  /* IMPATOM_BOND_ENDPOINTS_REFINER_H */
