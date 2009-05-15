/**
 *  \file LeavesRefiner.h
 *  \brief Return the hierarchy children of a particle.
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 */

#ifndef IMPCORE_LEAVES_REFINER_H
#define IMPCORE_LEAVES_REFINER_H

#include "config.h"
#include "internal/version_info.h"
#include "Hierarchy.h"
#include <IMP/atom/MolecularHierarchy.h>

#include <IMP/Refiner.h>

IMPCORE_BEGIN_NAMESPACE

class HierarchyTraits;

//! Return the hierarchy leaves under a particle.
/** \ingroup hierarchy
    \see Hierarchy
    \see MolecularHierarchy
*/
class IMPCOREEXPORT LeavesRefiner : public Refiner
{
  mutable std::map<Particle*, Particles> cache_;
  HierarchyTraits traits_;
public:
  //! Create a refiner for a particular type of hierarchy
  LeavesRefiner(HierarchyTraits tr);

  IMP_REFINER(LeavesRefiner, internal::version_info);
};

IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_LEAVES_REFINER_H */
