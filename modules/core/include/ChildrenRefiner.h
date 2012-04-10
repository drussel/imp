/**
 *  \file ChildrenRefiner.h
 *  \brief Return the hierarchy children of a particle.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */

#ifndef IMPCORE_CHILDREN_REFINER_H
#define IMPCORE_CHILDREN_REFINER_H

#include "core_config.h"
#include "Hierarchy.h"

#include <IMP/Refiner.h>
#include <IMP/refiner_macros.h>

IMPCORE_BEGIN_NAMESPACE

class HierarchyTraits;

//! Return the hierarchy children of a particle.
/** \ingroup hierarchy
    A simple example using is
    \pythonexample{cover_particles}
    \see Hierarchy
    \see Hierarchy
*/
class IMPCOREEXPORT ChildrenRefiner : public Refiner
{

  HierarchyTraits traits_;
public:
  //! Create a refiner for a particular type of hierarchy
  ChildrenRefiner(HierarchyTraits tr);

  IMP_REFINER(ChildrenRefiner);
};

IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_CHILDREN_REFINER_H */
