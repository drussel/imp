/**
 *  \file CentroidOfRefined.h
 *  \brief Set the coordinates of the particle to be the centoid of the
 *  refined particles.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 */

#ifndef IMPCORE_CENTROID_OF_REFINED_H
#define IMPCORE_CENTROID_OF_REFINED_H

#include "core_config.h"
#include "XYZ.h"
#include <IMP/Refiner.h>
#include <IMP/macros.h>
#include <IMP/Pointer.h>
#include <IMP/Decorator.h>
#include <IMP/SingletonModifier.h>


IMPCORE_BEGIN_NAMESPACE

//! Set the coordinates of the particle to be the centoid of the particles.
/** An exception is thrown if there are no refined particles. The weight
    FloatKey can be FloatKey and then all the weights will be equal.

    \see CoverRefined
    \see DerivativesToRefined
    \see DerivativesFromRefined
 */
class IMPCOREEXPORT CentroidOfRefined:
public SingletonModifier
{
  IMP::internal::OwnerPointer<Refiner> refiner_;
  FloatKeys ks_;
  FloatKey w_;
public:
  //! Set the keys ks to be the average of the refined particles.
  CentroidOfRefined(Refiner *r,
                    FloatKey weight=FloatKey(),
                    FloatKeys ks
                    = XYZ::get_xyz_keys());

  IMP_SINGLETON_MODIFIER(CentroidOfRefined);
};

//! A particle that is the centroid of other particles.
/** A decorator which sets up a particle to be the centroid of a
    set of other particles.

    \usesconstraint
*/
IMP_SUMMARY_DECORATOR_DECL(Centroid, XYZ, XYZs);

IMP_DECORATORS(Centroid, XYZs);

IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_CENTROID_OF_REFINED_H */
