/**
 *  \file SphereDistancePairScore.h
 *  \brief A score on the distance between the surfaces of two spheres.
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 */

#ifndef IMPCORE_SPHERE_DISTANCE_PAIR_SCORE_H
#define IMPCORE_SPHERE_DISTANCE_PAIR_SCORE_H

#include "config.h"
#include <IMP/PairScore.h>
#include <IMP/Pointer.h>
#include <IMP/UnaryFunction.h>

IMPCORE_BEGIN_NAMESPACE

//! A score on the distance between the surfaces of two spheres.
/** \see XYZR
    \see DistancePairScore
 */
class IMPCOREEXPORT SphereDistancePairScore : public PairScore
{
  Pointer<UnaryFunction> f_;
  FloatKey radius_;
public:
  /** */
  SphereDistancePairScore(UnaryFunction *f,
                          FloatKey radius=FloatKey("radius"));
  IMP_PAIR_SCORE(SphereDistancePairScore, get_module_version_info());
};

IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_SPHERE_DISTANCE_PAIR_SCORE_H */
