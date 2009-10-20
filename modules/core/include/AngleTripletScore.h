/**
 *  \file AngleTripletScore.h
 *  \brief A Score on the angle between three of particles.
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 */

#ifndef IMPCORE_ANGLE_TRIPLET_SCORE_H
#define IMPCORE_ANGLE_TRIPLET_SCORE_H

#include "config.h"
#include <IMP/TripletScore.h>
#include <IMP/UnaryFunction.h>
#include <IMP/Pointer.h>

IMPCORE_BEGIN_NAMESPACE

//! Apply a function to the angle between three particles.
/** */
class IMPCOREEXPORT AngleTripletScore : public TripletScore
{
  IMP::internal::OwnerPointer<UnaryFunction> f_;
public:
  //! Score the angle (in radius) using f
  AngleTripletScore(UnaryFunction *f);
  virtual ~AngleTripletScore(){}
  virtual Float evaluate(Particle *a, Particle *b, Particle *c,
                         DerivativeAccumulator *da) const;
  virtual void show(std::ostream &out=std::cout) const;
  VersionInfo get_version_info() const {
    return get_module_version_info();
  }
};

IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_ANGLE_TRIPLET_SCORE_H */
