/**
 *  \file BallMover.h
 *  \brief A modifier which variables within a ball.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPCORE_INCREMENTAL_BALL_MOVER_H
#define IMPCORE_INCREMENTAL_BALL_MOVER_H

#include "core_config.h"
#include "MonteCarlo.h"
#include "MoverBase.h"
#include "core_macros.h"

IMPCORE_BEGIN_NAMESPACE

//! Modify a set of continuous variables by perturbing them within a ball.
/** This modifier is designed to be used when doing incremental updates. See
    BallMover for a variant that moves all at the same time.
    \see MonteCarlo
 */
class IMPCOREEXPORT IncrementalBallMover :public Mover
{
public:
  /** The attributes are perturbed within a ball whose dimensionality is
      given by the number of attributes and radius by the given value.
      \param[in] sc The set of particles to perturb.
      \param[in] n The number of points to move at the same time.
      \param[in] radius The radius deviation to use.
   */
  IncrementalBallMover(SingletonContainer *sc,
                       unsigned int n,
                       Float radius);
  IMP_MOVER(IncrementalBallMover);
private:
  IMP::internal::OwnerPointer<SingletonContainer> sc_;
  unsigned int n_;
  Float radius_;
  ParticlesTemp moved_;
};

IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_INCREMENTAL_BALL_MOVER_H */
