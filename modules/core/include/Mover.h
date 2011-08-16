/**
 *  \file Mover.h    \brief The base class for movers for MC optimization.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPCORE_MOVER_H
#define IMPCORE_MOVER_H

#include "core_config.h"

#include <IMP/base_types.h>
#include <IMP/RefCounted.h>
#include <IMP/WeakPointer.h>
#include <IMP/Optimizer.h>

#include <vector>

IMPCORE_BEGIN_NAMESPACE

class MonteCarlo;

//! A base class for classes which perturb particles.
/** Mover objects are designed primarily to be used with
    the Monte Carlo optimizer. You probably want to use MoverBase
    if you are implementing a Mover.
    \see MonteCarlo
 */
class IMPCOREEXPORT Mover: public Object
{
  friend class MonteCarlo;
  WeakPointer<Optimizer> opt_;

/* Older versions of g++ do not extend friendship to
   MonteCarlo::MoverDataWrapper, and thus fail to compile MonteCarlo.h. */
#if defined(__GNUC__) && (__GNUC__ < 4                                  \
                          || defined(__GNUC_MINOR)                      \
                          && (__GNUC__ == 4 && __GNUC_MINOR < 1))
public:
#endif
  void set_optimizer(Optimizer *c) {
    if (c) set_was_used(true);
    opt_=c;
  }
public:
  Mover(std::string name=std::string("Mover %1%"));

  //! propose a modification
  /** \param[in] size A number between 0 and 1 used to scale the proposed
      moves. This number can be either used to scale a continuous move
      or affect the probability of a discrete move.

      The method should return the list of all particles that were
      actually moved.
   */
  virtual ParticlesTemp propose_move(Float size)=0;

  //! Roll back any changes made to the Particles
  virtual void reset_move()=0;

  //! Get a pointer to the optimizer which has this mover.
  Optimizer *get_optimizer() const {
    IMP_CHECK_OBJECT(this);
    return opt_;
  }

  IMP_REF_COUNTED_DESTRUCTOR(Mover);
};

IMP_OBJECTS(Mover,Movers);

IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_MOVER_H */
