/**
 *  \file Mover.h    \brief The base class for movers for MC optimization.
 *
 *  Copyright 2007-2010 Sali Lab. All rights reserved.
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

//! A base class for classes which pertub particles.
/** Mover objects are designed primarily to be used with
    the Monte Carlo optimizer. You probably want to use MoverBase
    if you are implementing a Mover.
    \see MonteCarlo
 */
class IMPCOREEXPORT Mover: public Object
{
  friend class MonteCarlo;
  void set_optimizer(Optimizer *c) {
    if (c) set_was_used(true);
    opt_=c;
  }

  WeakPointer<Optimizer> opt_;
public:
  Mover(std::string name=std::string("Mover"));

  //! propose a modification
  /** \param[in] size A number between 0 and 1 used to scale the proposed
      moves. This number can be either used to scale a continuous move
      or affect the probability of a discrete move.
   */
  virtual void propose_move(Float size)=0;
  //! set whether the proposed modification is accepted
  /** \note Accepting should not change the Particles at all.
   */
  virtual void accept_move()=0;

  //! Roll back any changes made to the Particles
  virtual void reject_move()=0;

  //! Get a pointer to the optimizer which has this mover.
  Optimizer *get_optimizer() const {
    IMP_CHECK_OBJECT(this);
    return opt_;
  }
  virtual void show(std::ostream&out= std::cout) const {
    out << "Mover doesn't implement show " << std::endl;
  }

  IMP_REF_COUNTED_DESTRUCTOR(Mover);
};

IMP_OUTPUT_OPERATOR(Mover);


IMP_OBJECTS(Mover);
/** \objects{Mover}
*/
/** \objectstemp{Mover}
*/
IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_MOVER_H */
