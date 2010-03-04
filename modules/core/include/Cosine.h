/**
 *  \file Cosine.h    \brief Cosine function.
 *
 *  Copyright 2007-2010 Sali Lab. All rights reserved.
 */

#ifndef IMPCORE_COSINE_H
#define IMPCORE_COSINE_H

#include "core_config.h"
#include <IMP/UnaryFunction.h>

IMPCORE_BEGIN_NAMESPACE

//! %Cosine function.
/** This evaluates the function
    |k| - k cos(nf + a)
    where k is a force constant, n the periodicity, a the phase, and f the
    input value. This is most commonly used for dihedral angle restraints,
    e.g. in the CHARMM force field.
 */
class IMPCOREEXPORT Cosine : public UnaryFunction
{
public:
  //! Constructor.
  /** \param[in] force_constant Force constant (score units)
      \param[in] periodicity Periodicity (generally 1-6)
      \param[in] phase Phase (radians)
   */
  Cosine(Float force_constant, int periodicity, Float phase) :
      force_constant_(force_constant), periodicity_(periodicity),
      phase_(phase) {}

  IMP_UNARY_FUNCTION(Cosine);
private:
  Float force_constant_;
  int periodicity_;
  Float phase_;
};

IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_COSINE_H */
