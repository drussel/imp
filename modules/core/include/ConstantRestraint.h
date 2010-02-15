/**
 *  \file ConstantRestraint.h    \brief Constant restraint.
 *
 *  Just return a constant.
 *
 *  Copyright 2007-2010 Sali Lab. All rights reserved.
 *
 */

#ifndef IMPCORE_CONSTANT_RESTRAINT_H
#define IMPCORE_CONSTANT_RESTRAINT_H

#include "config.h"

#include <IMP/Restraint.h>
#include <IMP/PairScore.h>

IMPCORE_BEGIN_NAMESPACE

//! Return a constant value.
/** This restraint is mostly for testing, but can also be used to make
    the total score look nicer.
 */
class IMPCOREEXPORT ConstantRestraint : public Restraint
{
  Float v_;
public:
  //! Add v to the total score.
  ConstantRestraint(Float v);

  IMP_RESTRAINT(ConstantRestraint)
};

IMPCORE_END_NAMESPACE

#endif /* IMPCORE_CONSTANT_RESTRAINT_H */
