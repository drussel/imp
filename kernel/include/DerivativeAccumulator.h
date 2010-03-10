/**
 *  \file DerivativeAccumulator.h   \brief Class for adding derivatives from
 *                                         restraints to the model.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMP_DERIVATIVE_ACCUMULATOR_H
#define IMP_DERIVATIVE_ACCUMULATOR_H

#include "kernel_config.h"
#include "base_types.h"
#include "utility.h"
#include "macros.h"
#include <vector>

IMP_BEGIN_NAMESPACE

//! Class for adding derivatives from restraints to the model.
/** This class was created so that restraints can be weighted using
    a RestraintSet and that the derivatives would be scaled appropriately */
class IMPEXPORT DerivativeAccumulator
{
public:
  //! the weight is one by default
  DerivativeAccumulator(Float weight=1.0)
      : weight_(weight) {}

  //! The weight is multiplied by the new weight
  DerivativeAccumulator(const DerivativeAccumulator &copy, Float weight=1.0)
      : weight_(copy.weight_ * weight) {}

  //! Scale a value appropriately.
  /** \param[in] value Value to add to the float attribute derivative.
   */
  Float operator()(const Float value) const {
    IMP_INTERNAL_CHECK(!is_nan(value), "Can't set derivative to NaN.");
    return value * weight_;
  }

private:
  Float weight_;
};

IMP_VALUES(DerivativeAccumulator, DerivativeAccumulators);

IMP_END_NAMESPACE

#endif  /* IMP_DERIVATIVE_ACCUMULATOR_H */
