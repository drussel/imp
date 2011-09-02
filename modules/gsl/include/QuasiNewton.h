/**
 *  \file QuasiNewton.h
 *  \brief A GSL-based Quasi-Newton optimizer
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 */

#ifndef IMPGSL_QUASI_NEWTON_H
#define IMPGSL_QUASI_NEWTON_H

#include "gsl_config.h"

#include "GSLOptimizer.h"

IMPGSL_BEGIN_NAMESPACE

//! A quasi-Newton optimizer taken from GSL
/** \untested{QuasiNewton}
 */
class IMPGSLEXPORT QuasiNewton: public GSLOptimizer
{
  double initial_step_, line_step_, min_gradient_;
public:
  QuasiNewton(Model *m=nullptr);

  IMP_OPTIMIZER(QuasiNewton);
};


IMPGSL_END_NAMESPACE

#endif  /* IMPGSL_QUASI_NEWTON_H */
