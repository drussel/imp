/**
 *  \file NormalMover.h
 *  \brief A modifier which perturbs a point with a normal distribution.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPCORE_NORMAL_MOVER_H
#define IMPCORE_NORMAL_MOVER_H

#include "core_config.h"
#include "MoverBase.h"

IMPCORE_BEGIN_NAMESPACE

//! Modify a set of continuous variables using a normal distribution.
/** \see MonteCarlo
 */
class IMPCOREEXPORT NormalMover :public MoverBase
{
public:
  /**  \param[in] sc The set of particles to perturb.
       \param[in] vars The variables to use (normally the keys for x,y,z)
       \param[in] sigma The standard deviation to use.
   */
  NormalMover(SingletonContainer *sc,
              const FloatKeys &vars,
              Float sigma);
  void set_sigma(Float sigma) {
    IMP_USAGE_CHECK(sigma > 0, "Sigma must be positive");
    stddev_=sigma;
  }
  Float get_sigma() const {
    return stddev_;
  }
  IMP_OBJECT(NormalMover);
private:
  virtual void do_move(Float f);
  Float stddev_;
};

IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_NORMAL_MOVER_H */
