/**
 *  \file SingletonRestraint.cpp
 *  \brief Implementation
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 *
 */

#include "IMP/core/SingletonRestraint.h"
#include <IMP/internal/container_helpers.h>

#include <IMP/log.h>


IMPCORE_BEGIN_NAMESPACE

SingletonRestraint
::SingletonRestraint(SingletonScore *ss,
                     Particle* vt,
                     std::string name):
  Restraint(name),
  ss_(ss),
  v_(vt),
  score_(std::numeric_limits<double>::quiet_NaN())
{
}

double SingletonRestraint
::unprotected_evaluate(DerivativeAccumulator *accum) const
{
  IMP_OBJECT_LOG;
  IMP_CHECK_OBJECT(ss_);
  score_ = ss_->evaluate(v_, accum);

  return score_;
}

double SingletonRestraint
::unprotected_incremental_evaluate(DerivativeAccumulator *accum) const
{
  if (IMP::internal::is_dirty(v_)) {
    score_+=ss_->evaluate_change(v_, accum);
  }
  return score_;
}

ParticlesList SingletonRestraint::get_interacting_particles() const
{
  return ss_->get_interacting_particles(v_);
}

ParticlesTemp SingletonRestraint::get_input_particles() const
{
  return ss_->get_input_particles(v_);
}

ContainersTemp SingletonRestraint::get_input_containers() const
{
  return ss_->get_input_containers(v_);
}

void SingletonRestraint::show(std::ostream& out) const
{
  out << "SingletonRestraint with score function ";
  ss_->show(out);
  out << " and " << v_;
  out << std::endl;
}

IMPCORE_END_NAMESPACE
