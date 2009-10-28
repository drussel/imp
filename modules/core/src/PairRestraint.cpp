/**
 *  \file PairRestraint.cpp
 *  \brief Implementation
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 *
 */

#include "IMP/core/PairRestraint.h"
#include <IMP/internal/container_helpers.h>

#include <IMP/log.h>


IMPCORE_BEGIN_NAMESPACE

namespace {
  typedef IMP::internal::ContainerTraits<ParticlePair> Traits;
}

PairRestraint
::PairRestraint(PairScore *ss,
                     Particle *a, Particle *b,
                     std::string name):
  Restraint(name),
  ss_(ss),
  v_(ParticlePair(a,b)),
  score_(std::numeric_limits<double>::quiet_NaN())
{
}

double PairRestraint
::unprotected_evaluate(DerivativeAccumulator *accum) const
{
  IMP_OBJECT_LOG;
  IMP_CHECK_OBJECT(ss_);
  score_ = Traits::evaluate(ss_, v_, accum);

  return score_;
}

double PairRestraint
::unprotected_incremental_evaluate(DerivativeAccumulator *accum) const
{
  if (IMP::internal::ContainerTraits<ParticlePair>::is_dirty(v_)) {
    score_+=Traits::evaluate_change(ss_, v_, accum);
  }
  return score_;
}

ParticlesList PairRestraint::get_interacting_particles() const
{
  return IMP::internal::get_interacting_particles(v_, ss_.get());
}

ParticlesTemp PairRestraint::get_input_particles() const
{
  return IMP::internal::get_input_particles(v_, ss_.get());
}

ObjectsTemp PairRestraint::get_input_objects() const
{
  return ObjectsTemp();
}

void PairRestraint::show(std::ostream& out) const
{
  out << "PairRestraint with score function ";
  ss_->show(out);
  out << " and " << v_;
  out << std::endl;
}

IMPCORE_END_NAMESPACE
