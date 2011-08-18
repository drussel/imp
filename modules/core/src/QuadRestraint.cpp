/**
 *  \file QuadRestraint.cpp
 *  \brief Implementation
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 *
 */

#include "IMP/core/QuadRestraint.h"
#include <IMP/internal/container_helpers.h>

#include <IMP/log.h>


IMPCORE_BEGIN_NAMESPACE

QuadRestraint
::QuadRestraint(QuadScore *ss,
                     const ParticleQuad& vt,
                     std::string name):
  QuadScoreRestraint(name),
  ss_(ss),
  v_(IMP::internal::get_index(vt))
{
}

double QuadRestraint
::unprotected_evaluate(DerivativeAccumulator *accum) const
{
  IMP_OBJECT_LOG;
  IMP_CHECK_OBJECT(ss_);
  return ss_->evaluate_index(get_model(), v_, accum);
}

double QuadRestraint
::unprotected_evaluate_if_good(DerivativeAccumulator *accum,
                               double max) const
{
  IMP_OBJECT_LOG;
  IMP_CHECK_OBJECT(ss_);
  return ss_->evaluate_if_good_index(get_model(), v_, accum, max);
}



ParticlesTemp QuadRestraint::get_input_particles() const
{
  ParticleQuad vi= IMP::internal::get_particle(get_model(), v_);
  return IMP::internal::get_input_particles(ss_.get(),
                                            vi);
}

ContainersTemp QuadRestraint::get_input_containers() const
{
  ParticleQuad vi= IMP::internal::get_particle(get_model(), v_);
  return IMP::internal::get_input_containers(ss_.get(),
                                             vi);
}


Restraints QuadRestraint::get_current_decomposition() const {
  ParticleQuad vi= IMP::internal::get_particle(get_model(), v_);
  return ss_->get_current_decomposition(vi);
}

void QuadRestraint::do_show(std::ostream& out) const
{
  out << "score " << ss_->get_name() << std::endl;
  out << "data "
      << IMP::internal::streamable(IMP::internal::get_particle(get_model(),v_))
      << std::endl;
}

IMPCORE_END_NAMESPACE
