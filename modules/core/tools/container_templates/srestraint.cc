/**
 *  \file GroupnameRestraint.cpp
 *  \brief Implementation
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 *
 */

#include "IMP/core/GroupnameRestraint.h"
#include <IMP/internal/container_helpers.h>

#include <IMP/log.h>


IMPCORE_BEGIN_NAMESPACE

GroupnameRestraint
::GroupnameRestraint(GroupnameScore *ss,
                     ClassnameArguments,
                     std::string name):
  Restraint(name),
  ss_(ss),
  v_(FromClassnameArguments)
{
}

double
GroupnameRestraint::unprotected_evaluate(DerivativeAccumulator *accum) const
{
  IMP_OBJECT_LOG;
  IMP_CHECK_OBJECT(ss_);

  double score=0;
  score += IMP::internal::ContainerTraits<Classname>
    ::evaluate(ss_, v_, accum);

  return score;
}


ParticlesList GroupnameRestraint::get_interacting_particles() const
{
  return IMP::internal::get_interacting_particles(v_, ss_.get());
}

ParticlesTemp GroupnameRestraint::get_read_particles() const
{
  return IMP::internal::get_read_particles(v_, ss_.get());
}

ParticlesTemp GroupnameRestraint::get_write_particles() const
{
  return IMP::internal::get_write_particles(v_, ss_.get());
}

void GroupnameRestraint::show(std::ostream& out) const
{
  out << "GroupnameRestraint with score function ";
  ss_->show(out);
  out << " and " << v_;
  out << std::endl;
}

IMPCORE_END_NAMESPACE
