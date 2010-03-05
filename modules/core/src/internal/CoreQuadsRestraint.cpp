/**
 *  \file QuadsRestraint.cpp
 *  \brief Implementation
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 *
 */

#include "IMP/core/internal/CoreQuadsRestraint.h"
#include <IMP/internal/container_helpers.h>

#include <IMP/QuadScore.h>
#include <IMP/log.h>


IMPCORE_BEGIN_INTERNAL_NAMESPACE

CoreQuadsRestraint
::CoreQuadsRestraint(QuadScore *ss,
                      QuadContainer *pc,
                      std::string name): Restraint(name),
                                         ss_(ss), pc_(pc) {

}

double CoreQuadsRestraint
::unprotected_evaluate(DerivativeAccumulator *accum) const
{
  IMP_OBJECT_LOG;
  IMP_CHECK_OBJECT(ss_);
  IMP_CHECK_OBJECT(pc_);

  score_= pc_->evaluate(ss_, accum);
  return score_;
}

double CoreQuadsRestraint
::unprotected_incremental_evaluate(DerivativeAccumulator *accum) const
{
  IMP_OBJECT_LOG;
  IMP_CHECK_OBJECT(ss_);
  IMP_CHECK_OBJECT(pc_);
  IMP_LOG(VERBOSE, "Scores are " << score_);
  score_+=pc_->evaluate_change(ss_, accum);
  // compute the base for the added ones
  IMP_LOG(VERBOSE, " " << score_);
  score_ +=pc_->get_added_quads_container()
    ->evaluate_prechange(ss_, accum);
  IMP_LOG(VERBOSE," " << score_);
  if (accum) {
    DerivativeAccumulator nda(*accum, -1);
    score_ -=pc_->get_removed_quads_container()
      ->evaluate_prechange(ss_, &nda);
  } else {
    score_ -=pc_->get_removed_quads_container()
      ->evaluate_prechange(ss_, NULL);
  }
  IMP_LOG(VERBOSE," " << score_ << std::endl);
  return score_;
}

ParticlesList CoreQuadsRestraint::get_interacting_particles() const
{
  IMP_OBJECT_LOG;
  ParticlesList ret0= IMP::internal::get_interacting_particles(pc_.get(),
                                                               ss_.get());
  return ret0;
}

ParticlesTemp CoreQuadsRestraint::get_input_particles() const
{
  IMP_OBJECT_LOG;
  ParticlesTemp ret0= IMP::internal::get_input_particles(pc_.get(),
                                                         ss_.get());
  return ret0;
}

ContainersTemp CoreQuadsRestraint::get_input_containers() const
{
  ContainersTemp ret= IMP::internal::get_input_containers(pc_.get(),
                                                          ss_.get());
  ret.push_back(pc_);
  return ret;
}


void CoreQuadsRestraint::do_show(std::ostream& out) const
{
  out << "score " << *ss_ << std::endl;
  out << "container " << *pc_ << std::endl;
}

IMPCORE_END_INTERNAL_NAMESPACE
