/**
 *  \file SingletonsRestraint.cpp
 *  \brief Implementation
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 *
 */

#include "IMP/core/internal/CoreSingletonsRestraint.h"
#include "IMP/core/SingletonRestraint.h"
#include <IMP/internal/container_helpers.h>

#include <IMP/SingletonScore.h>
#include <IMP/log.h>
#include <sstream>


IMPCORE_BEGIN_INTERNAL_NAMESPACE

CoreSingletonsRestraint
::CoreSingletonsRestraint(SingletonScore *ss,
                      SingletonContainer *pc,
                      std::string name):
  SingletonsScoreRestraint(name),
  ss_(ss), pc_(pc) {

}

double CoreSingletonsRestraint
::unprotected_evaluate(DerivativeAccumulator *accum) const
{
  IMP_OBJECT_LOG;
  IMP_CHECK_OBJECT(ss_);
  IMP_CHECK_OBJECT(pc_);

  score_= pc_->evaluate(ss_, accum);
  return score_;
}


double CoreSingletonsRestraint
::unprotected_incremental_evaluate(DerivativeAccumulator *accum) const
{
  IMP_OBJECT_LOG;
  IMP_CHECK_OBJECT(ss_);
  IMP_CHECK_OBJECT(pc_);
  IMP_LOG(VERBOSE, "Scores are " << score_);
  score_+=pc_->evaluate_change(ss_, accum);
  // compute the base for the added ones
  IMP_LOG(VERBOSE, " " << score_);
  score_ +=ac_
    ->evaluate_prechange(ss_, accum);
  IMP_LOG(VERBOSE," " << score_);
  if (accum) {
    DerivativeAccumulator nda(*accum, -1);
    score_ -=rc_
      ->evaluate_prechange(ss_, &nda);
  } else {
    score_ -=rc_
      ->evaluate_prechange(ss_, NULL);
  }
  IMP_LOG(VERBOSE," " << score_ << std::endl);
  return score_;
}

void CoreSingletonsRestraint
::set_is_incremental(bool tf) {
  if (tf) {
    ac_= pc_->get_added_container();
    rc_= pc_->get_removed_container();
  } else {
    ac_= NULL;
    rc_= NULL;
  }
}

ParticlesTemp CoreSingletonsRestraint::get_input_particles() const
{
  IMP_OBJECT_LOG;
  ParticlesTemp ret= IMP::internal::get_input_particles(ss_.get(),
                                      pc_->get_contained_particles());
  return ret;
}

ContainersTemp CoreSingletonsRestraint::get_input_containers() const
{
  ContainersTemp ret= IMP::internal::get_input_containers(ss_.get(),
                                          pc_->get_contained_particles());
  ret.push_back(pc_);
  return ret;
}


Restraints CoreSingletonsRestraint::get_decomposition() const {
    Restraints ret(pc_->get_number());
    for (unsigned int i=0; i< ret.size(); ++i) {
      ret[i]= new SingletonRestraint(ss_, pc_->get(i));
      std::ostringstream oss;
      oss << get_name() << " on " << IMP::internal::streamable(pc_->get(i));
      ret[i]->set_name(oss.str());
    }
    return ret;
  }

void CoreSingletonsRestraint::do_show(std::ostream& out) const
{
  out << "score " << *ss_ << std::endl;
  out << "container " << *pc_ << std::endl;
}

IMPCORE_END_INTERNAL_NAMESPACE
