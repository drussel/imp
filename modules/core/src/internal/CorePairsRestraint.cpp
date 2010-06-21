/**
 *  \file PairsRestraint.cpp
 *  \brief Implementation
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 *
 */

#include "IMP/core/internal/CorePairsRestraint.h"
#include "IMP/core/PairRestraint.h"
#include <IMP/internal/container_helpers.h>

#include <IMP/PairScore.h>
#include <IMP/log.h>


IMPCORE_BEGIN_INTERNAL_NAMESPACE

CorePairsRestraint
::CorePairsRestraint(PairScore *ss,
                      PairContainer *pc,
                      std::string name):
  DecomposableRestraint(name),
  ss_(ss), pc_(pc) {

}

double CorePairsRestraint
::unprotected_evaluate(DerivativeAccumulator *accum) const
{
  IMP_OBJECT_LOG;
  IMP_CHECK_OBJECT(ss_);
  IMP_CHECK_OBJECT(pc_);

  score_= pc_->evaluate(ss_, accum);
  return score_;
}

double CorePairsRestraint
::unprotected_evaluate_subset(DerivativeAccumulator *accum) const
{
  IMP_OBJECT_LOG;
  IMP_CHECK_OBJECT(ss_);
  IMP_CHECK_OBJECT(pc_);

  score_= pc_->evaluate_subset(ss_, accum);
  return score_;
}

double CorePairsRestraint
::unprotected_incremental_evaluate(DerivativeAccumulator *accum) const
{
  IMP_OBJECT_LOG;
  IMP_CHECK_OBJECT(ss_);
  IMP_CHECK_OBJECT(pc_);
  IMP_LOG(VERBOSE, "Scores are " << score_);
  score_+=pc_->evaluate_change(ss_, accum);
  // compute the base for the added ones
  IMP_LOG(VERBOSE, " " << score_);
  score_ +=pc_->get_added_pairs_container()
    ->evaluate_prechange(ss_, accum);
  IMP_LOG(VERBOSE," " << score_);
  if (accum) {
    DerivativeAccumulator nda(*accum, -1);
    score_ -=pc_->get_removed_pairs_container()
      ->evaluate_prechange(ss_, &nda);
  } else {
    score_ -=pc_->get_removed_pairs_container()
      ->evaluate_prechange(ss_, NULL);
  }
  IMP_LOG(VERBOSE," " << score_ << std::endl);
  return score_;
}

ParticlesTemp CorePairsRestraint::get_input_particles() const
{
  IMP_OBJECT_LOG;
  ParticlesTemp ret= IMP::internal::get_input_particles(ss_.get(),
                                      pc_->get_contained_particles());
  return ret;
}

ContainersTemp CorePairsRestraint::get_input_containers() const
{
  ContainersTemp ret= IMP::internal::get_input_containers(ss_.get(),
                                          pc_->get_contained_particles());
  ret.push_back(pc_);
  return ret;
}


Restraints CorePairsRestraint::get_decomposition() const {
    Restraints ret(pc_->get_number());
    for (unsigned int i=0; i< ret.size(); ++i) {
      ret[i]= new PairRestraint(ss_, pc_->get(i));
    }
    return ret;
  }

void CorePairsRestraint::do_show(std::ostream& out) const
{
  out << "score " << *ss_ << std::endl;
  out << "container " << *pc_ << std::endl;
}

IMPCORE_END_INTERNAL_NAMESPACE
