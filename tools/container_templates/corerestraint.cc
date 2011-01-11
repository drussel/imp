/**
 *  \file CLASSNAMEsRestraint.cpp
 *  \brief Implementation
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 *
 */

#include "IMP/core/internal/CoreCLASSNAMEsRestraint.h"
#include "IMP/core/CLASSNAMERestraint.h"
#include <IMP/internal/container_helpers.h>

#include <IMP/CLASSNAMEScore.h>
#include <IMP/log.h>
#include <sstream>


IMPCORE_BEGIN_INTERNAL_NAMESPACE

CoreCLASSNAMEsRestraint
::CoreCLASSNAMEsRestraint(CLASSNAMEScore *ss,
                      CLASSNAMEContainer *pc,
                      std::string name):
  CLASSNAMEsScoreRestraint(name),
  ss_(ss), pc_(pc) {

}

double CoreCLASSNAMEsRestraint
::unprotected_evaluate(DerivativeAccumulator *accum) const
{
  IMP_OBJECT_LOG;
  IMP_CHECK_OBJECT(ss_);
  IMP_CHECK_OBJECT(pc_);

  score_= pc_->evaluate(ss_, accum);
  return score_;
}


double CoreCLASSNAMEsRestraint
::unprotected_incremental_evaluate(DerivativeAccumulator *accum) const
{
  IMP_OBJECT_LOG;
  IMP_CHECK_OBJECT(ss_);
  IMP_CHECK_OBJECT(pc_);
  IMP_LOG(VERBOSE, "Scores are " << score_);
  score_+=pc_->evaluate_change(ss_, accum);
  // compute the base for the added ones
  IMP_LOG(VERBOSE, " " << score_);
  score_ +=pc_->get_added_container()
    ->evaluate_prechange(ss_, accum);
  IMP_LOG(VERBOSE," " << score_);
  if (accum) {
    DerivativeAccumulator nda(*accum, -1);
    score_ -=pc_->get_removed_container()
      ->evaluate_prechange(ss_, &nda);
  } else {
    score_ -=pc_->get_removed_container()
      ->evaluate_prechange(ss_, NULL);
  }
  IMP_LOG(VERBOSE," " << score_ << std::endl);
  return score_;
}

ParticlesTemp CoreCLASSNAMEsRestraint::get_input_particles() const
{
  IMP_OBJECT_LOG;
  ParticlesTemp ret= IMP::internal::get_input_particles(ss_.get(),
                                      pc_->get_contained_particles());
  return ret;
}

ContainersTemp CoreCLASSNAMEsRestraint::get_input_containers() const
{
  ContainersTemp ret= IMP::internal::get_input_containers(ss_.get(),
                                          pc_->get_contained_particles());
  ret.push_back(pc_);
  return ret;
}


Restraints CoreCLASSNAMEsRestraint::get_decomposition() const {
    Restraints ret(pc_->get_number());
    for (unsigned int i=0; i< ret.size(); ++i) {
      ret[i]= new CLASSNAMERestraint(ss_, pc_->get(i));
      std::ostringstream oss;
      oss << get_name() << " on " << IMP::internal::streamable(pc_->get(i));
      ret[i]->set_name(oss.str());
    }
    return ret;
  }

void CoreCLASSNAMEsRestraint::do_show(std::ostream& out) const
{
  out << "score " << *ss_ << std::endl;
  out << "container " << *pc_ << std::endl;
}

IMPCORE_END_INTERNAL_NAMESPACE
