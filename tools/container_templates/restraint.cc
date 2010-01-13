/**
 *  \file GroupnamesRestraint.cpp
 *  \brief Implementation
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2010 Sali Lab. All rights reserved.
 *
 */

#include "IMP/core/GroupnamesRestraint.h"
#include <IMP/internal/container_helpers.h>
#include "IMP/core/ListGroupnameContainer.h"

#include <IMP/GroupnameScore.h>
#include <IMP/log.h>


IMPCORE_BEGIN_NAMESPACE

GroupnamesRestraint
::GroupnamesRestraint(GroupnameScore *ss,
                      const Classnames &pc,
                      std::string name):
  Restraint(name),
  ss_(ss),
  pc_(new ListGroupnameContainer(pc))
{
}

GroupnamesRestraint
::GroupnamesRestraint(GroupnameScore *ss,
                      GroupnameContainer *pc,
                      std::string name): Restraint(name),
                                         ss_(ss), pc_(pc) {

}

double GroupnamesRestraint
::unprotected_evaluate(DerivativeAccumulator *accum) const
{
  IMP_OBJECT_LOG;
  IMP_CHECK_OBJECT(ss_);
  IMP_CHECK_OBJECT(pc_);

  score_= pc_->evaluate(ss_, accum);
  return score_;
}

double GroupnamesRestraint
::unprotected_incremental_evaluate(DerivativeAccumulator *accum) const
{
  IMP_OBJECT_LOG;
  IMP_CHECK_OBJECT(ss_);
  IMP_CHECK_OBJECT(pc_);
  IMP_LOG(VERBOSE, "Scores are " << score_);
  score_+=pc_->evaluate_change(ss_, accum);
  // compute the base for the added ones
  IMP_LOG(VERBOSE, " " << score_);
  score_ +=pc_->get_added_groupnames_container()
    ->evaluate_prechange(ss_, accum);
  IMP_LOG(VERBOSE," " << score_);
  if (accum) {
    DerivativeAccumulator nda(*accum, -1);
    score_ -=pc_->get_removed_groupnames_container()
      ->evaluate_prechange(ss_, &nda);
  } else {
    score_ -=pc_->get_removed_groupnames_container()
      ->evaluate_prechange(ss_, NULL);
  }
  IMP_LOG(VERBOSE," " << score_ << std::endl);
  return score_;
}

ParticlesList GroupnamesRestraint::get_interacting_particles() const
{
  IMP_OBJECT_LOG;
  ParticlesList ret0= IMP::internal::get_interacting_particles(pc_.get(),
                                                               ss_.get());
  return ret0;
}

ParticlesTemp GroupnamesRestraint::get_input_particles() const
{
  IMP_OBJECT_LOG;
  ParticlesTemp ret0= IMP::internal::get_input_particles(pc_.get(),
                                                         ss_.get());
  return ret0;
}

ContainersTemp GroupnamesRestraint::get_input_containers() const
{
  ContainersTemp ret= IMP::internal::get_input_containers(pc_.get(),
                                                          ss_.get());
  ret.push_back(pc_);
  return ret;
}

namespace {
  ListGroupnameContainer *
  check_methods(GroupnameContainer *pc, std::string str) {
    ListGroupnameContainer *ret
      = dynamic_cast<ListGroupnameContainer*>(pc);
    if (! ret) {
      std::ostringstream oss;
      oss << "Method GroupnamesRestraint::" << str
          << " can only be called if the GroupnameContainer "
          << " is a ListGroupnameContainer.";
      throw UsageException(oss.str().c_str());
    }
    return ret;
  }
}

void GroupnamesRestraint::add_classnames(const Classnames &ps) {
  ListGroupnameContainer *pc= check_methods(pc_.get(), "add_classnames");
  pc->add_classnames(ps);
}

void GroupnamesRestraint::set_classnames(const Classnames &ps){
  ListGroupnameContainer *pc= check_methods(pc_.get(), "add_classnames");
  pc->set_classnames(ps);
}

void GroupnamesRestraint::add_classname(Value v){
  ListGroupnameContainer *pc= check_methods(pc_.get(), "add_classnames");
  pc->add_classname(v);
}

void GroupnamesRestraint::clear_classnames() {
  ListGroupnameContainer *pc= check_methods(pc_.get(), "add_classnames");
  pc->clear_classnames();
}


void GroupnamesRestraint::show(std::ostream& out) const
{
  out << "GroupnameRestraint with score function ";
  ss_->show(out);
  out << " and container ";
  pc_->show(out);
  out << std::endl;
}

IMPCORE_END_NAMESPACE
