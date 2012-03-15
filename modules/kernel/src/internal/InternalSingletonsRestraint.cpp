/**
 *  \file SingletonsRestraint.cpp
 *  \brief Implementation
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#include "IMP/internal/InternalSingletonsRestraint.h"
#include <IMP/internal/container_helpers.h>

#include <IMP/SingletonScore.h>
#include <IMP/log.h>
#include <sstream>


IMP_BEGIN_INTERNAL_NAMESPACE

InternalSingletonsRestraint
::InternalSingletonsRestraint(SingletonScore *ss,
                      SingletonContainer *pc,
                      std::string name):
  Restraint(name),
  ss_(ss), pc_(pc) {

}

double InternalSingletonsRestraint
::unprotected_evaluate(DerivativeAccumulator *accum) const
{
  IMP_OBJECT_LOG;
  IMP_CHECK_OBJECT(ss_);
  IMP_CHECK_OBJECT(pc_);

  return pc_->evaluate(ss_, accum);
}

double InternalSingletonsRestraint
::unprotected_evaluate_if_good(DerivativeAccumulator *da, double max) const {
  IMP_OBJECT_LOG;
  IMP_CHECK_OBJECT(ss_);
  IMP_CHECK_OBJECT(pc_);

  return pc_->evaluate_if_good(ss_, da, max);
}



ParticlesTemp InternalSingletonsRestraint::get_input_particles() const
{
  IMP_OBJECT_LOG;
  ParticlesTemp ret= IMP::internal::get_input_particles(ss_.get(),
                                      pc_->get_contained_particles());
  return ret;
}

ContainersTemp InternalSingletonsRestraint::get_input_containers() const
{
  ContainersTemp ret= IMP::internal::get_input_containers(ss_.get(),
                                          pc_->get_contained_particles());
  ret.push_back(pc_);
  return ret;
}


Restraints InternalSingletonsRestraint::do_create_decomposition() const {
  Restraints cur= pc_->create_decomposition(ss_);
  for (unsigned int i=0; i< cur.size(); ++i) {
    std::ostringstream oss;
    oss << get_name() << " " << i;
    cur[i]->set_name(oss.str());
  }
  return cur;
}

Restraints
InternalSingletonsRestraint::do_create_current_decomposition() const {
  Restraints ret;
  for (unsigned int i=0; i< pc_->get_number(); ++i) {
    Restraints cur=ss_->create_current_decomposition(pc_->get(i));
    ret.insert(ret.end(), cur.begin(), cur.end());
  }
  for (unsigned int i=0; i< ret.size(); ++i) {
    std::ostringstream oss;
    oss << get_name() << " " << i;
    ret[i]->set_name(oss.str());
  }
  return ret;
}

void InternalSingletonsRestraint::do_show(std::ostream& out) const
{
  out << "score " << ss_->get_name() << std::endl;
  out << "container " << pc_->get_name() << std::endl;
}

IMP_END_INTERNAL_NAMESPACE