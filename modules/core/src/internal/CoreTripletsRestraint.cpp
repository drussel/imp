/**
 *  \file TripletsRestraint.cpp
 *  \brief Implementation
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 *
 */

#include "IMP/core/internal/CoreTripletsRestraint.h"
#include "IMP/core/TripletRestraint.h"
#include <IMP/internal/container_helpers.h>

#include <IMP/TripletScore.h>
#include <IMP/log.h>
#include <sstream>


IMPCORE_BEGIN_INTERNAL_NAMESPACE

CoreTripletsRestraint
::CoreTripletsRestraint(TripletScore *ss,
                      TripletContainer *pc,
                      std::string name):
  TripletsScoreRestraint(name),
  ss_(ss), pc_(pc) {

}

double CoreTripletsRestraint
::unprotected_evaluate(DerivativeAccumulator *accum) const
{
  IMP_OBJECT_LOG;
  IMP_CHECK_OBJECT(ss_);
  IMP_CHECK_OBJECT(pc_);

  return pc_->evaluate(ss_, accum);
}


ParticlesTemp CoreTripletsRestraint::get_input_particles() const
{
  IMP_OBJECT_LOG;
  ParticlesTemp ret= IMP::internal::get_input_particles(ss_.get(),
                                      pc_->get_contained_particles());
  return ret;
}

ContainersTemp CoreTripletsRestraint::get_input_containers() const
{
  ContainersTemp ret= IMP::internal::get_input_containers(ss_.get(),
                                          pc_->get_contained_particles());
  ret.push_back(pc_);
  return ret;
}


Restraints CoreTripletsRestraint::get_decomposition() const {
  Restraints ret(pc_->get_number());
  for (unsigned int i=0; i< ret.size(); ++i) {
    ret[i]= new TripletRestraint(ss_, pc_->get(i));
    std::ostringstream oss;
    oss << get_name() << " " << i;
    ret[i]->set_name(oss.str());
    }
  return ret;
}

Restraints CoreTripletsRestraint::get_instant_decomposition() const {
  Restraints ret;
  for (unsigned int i=0; i< pc_->get_number(); ++i) {
    Restraints cur=ss_->get_instant_decomposition(pc_->get(i));
    ret.insert(ret.end(), cur.begin(), cur.end());
  }
  for (unsigned int i=0; i< ret.size(); ++i) {
    std::ostringstream oss;
    oss << get_name() << " " << i;
    ret[i]->set_name(oss.str());
  }
  return ret;
}

void CoreTripletsRestraint::do_show(std::ostream& out) const
{
  out << "score " << ss_->get_name() << std::endl;
  out << "container " << pc_->get_name() << std::endl;
}

IMPCORE_END_INTERNAL_NAMESPACE
