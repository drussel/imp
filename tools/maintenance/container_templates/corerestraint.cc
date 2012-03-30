/**
 *  \file CLASSNAMEsRestraint.cpp
 *  \brief Implementation
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#include "IMP/internal/InternalCLASSNAMEsRestraint.h"
#include <IMP/internal/container_helpers.h>
#include <IMP/internal/create_decomposition.h>

#include <IMP/CLASSNAMEScore.h>
#include <IMP/log.h>
#include <sstream>


IMP_BEGIN_INTERNAL_NAMESPACE

InternalCLASSNAMEsRestraint
::InternalCLASSNAMEsRestraint(CLASSNAMEScore *ss,
                      CLASSNAMEContainer *pc,
                      std::string name):
  Restraint(pc->get_model(), name),
  ss_(ss), pc_(pc) {
}

double InternalCLASSNAMEsRestraint
::unprotected_evaluate(DerivativeAccumulator *accum) const
{
  IMP_OBJECT_LOG;
  IMP_CHECK_OBJECT(ss_);
  IMP_CHECK_OBJECT(pc_);

  return pc_->evaluate(ss_, accum);
}

double InternalCLASSNAMEsRestraint
::unprotected_evaluate_if_good(DerivativeAccumulator *da, double max) const {
  IMP_OBJECT_LOG;
  IMP_CHECK_OBJECT(ss_);
  IMP_CHECK_OBJECT(pc_);

  return pc_->evaluate_if_good(ss_, da, max);
}



ParticlesTemp InternalCLASSNAMEsRestraint::get_input_particles() const
{
  IMP_OBJECT_LOG;
  ParticlesTemp ret= IMP::internal::get_input_particles(ss_.get(),
                                      pc_->get_all_possible_particles());
  return ret;
}

ContainersTemp InternalCLASSNAMEsRestraint::get_input_containers() const
{
  ContainersTemp ret= IMP::internal::get_input_containers(ss_.get(),
                                          pc_->get_all_possible_particles());
  ret.push_back(pc_);
  return ret;
}


Restraints InternalCLASSNAMEsRestraint::do_create_decomposition() const {
  return IMP::internal::create_decomposition(get_model(),
                                             ss_.get(),
                                             pc_->get_all_possible_indexes(),
                                             get_name());
}

Restraints
InternalCLASSNAMEsRestraint::do_create_current_decomposition() const {
  return IMP::internal::create_decomposition(get_model(),
                                             ss_.get(),
                                             pc_->get_indexes(), get_name());
}

void InternalCLASSNAMEsRestraint::do_show(std::ostream& out) const
{
  out << "score " << ss_->get_name() << std::endl;
  out << "container " << pc_->get_name() << std::endl;
}

IMP_END_INTERNAL_NAMESPACE
