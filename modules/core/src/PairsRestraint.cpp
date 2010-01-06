/**
 *  \file PairsRestraint.cpp
 *  \brief Implementation
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2010 Sali Lab. All rights reserved.
 *
 */

#include "IMP/core/PairsRestraint.h"
#include <IMP/internal/container_helpers.h>
#include "IMP/core/ListPairContainer.h"

#include <IMP/PairScore.h>
#include <IMP/log.h>


IMPCORE_BEGIN_NAMESPACE

PairsRestraint
::PairsRestraint(PairScore *ss,
                      const ParticlePairs &pc,
                      std::string name):
  Restraint(name),
  ss_(ss),
  pc_(new ListPairContainer(pc))
{
}

PairsRestraint
::PairsRestraint(PairScore *ss,
                      PairContainer *pc,
                      std::string name): Restraint(name),
                                         ss_(ss), pc_(pc) {

}

double PairsRestraint
::unprotected_evaluate(DerivativeAccumulator *accum) const
{
  IMP_OBJECT_LOG;
  IMP_CHECK_OBJECT(ss_);
  IMP_CHECK_OBJECT(pc_);

  score_= pc_->evaluate(ss_, accum);
  return score_;
}

double PairsRestraint
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

ParticlesList PairsRestraint::get_interacting_particles() const
{
  IMP_OBJECT_LOG;
  ParticlesList ret0= IMP::internal::get_interacting_particles(pc_.get(),
                                                               ss_.get());
  return ret0;
}

ParticlesTemp PairsRestraint::get_input_particles() const
{
  IMP_OBJECT_LOG;
  ParticlesTemp ret0= IMP::internal::get_input_particles(pc_.get(),
                                                         ss_.get());
  return ret0;
}

ContainersTemp PairsRestraint::get_input_containers() const
{
  ContainersTemp ret= IMP::internal::get_input_containers(pc_.get(),
                                                          ss_.get());
  ret.push_back(pc_);
  return ret;
}

namespace {
  ListPairContainer *
  check_methods(PairContainer *pc, std::string str) {
    ListPairContainer *ret
      = dynamic_cast<ListPairContainer*>(pc);
    if (! ret) {
      std::ostringstream oss;
      oss << "Method PairsRestraint::" << str
          << " can only be called if the PairContainer "
          << " is a ListPairContainer.";
      throw UsageException(oss.str().c_str());
    }
    return ret;
  }
}

void PairsRestraint::add_particle_pairs(const ParticlePairs &ps) {
  ListPairContainer *pc= check_methods(pc_.get(), "add_particle_pairs");
  pc->add_particle_pairs(ps);
}

void PairsRestraint::set_particle_pairs(const ParticlePairs &ps){
  ListPairContainer *pc= check_methods(pc_.get(), "add_particle_pairs");
  pc->set_particle_pairs(ps);
}

void PairsRestraint::add_particle_pair(ParticlePair v){
  ListPairContainer *pc= check_methods(pc_.get(), "add_particle_pairs");
  pc->add_particle_pair(v);
}

void PairsRestraint::clear_particle_pairs() {
  ListPairContainer *pc= check_methods(pc_.get(), "add_particle_pairs");
  pc->clear_particle_pairs();
}


void PairsRestraint::show(std::ostream& out) const
{
  out << "PairRestraint with score function ";
  ss_->show(out);
  out << " and container ";
  pc_->show(out);
  out << std::endl;
}

IMPCORE_END_NAMESPACE
