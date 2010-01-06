/**
 *  \file QuadsRestraint.cpp
 *  \brief Implementation
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2010 Sali Lab. All rights reserved.
 *
 */

#include "IMP/core/QuadsRestraint.h"
#include <IMP/internal/container_helpers.h>
#include "IMP/core/ListQuadContainer.h"

#include <IMP/QuadScore.h>
#include <IMP/log.h>


IMPCORE_BEGIN_NAMESPACE

QuadsRestraint
::QuadsRestraint(QuadScore *ss,
                      const ParticleQuads &pc,
                      std::string name):
  Restraint(name),
  ss_(ss),
  pc_(new ListQuadContainer(pc))
{
}

QuadsRestraint
::QuadsRestraint(QuadScore *ss,
                      QuadContainer *pc,
                      std::string name): Restraint(name),
                                         ss_(ss), pc_(pc) {

}

double QuadsRestraint
::unprotected_evaluate(DerivativeAccumulator *accum) const
{
  IMP_OBJECT_LOG;
  IMP_CHECK_OBJECT(ss_);
  IMP_CHECK_OBJECT(pc_);

  score_= pc_->evaluate(ss_, accum);
  return score_;
}

double QuadsRestraint
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

ParticlesList QuadsRestraint::get_interacting_particles() const
{
  IMP_OBJECT_LOG;
  ParticlesList ret0= IMP::internal::get_interacting_particles(pc_.get(),
                                                               ss_.get());
  return ret0;
}

ParticlesTemp QuadsRestraint::get_input_particles() const
{
  IMP_OBJECT_LOG;
  ParticlesTemp ret0= IMP::internal::get_input_particles(pc_.get(),
                                                         ss_.get());
  return ret0;
}

ContainersTemp QuadsRestraint::get_input_containers() const
{
  ContainersTemp ret= IMP::internal::get_input_containers(pc_.get(),
                                                          ss_.get());
  ret.push_back(pc_);
  return ret;
}

namespace {
  ListQuadContainer *
  check_methods(QuadContainer *pc, std::string str) {
    ListQuadContainer *ret
      = dynamic_cast<ListQuadContainer*>(pc);
    if (! ret) {
      std::ostringstream oss;
      oss << "Method QuadsRestraint::" << str
          << " can only be called if the QuadContainer "
          << " is a ListQuadContainer.";
      throw UsageException(oss.str().c_str());
    }
    return ret;
  }
}

void QuadsRestraint::add_particle_quads(const ParticleQuads &ps) {
  ListQuadContainer *pc= check_methods(pc_.get(), "add_particle_quads");
  pc->add_particle_quads(ps);
}

void QuadsRestraint::set_particle_quads(const ParticleQuads &ps){
  ListQuadContainer *pc= check_methods(pc_.get(), "add_particle_quads");
  pc->set_particle_quads(ps);
}

void QuadsRestraint::add_particle_quad(ParticleQuad v){
  ListQuadContainer *pc= check_methods(pc_.get(), "add_particle_quads");
  pc->add_particle_quad(v);
}

void QuadsRestraint::clear_particle_quads() {
  ListQuadContainer *pc= check_methods(pc_.get(), "add_particle_quads");
  pc->clear_particle_quads();
}


void QuadsRestraint::show(std::ostream& out) const
{
  out << "QuadRestraint with score function ";
  ss_->show(out);
  out << " and container ";
  pc_->show(out);
  out << std::endl;
}

IMPCORE_END_NAMESPACE
