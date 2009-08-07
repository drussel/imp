/**
 *  \file ExampleComplexRestraint.cpp
 *  \brief Restrain the diameter of a set of points.
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 *
 */

#include "IMP/examples/ExampleComplexRestraint.h"
#include <IMP/PairContainer.h>
#include <IMP/core/XYZR.h>
#include <IMP/core/FixedRefiner.h>
#include <IMP/core/PairsRestraint.h>
#include <IMP/core/CoverRefined.h>
#include <IMP/core/SingletonScoreState.h>
#include <IMP/core/DistancePairScore.h>
#include <IMP/core/internal/evaluate_distance_pair_score.h>
#include <boost/lambda/lambda.hpp>

IMPEXAMPLES_BEGIN_NAMESPACE

ExampleComplexRestraint::ExampleComplexRestraint(UnaryFunction *f,
                                     SingletonContainer *sc,
                                     Float diameter):diameter_(diameter),
                                                     sc_(sc), f_(f),
                                                     dr_("diameter_radius"){
  IMP_check(sc->get_number_of_particles()>2,
            "Need at least two particles to restrain diameter",
            ValueException);
  IMP_check(diameter>0, "The diameter must be positive",
            ValueException);
}

void ExampleComplexRestraint::set_model(Model *m) {
  if (m) {
    IMP_LOG(TERSE, "Creating components of ExampleComplexRestraint"
            << std::endl);
    Model *m= sc_->get_particle(0)->get_model();

    p_= new Particle(m);
    core::XYZR d= core::XYZR::setup_particle(p_, dr_);
    d.set_coordinates_are_optimized(false);
    core::CoverRefined *cr
      = new core::CoverRefined(
             new core::FixedRefiner(Particles(sc_->particles_begin(),
                                              sc_->particles_end())),
                               dr_, 0);
    ss_= new core::SingletonScoreState(cr, NULL, p_);
    m->add_score_state(ss_);
  } else {
    IMP_LOG(TERSE, "Removing components of ExampleComplexRestraint"
            << std::endl);
    IMP_CHECK_OBJECT(ss_.get());
    IMP_CHECK_OBJECT(p_.get());
    m->remove_score_state(ss_);
    m->remove_particle(p_);
    ss_=NULL;
    p_=NULL;
  }
  Restraint::set_model(m);
}

Float ExampleComplexRestraint::evaluate(DerivativeAccumulator *da) {
  IMP_CHECK_OBJECT(sc_.get());
  double v=0;
  core::XYZ dp(p_);
  double radius= diameter_/2.0;
  for (SingletonContainer::ParticleIterator pit= sc_->particles_begin();
       pit != sc_->particles_end(); ++pit) {
    v+= core::internal::evaluate_distance_pair_score(dp,
                                               core::XYZ(*pit),
                                               da, f_.get(),
                                               boost::lambda::_1-radius);
  }
  return v;
}

void ExampleComplexRestraint::show(std::ostream &out) const {
  out << "ExampleComplexRestraint" << std::endl;
}

ParticlesList ExampleComplexRestraint::get_interacting_particles() const {
  ParticlesList ret;
  for (SingletonContainer::ParticleIterator pit= sc_->particles_begin();
       pit != sc_->particles_end(); ++pit) {
    Particles ps(2);
    ps.set(0,p_);
    ps.set(1,*pit);
    ret.push_back(ps);
  }
  return ret;
}

IMPEXAMPLES_END_NAMESPACE
