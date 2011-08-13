/**
 *  \file DiameterRestraint.cpp
 *  \brief XXXX.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 *
 */

#include "IMP/core/DiameterRestraint.h"
#include <IMP/PairContainer.h>
#include <IMP/core/XYZR.h>
#include <IMP/core/FixedRefiner.h>
#include <IMP/core/internal/CorePairsRestraint.h>
#include <IMP/core/CoverRefined.h>
#include <IMP/core/DistancePairScore.h>
#include <IMP/core/SingletonConstraint.h>
#include <IMP/core/internal/evaluate_distance_pair_score.h>
#include <IMP/core/FixedRefiner.h>
#include <boost/lambda/lambda.hpp>

IMPCORE_BEGIN_NAMESPACE

DiameterRestraint::DiameterRestraint(UnaryFunction *f,
                                     SingletonContainer *sc,
                                     Float diameter):diameter_(diameter),
                                                     sc_(sc), f_(f){
  IMP_USAGE_CHECK(sc->get_indexes().size()>=2,
            "Need at least two particles to restrain diameter");
  IMP_USAGE_CHECK(diameter>0, "The diameter must be positive");
}

void DiameterRestraint::set_model(Model *m) {
  if (m) {
    IMP_LOG(TERSE, "Creating components of DiameterRestraint" << std::endl);
    Model *m= sc_->get_particle(0)->get_model();

    // make pairs from special generator
    p_= new Particle(m);
    XYZR d= XYZR::setup_particle(p_);
    p_->set_name("DiameterRestraint center");
    d.set_coordinates_are_optimized(false);
    Pointer<core::CoverRefined> cr
      = new core::CoverRefined(
                               new FixedRefiner(sc_->get_particles()),
            0);
    ss_= new core::SingletonConstraint(cr, NULL, p_);

    m->add_score_state(ss_);
    // make sure model hasn't been cleanup up already
  } else {
    if (ss_ && p_->get_is_active()) {
      IMP_LOG(TERSE, "Removing components of DiameterRestraint" << std::endl);
      IMP_CHECK_OBJECT(ss_.get());
      IMP_CHECK_OBJECT(p_.get());
      get_model()->remove_score_state(ss_);
      get_model()->remove_particle(p_);
    }
    ss_=NULL;
    p_=NULL;
  }
  Restraint::set_model(m);
}

double
DiameterRestraint::unprotected_evaluate(DerivativeAccumulator *da) const {
  IMP_CHECK_OBJECT(sc_.get());
  double v=0;
  XYZ dp(p_);
  double radius= diameter_/2.0;
  IMP_FOREACH_SINGLETON(sc_, {
    v+= internal::evaluate_distance_pair_score(dp,
                                               XYZ(_1),
                                               da, f_.get(),
                                               boost::lambda::_1-radius);
    });
  return v;
}

void DiameterRestraint::do_show(std::ostream &out) const {
  out << "diameter " << diameter_ << std::endl;
}

ParticlesTemp DiameterRestraint::get_input_particles() const {
  ParticlesTemp t=sc_->get_particles();
  t.push_back(p_);
  return t;
}

ContainersTemp DiameterRestraint::get_input_containers() const {
  return ContainersTemp(1, sc_);
}

IMPCORE_END_NAMESPACE
