/**
 *  \file CoverRefined.cpp
 *  \brief Cover a the refined particles with a sphere.
 *
 *  Copyright 2007-8 Sali Lab. All rights reserved.
 */

#include "IMP/core/CoverRefined.h"

#include "IMP/core/bond_decorators.h"
#include "IMP/core/XYZRDecorator.h"
#include "IMP/core/FixedParticleRefiner.h"
#include "IMP/core/SingletonScoreState.h"
#include "IMP/core/DerivativesToRefined.h"

IMPCORE_BEGIN_NAMESPACE

CoverRefined
::CoverRefined(ParticleRefiner *ref,
               FloatKey rk,
               Float slack): ref_(ref),
                             rk_(rk),
                             slack_(slack)
{
}

CoverRefined::~CoverRefined()
{
}

void CoverRefined::apply(Particle *p) const
{
  XYZRDecorator dp(p, rk_);
  IMP_CHECK_OBJECT(ref_.get());
  IMP_check(ref_->get_can_refine(p), "Passed particles cannot be refined",
            ValueException);
  Particles ps= ref_->get_refined(p);
  set_enclosing_sphere(ps, dp);
  dp.set_radius(dp.get_radius()+slack_);
  ref_->cleanup_refined(p, ps);
}

void CoverRefined::show(std::ostream &out) const
{
  out << "CoverRefined with "
      << *ref_ << " and " << rk_ << std::endl;
}


Particle* create_cover_particle(Model *m, const Particles &ps,
                                FloatKey radius_key, Float slack) {
  IMP_check(!ps.empty(), "Need at least one particle to cover",
            ValueException);
  IMP_IF_CHECK(EXPENSIVE) {
    for (unsigned int i=0; i< ps.size(); ++i) {
      IMP_check(XYZRDecorator::is_instance_of(ps[i], radius_key),
                "Particles must have radius attribute " << radius_key,
                ValueException);
    }
  }
  Particle *p= new Particle(m);
  XYZRDecorator d= XYZRDecorator::create(p, algebra::Vector3D(0,0,0),
                                         0, radius_key);
  FixedParticleRefiner *fpr= new FixedParticleRefiner(ps);

  CoverRefined *cr= new CoverRefined(fpr, radius_key, slack);
  DerivativesToRefined *dtr= new DerivativesToRefined(fpr,
                                    XYZDecorator::get_xyz_keys());
  SingletonScoreState *sss= new SingletonScoreState(cr, dtr, p);
  m->add_score_state(sss);
  IMP_assert(fpr->get_refined(p).size() == ps.size(),
             "FixedPR is broken");
  return p;
}


IMPCORE_END_NAMESPACE
