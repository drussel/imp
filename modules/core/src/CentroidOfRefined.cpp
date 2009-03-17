/**
 *  \file CentroidOfRefined.cpp
 *  \brief CentroidOf a the refined particles with a sphere.
 *
 *  Copyright 2007-8 Sali Lab. All rights reserved.
 */

#include "IMP/core/CentroidOfRefined.h"

#include "IMP/core/bond_decorators.h"
#include "IMP/core/XYZRDecorator.h"
#include "IMP/core/FixedParticleRefiner.h"
#include "IMP/core/SingletonScoreState.h"
#include "IMP/core/SingletonsScoreState.h"
#include "IMP/core/DerivativesToRefined.h"

IMPCORE_BEGIN_NAMESPACE

CentroidOfRefined
::CentroidOfRefined(ParticleRefiner *r,
                                     FloatKey weight,
                                     FloatKeys ks): r_(r),
ks_(ks), w_(weight)
{
}

CentroidOfRefined::~CentroidOfRefined()
{
}

void CentroidOfRefined::apply(Particle *p) const
{
  Particles ps = r_->get_refined(p);
  unsigned int n= ps.size();
  double tw=0;
  if (w_ != FloatKey()) {
    for (unsigned int i=0; i< n; ++i) {
      tw+= ps[i]->get_value(w_);
    }
  } else {
    tw=1;
  }
  for (unsigned int j=0; j< ks_.size(); ++j) {
    double v=0;
    for (unsigned int i=0; i< n; ++i) {
      double w;
      if (w_ != FloatKey()) {
        w= ps[i]->get_value(w_)/tw;
      } else {
        w= Float(1.0)/n;
      }
      v += ps[i]->get_value(ks_[j])*w;
    }
    p->set_value(ks_[j], v);
  }
  r_->cleanup_refined(p, ps);
}

void CentroidOfRefined::show(std::ostream &out) const
{
  out << "CentroidOfRefined" << std::endl;
}

void create_centroids(SingletonContainer *sc,
                      ParticleRefiner *pr,
                      FloatKey weight,
                      FloatKeys ks) {
  IMP_check(sc->get_number_of_particles() >0,
            "Need some particles to set up as centroid",
            ValueException);
  Model *m= sc->get_particle(0)->get_model();
  for (SingletonContainer::ParticleIterator pit= sc->particles_begin();
       pit != sc->particles_end(); ++pit) {
    for (unsigned int i=0; i< ks.size(); ++i) {
      if (!(*pit)->has_attribute(ks[i])) {
        (*pit)->add_attribute(ks[i], 0, false);
      }
    }
  }

  CentroidOfRefined *cr= new CentroidOfRefined(pr, weight, ks);
  DerivativesToRefined *dtr= new DerivativesToRefined(pr,
                                                      ks);
  SingletonsScoreState *sss= new SingletonsScoreState(sc, cr, dtr);
  m->add_score_state(sss);
}



void create_centroid(Particle *p, ParticleRefiner *pr,
                     FloatKey weight,
                     FloatKeys ks) {
  for (unsigned int i=0; i< ks.size(); ++i) {
    if (!p->has_attribute(ks[i])) {
      p->add_attribute(ks[i], 0, false);
    }
  }

  CentroidOfRefined *cr= new CentroidOfRefined(pr, weight, ks);
  DerivativesToRefined *dtr= new DerivativesToRefined(pr,ks);
  SingletonScoreState *sss= new SingletonScoreState(cr, dtr, p);
  p->get_model()->add_score_state(sss);
  IMP_check(pr->get_refined(p).size()>0,
             "Need particles to compute the centroid of",
             ValueException);
}


IMPCORE_END_NAMESPACE
