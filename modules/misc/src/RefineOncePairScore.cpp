/**
 *  \file RefineOncePairScore.cpp
 *  \brief Refine particles at most once with a Refiner.
 *
 *
 *  Copyright 2007-2010 Sali Lab. All rights reserved.
 */

#include <IMP/misc/RefineOncePairScore.h>

#include <IMP/core/XYZ.h>
#include <IMP/deprecation.h>
#include <IMP/internal/container_helpers.h>
#include <cmath>

IMPMISC_BEGIN_NAMESPACE

#ifndef IMP_NO_DEPRECATED

RefineOncePairScore::RefineOncePairScore(Refiner *r,
                                         PairScore *f): r_(r), f_(f) {
  IMP_DEPRECATED(RefineOncePairScore, RefinedPairsPairScore);
}

namespace {
  ParticlesTemp get_set(Particle *a, Refiner *r) {
    ParticlesTemp ret;
    if (r->get_can_refine(a)) {
      ret= r->get_refined(a);
    } else {
      ret.push_back(a);
    }
    return ret;
  }
}

bool RefineOncePairScore::get_is_changed(const ParticlePair &p) const {
  for (unsigned int i=0; i< 2; ++i) {
    ParticlesTemp ps=get_set(p[0], r_);
    for (unsigned int j=0; j< ps.size(); ++j) {
      if (ps[i]->get_is_changed()) return true;
    }
  }
  return false;
}

Float RefineOncePairScore::evaluate(const ParticlePair &p,
                                    DerivativeAccumulator *da) const
{
  Particles ps[2]={get_set(p[0], r_), get_set(p[1], r_)};
  Float ret=0;
  for (unsigned int i=0; i< ps[0].size(); ++i) {
    for (unsigned int j=0; j< ps[1].size(); ++j) {
      ret+=f_->evaluate(ParticlePair(ps[0][i], ps[1][j]), da);
    }
  }
  return ret;
}

ParticlesList
RefineOncePairScore::get_interacting_particles(const ParticlePair &p) const {
  Particles ps[2]={get_set(p[0], r_), get_set(p[1], r_)};
  ParticlesList ret;
  for (unsigned int i=0; i< ps[0].size(); ++i) {
    for (unsigned int j=0; j< ps[1].size(); ++j) {
      ret.push_back(IMP::internal::get_union(
                f_->get_interacting_particles(ParticlePair(ps[0][i],
                                                           ps[1][j]))));
    }
  }
  return ret;
}

ParticlesTemp RefineOncePairScore
::get_input_particles(const ParticlePair &p) const {
  Particles ps[2]={get_set(p[0], r_), get_set(p[1], r_)};
  ParticlesTemp ret;
  for (unsigned int i=0; i< ps[0].size(); ++i) {
    for (unsigned int j=0; j< ps[1].size(); ++j) {
      ParticlesTemp cps= f_->get_input_particles(ParticlePair(ps[0][i],
                                                              ps[1][j]));
      ret.insert(ret.end(), cps.begin(), cps.end());
    }
  }
  ret.push_back(p[0]);
  ret.push_back(p[1]);
  return ret;
}


ContainersTemp RefineOncePairScore
::get_input_containers(const ParticlePair &p) const {
  ContainersTemp ret= r_->get_input_containers(p[0]);
  ContainersTemp ret1= r_->get_input_containers(p[1]);
  ret.insert(ret.end(), ret1.begin(), ret1.end());
  return ret;
}

void RefineOncePairScore::do_show(std::ostream &out) const
{
  f_->show(out);
  r_->show(out);
}

#endif // IMP_NO_DEPRECATED

IMPMISC_END_NAMESPACE
