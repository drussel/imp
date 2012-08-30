/**
 *  \file MinimumPairScore.cpp  \brief Define PairScore
 *
 *  WARNING This file was generated from MinimumNAMEScore.cc
 *  in tools/maintenance/container_templates/container
 *  by tools/maintenance/make-container.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */

#include <IMP/container/MinimumPairScore.h>
#include <IMP/core/PairRestraint.h>
#include "IMP/algebra/internal/MinimalSet.h"

IMPCONTAINER_BEGIN_NAMESPACE

MinimumPairScore::MinimumPairScore(const PairScoresTemp &scores,
                                           unsigned int n,
                                           std::string name):
  PairScore(name),
  scores_(scores.begin(), scores.end()),
  n_(n)
{
}


namespace {
  typedef algebra::internal::MinimalSet<double,
          PairScore*, std::less<double> >
  MinimumPairScoreMS;
  template <class It>
  MinimumPairScoreMS
  find_minimal_set_MinimumPairScore(It b, It e, const ParticlePair& v,
                                          unsigned int n) {
    IMP_LOG(TERSE, "Finding Minimum " << n << " of "
            << std::distance(b,e) << std::endl);
    MinimumPairScoreMS bestn(n);
    for (It it= b; it != e; ++it) {
      double score= (*it)->evaluate(v, nullptr);
      bestn.insert(score, *it);
    }
    return bestn;
  }
}

double MinimumPairScore::evaluate(const ParticlePair& v,
                                      DerivativeAccumulator *da) const {
  MinimumPairScoreMS bestn
    = find_minimal_set_MinimumPairScore(scores_.begin(),
                                              scores_.end(), v, n_);

  double score=0;
  for (unsigned int i=0; i< bestn.size(); ++i) {
    if (da) {
      bestn[i].second->evaluate(v, da);
    }
    score+= bestn[i].first;
  }
  return score;
}

ParticlesTemp MinimumPairScore
::get_input_particles(Particle* p) const {
  ParticlesTemp ret;
  for (unsigned int i=0; i< scores_.size(); ++i) {
    ParticlesTemp c= scores_[i]->get_input_particles(p);
    ret.insert(ret.end(), c.begin(), c.end());
  }
  return ret;
}

ContainersTemp MinimumPairScore
::get_input_containers(Particle* p) const {
  ContainersTemp ret;
  for (unsigned int i=0; i< scores_.size(); ++i) {
    ContainersTemp c= scores_[i]->get_input_containers(p);
    ret.insert(ret.end(), c.begin(), c.end());
  }
  return ret;
}


Restraints MinimumPairScore
::create_current_decomposition(const ParticlePair& vt) const {
  Restraints ret;
  MinimumPairScoreMS bestn
    = find_minimal_set_MinimumPairScore(scores_.begin(),
                                              scores_.end(), vt, n_);
  for (unsigned int i=0; i< bestn.size(); ++i) {
    ret.push_back(new core::PairRestraint(bestn[i].second, vt));
  }
  return ret;
}


void MinimumPairScore::do_show(std::ostream &out) const
{
  out << "size scores " << scores_.size() << std::endl;
}

IMPCONTAINER_END_NAMESPACE
