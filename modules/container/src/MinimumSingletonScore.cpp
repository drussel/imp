/**
 *  \file MinimumSingletonScore.cpp  \brief Define SingletonScore
 *
 *  WARNING This file was generated from MinimumNAMEScore.cc
 *  in tools/maintenance/container_templates/container
 *  by tools/maintenance/make-container.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */

#include <IMP/container/MinimumSingletonScore.h>
#include <IMP/core/SingletonRestraint.h>
#include "IMP/algebra/internal/MinimalSet.h"

IMPCONTAINER_BEGIN_NAMESPACE

MinimumSingletonScore::MinimumSingletonScore(const SingletonScoresTemp &scores,
                                           unsigned int n,
                                           std::string name):
  SingletonScore(name),
  scores_(scores.begin(), scores.end()),
  n_(n)
{
}


namespace {
  typedef algebra::internal::MinimalSet<double,
          SingletonScore*, std::less<double> >
  MinimumSingletonScoreMS;
  template <class It>
  MinimumSingletonScoreMS
  find_minimal_set_MinimumSingletonScore(It b, It e, Particle* v,
                                          unsigned int n) {
    IMP_LOG(TERSE, "Finding Minimum " << n << " of "
            << std::distance(b,e) << std::endl);
    MinimumSingletonScoreMS bestn(n);
    for (It it= b; it != e; ++it) {
      double score= (*it)->evaluate(v, nullptr);
      bestn.insert(score, *it);
    }
    return bestn;
  }
}

double MinimumSingletonScore::evaluate(Particle* v,
                                      DerivativeAccumulator *da) const {
  MinimumSingletonScoreMS bestn
    = find_minimal_set_MinimumSingletonScore(scores_.begin(),
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

ParticlesTemp MinimumSingletonScore
::get_input_particles(Particle* p) const {
  ParticlesTemp ret;
  for (unsigned int i=0; i< scores_.size(); ++i) {
    ParticlesTemp c= scores_[i]->get_input_particles(p);
    ret.insert(ret.end(), c.begin(), c.end());
  }
  return ret;
}

ContainersTemp MinimumSingletonScore
::get_input_containers(Particle* p) const {
  ContainersTemp ret;
  for (unsigned int i=0; i< scores_.size(); ++i) {
    ContainersTemp c= scores_[i]->get_input_containers(p);
    ret.insert(ret.end(), c.begin(), c.end());
  }
  return ret;
}


Restraints MinimumSingletonScore
::create_current_decomposition(Particle* vt) const {
  Restraints ret;
  MinimumSingletonScoreMS bestn
    = find_minimal_set_MinimumSingletonScore(scores_.begin(),
                                              scores_.end(), vt, n_);
  for (unsigned int i=0; i< bestn.size(); ++i) {
    ret.push_back(new core::SingletonRestraint(bestn[i].second, vt));
  }
  return ret;
}


void MinimumSingletonScore::do_show(std::ostream &out) const
{
  out << "size scores " << scores_.size() << std::endl;
}

IMPCONTAINER_END_NAMESPACE
