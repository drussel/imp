/**
 *  \file MinMaxGroupnameScore.cpp  \brief Define GroupnameScore
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2010 Sali Lab. All rights reserved.
 */

#include <IMP/container/MinMaxGroupnameScore.h>
#include "IMP/core/internal/MinimalSet.h"

IMPCONTAINER_BEGIN_NAMESPACE

namespace {
  unsigned int next_index=0;
}

MinMaxGroupnameScore::MinMaxGroupnameScore(const GroupnameScoresTemp &scores,
                                           unsigned int n,
                                           std::string name):
  GroupnameScore(IMP::internal::make_object_name(name, next_index++)),
  scores_(scores),
  n_(n)
{
}


namespace {
  typedef core::internal::MinimalSet<double,
          GroupnameScore*, std::COMPARATOR<double> > MS;
  template <class It>
  MS find_minimal_set(It b, It e, PassValue v, unsigned int n) {
    IMP_LOG(TERSE, "Finding MinMax " << n << " of "
            << std::distance(b,e) << std::endl);
    MS bestn(n);
    for (It it= b; it != e; ++it) {
      double score= (*it)->evaluate(v, NULL);

      if (bestn.can_insert(score)) {
        bestn.insert(score, *it);
      }
    }
    return bestn;
  }
}

double MinMaxGroupnameScore::evaluate(PassValue v,
                                      DerivativeAccumulator *da) const {
  MS bestn= find_minimal_set(scores_.begin(),
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

bool MinMaxGroupnameScore::get_is_changed(PassValue v) const {
  for (unsigned int i=0; i< scores_.size(); ++i) {
    if (scores_[i]->get_is_changed(v)) return true;
  }
  return false;
}


ParticlesList MinMaxGroupnameScore
::get_interacting_particles(PassValue p) const {
  ParticlesList ret;
  for (unsigned int i=0; i< scores_.size(); ++i) {
    ParticlesList c= scores_[i]->get_interacting_particles(p);
    ret.insert(ret.end(), c.begin(), c.end());
  }
  return ret;
}

ParticlesTemp MinMaxGroupnameScore
::get_input_particles(PassValue p) const {
  ParticlesTemp ret;
  for (unsigned int i=0; i< scores_.size(); ++i) {
    ParticlesTemp c= scores_[i]->get_input_particles(p);
    ret.insert(ret.end(), c.begin(), c.end());
  }
  return ret;
}

ContainersTemp MinMaxGroupnameScore
::get_input_containers(PassValue p) const {
  ContainersTemp ret;
  for (unsigned int i=0; i< scores_.size(); ++i) {
    ContainersTemp c= scores_[i]->get_input_containers(p);
    ret.insert(ret.end(), c.begin(), c.end());
  }
  return ret;
}


void MinMaxGroupnameScore::do_show(std::ostream &out) const
{
  out << "size scores " << scores_.size() << std::endl;
}

IMPCONTAINER_END_NAMESPACE
