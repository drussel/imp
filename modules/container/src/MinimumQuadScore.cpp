/**
 *  \file MinimumQuadScore.cpp  \brief Define QuadScore
 *
 *  WARNING This file was generated from MinimumNAMEScore.cc
 *  in tools/maintenance/container_templates/container
 *  by tools/maintenance/make-container.
 *
 *  Copyright 2007-2013 IMP Inventors. All rights reserved.
 */

#include <IMP/container/MinimumQuadScore.h>
#include <IMP/core/QuadRestraint.h>
#include "IMP/algebra/internal/MinimalSet.h"

IMPCONTAINER_BEGIN_NAMESPACE

MinimumQuadScore::MinimumQuadScore(const QuadScoresTemp &scores,
                                           unsigned int n,
                                           std::string name):
  QuadScore(name),
  scores_(scores.begin(), scores.end()),
  n_(n)
{
}


namespace {
  typedef algebra::internal::MinimalSet<double,
          QuadScore*, std::less<double> >
  MinimumQuadScoreMS;
  template <class It>
  MinimumQuadScoreMS
  find_minimal_set_MinimumQuadScore(It b, It e,
                                         Model *m,
                                         const ParticleIndexQuad& v,
                                          unsigned int n) {
    IMP_LOG(TERSE, "Finding Minimum " << n << " of "
            << std::distance(b,e) << std::endl);
    MinimumQuadScoreMS bestn(n);
    for (It it= b; it != e; ++it) {
      double score= (*it)->evaluate_index(m, v, nullptr);
      bestn.insert(score, *it);
    }
    return bestn;
  }
}

double MinimumQuadScore
::evaluate_index(Model *m, const ParticleIndexQuad& v,
                 DerivativeAccumulator *da) const {
  MinimumQuadScoreMS bestn
    = find_minimal_set_MinimumQuadScore(scores_.begin(),
                                             scores_.end(), m, v, n_);

  double score=0;
  for (unsigned int i=0; i< bestn.size(); ++i) {
    if (da) {
      bestn[i].second->evaluate_index(m, v, da);
    }
    score+= bestn[i].first;
  }
  return score;
}

ModelObjectsTemp MinimumQuadScore
::do_get_inputs(Model *m,
             const ParticleIndexes &pis) const {
  ModelObjectsTemp ret;
  for (unsigned int i=0; i< scores_.size(); ++i) {
    ret+= scores_[i]->get_inputs(m, pis);
  }
  return ret;
}

Restraints MinimumQuadScore
::do_create_current_decomposition(Model *m,
                               const ParticleIndexQuad& vt) const {
  Restraints ret;
  MinimumQuadScoreMS bestn
    = find_minimal_set_MinimumQuadScore(scores_.begin(),
                                             scores_.end(), m, vt, n_);
  for (unsigned int i=0; i< bestn.size(); ++i) {
    ret.push_back(IMP::internal::create_tuple_restraint(bestn[i].second,
                                                        m, vt));
    ret.back()->set_last_score(bestn[i].first);
  }
  return ret;
}


void MinimumQuadScore::do_show(std::ostream &out) const
{
  out << "size scores " << scores_.size() << std::endl;
}

IMPCONTAINER_END_NAMESPACE
