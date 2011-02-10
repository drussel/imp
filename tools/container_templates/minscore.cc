/**
 *  \file MINORMAXCLASSNAMEScore.cpp  \brief Define CLASSNAMEScore
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 */

#include <IMP/container/MINORMAXCLASSNAMEScore.h>
#include "IMP/algebra/internal/MinimalSet.h"

IMPCONTAINER_BEGIN_NAMESPACE

MINORMAXCLASSNAMEScore::MINORMAXCLASSNAMEScore(const CLASSNAMEScoresTemp &scores,
                                           unsigned int n,
                                           std::string name):
  CLASSNAMEScore(name),
  scores_(scores),
  n_(n)
{
}


namespace {
  typedef algebra::internal::MinimalSet<double,
          CLASSNAMEScore*, std::COMPARATOR<double> >
  MINORMAXCLASSNAMEScoreMS;
  template <class It>
  MINORMAXCLASSNAMEScoreMS
  find_minimal_set_MINORMAXCLASSNAMEScore(It b, It e, ARGUMENTTYPE v,
                                          unsigned int n) {
    IMP_LOG(TERSE, "Finding MINORMAX " << n << " of "
            << std::distance(b,e) << std::endl);
    MINORMAXCLASSNAMEScoreMS bestn(n);
    for (It it= b; it != e; ++it) {
      double score= (*it)->evaluate(v, NULL);
      bestn.insert(score, *it);
    }
    return bestn;
  }
}

double MINORMAXCLASSNAMEScore::evaluate(ARGUMENTTYPE v,
                                      DerivativeAccumulator *da) const {
  MINORMAXCLASSNAMEScoreMS bestn
    = find_minimal_set_MINORMAXCLASSNAMEScore(scores_.begin(),
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

bool MINORMAXCLASSNAMEScore::get_is_changed(ARGUMENTTYPE v) const {
  for (unsigned int i=0; i< scores_.size(); ++i) {
    if (scores_[i]->get_is_changed(v)) return true;
  }
  return false;
}


ParticlesTemp MINORMAXCLASSNAMEScore
::get_input_particles(Particle* p) const {
  ParticlesTemp ret;
  for (unsigned int i=0; i< scores_.size(); ++i) {
    ParticlesTemp c= scores_[i]->get_input_particles(p);
    ret.insert(ret.end(), c.begin(), c.end());
  }
  return ret;
}

ContainersTemp MINORMAXCLASSNAMEScore
::get_input_containers(Particle* p) const {
  ContainersTemp ret;
  for (unsigned int i=0; i< scores_.size(); ++i) {
    ContainersTemp c= scores_[i]->get_input_containers(p);
    ret.insert(ret.end(), c.begin(), c.end());
  }
  return ret;
}


void MINORMAXCLASSNAMEScore::do_show(std::ostream &out) const
{
  out << "size scores " << scores_.size() << std::endl;
}

IMPCONTAINER_END_NAMESPACE
