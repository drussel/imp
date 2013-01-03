/**
 *  \file MinimumCLASSNAMERestraint.cpp
 *  \brief The minimum score over a container.
 *
 *  BLURB
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#include "IMP/container/MinimumCLASSNAMERestraint.h"
#include "IMP/algebra/internal/MinimalSet.h"
#include <IMP/internal/container_helpers.h>
#include <IMP/core/CLASSNAMERestraint.h>
#include <IMP/LCCLASSNAME_macros.h>
#include <IMP/container_macros.h>
#include <IMP/internal/TupleRestraint.h>

IMPCONTAINER_BEGIN_NAMESPACE

MinimumCLASSNAMERestraint
::MinimumCLASSNAMERestraint(CLASSNAMEScore *f,
                           CLASSNAMEContainerAdaptor c,
                           unsigned int n,
                           std::string name):
  Restraint(c->get_model(), name),
  f_(f), c_(c), n_(n){
}

namespace {
  typedef algebra::internal::MinimalSet<double,
          INDEXTYPE, std::less<double> > CLASSNAMEMinimumMS;
  template <class C, class F>
  CLASSNAMEMinimumMS find_minimal_set_CLASSNAMEMinimum(C* c, F *f,
                                                         unsigned int n) {
    IMP_LOG(VERBOSE, "Finding Minimum " << n << " of "
            << c->get_number() << std::endl);
    CLASSNAMEMinimumMS bestn(n);
    IMP_CONTAINER_FOREACH_TEMPLATE(C, c, {
        double score= f->evaluate_index(c->get_model(),
                                        _1, nullptr);
        IMP_LOG(VERBOSE, "Found " << score << " for "
                << _1 << std::endl);
        bestn.insert(score, _1);
      });
    return bestn;
  }
}

double MinimumCLASSNAMERestraint
::unprotected_evaluate(DerivativeAccumulator *da) const {
  IMP_OBJECT_LOG;
  CLASSNAMEMinimumMS bestn
    = find_minimal_set_CLASSNAMEMinimum(c_.get(),
                                         f_.get(), n_);

  double score=0;
  for (unsigned int i=0; i< bestn.size(); ++i) {
    if (da) {
      f_->evaluate_index(get_model(), bestn[i].second, da);
    }
    score+= bestn[i].first;
  }
  IMP_LOG(VERBOSE, "Total score is " << score << std::endl);
  return score;
}

double MinimumCLASSNAMERestraint
::unprotected_evaluate_if_good(DerivativeAccumulator *da,
                               double max) const {
  IMP_OBJECT_LOG;
  CLASSNAMEMinimumMS bestn
    = find_minimal_set_CLASSNAMEMinimum(c_.get(),
                                         f_.get(), n_);

  double score=0;
  for (unsigned int i=0; i< bestn.size(); ++i) {
    if (da) {
      f_->evaluate_index(get_model(), bestn[i].second, da);
    }
    score+= bestn[i].first;
    if (score > max) break;
  }
  IMP_LOG(VERBOSE, "Total score is " << score << std::endl);
  return score;
}

Restraints MinimumCLASSNAMERestraint
::do_create_current_decomposition() const {
  IMP_OBJECT_LOG;
  CLASSNAMEMinimumMS bestn
    = find_minimal_set_CLASSNAMEMinimum(c_.get(),
                                         f_.get(), n_);
  Restraints ret;
  for (unsigned int i=0; i< bestn.size(); ++i) {
    ret.push_back(IMP::internal::create_tuple_restraint(f_.get(),
                                                        get_model(),
                                                        bestn[i].second,
                                                        get_name()));
    ret.back()->set_last_score(bestn[i].first);
  }
  return ret;
}


void MinimumCLASSNAMERestraint::do_show(std::ostream &out) const {
  out << "container " << *c_ << std::endl;
  out << "score " << *f_ << std::endl;
}


ModelObjectsTemp MinimumCLASSNAMERestraint::do_get_inputs() const
{
  ModelObjectsTemp ret;
  ret+=f_->get_inputs(get_model(),
                      c_->get_all_possible_indexes());
  ret.push_back(c_);
  return ret;
}



IMPCONTAINER_END_NAMESPACE
