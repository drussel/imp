/**
 *  \file MinimumQuadScoreRestraint.cpp
 *  \brief The minimum score over a container.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 *
 */

#include "IMP/core/MinimumQuadScoreRestraint.h"
#include "IMP/core/internal/MinimalSet.h"
#include <IMP/internal/container_helpers.h>
#include <IMP/internal/utility.h>


IMPCORE_BEGIN_NAMESPACE

MinimumQuadScoreRestraint
::MinimumQuadScoreRestraint(QuadScore *f,
                                 QuadContainer *c,
                                 unsigned int n,
                                 std::string name):
  Restraint(name),
  f_(f), c_(c), n_(n){
}

namespace {
  typedef internal::MinimalSet<double,
    QuadContainer::ParticleQuadIterator> MS;
  template <class It, class F>
  MS find_minimal_set(It b, It e, F *f, unsigned int n) {
    IMP_LOG(TERSE, "Finding lowest " << n << " of "
            << std::distance(b,e) << std::endl);
    MS bestn(n);
    for (It it= b; it != e; ++it) {
      double score= f->evaluate(*it, NULL);

      if (bestn.can_insert(score)) {
        bestn.insert(score, it);
      }
    }
    return bestn;
  }
}

double MinimumQuadScoreRestraint
::unprotected_evaluate(DerivativeAccumulator *da) const {
  MS bestn= find_minimal_set(c_->particle_quads_begin(),
                             c_->particle_quads_end(), f_.get(), n_);

  double score=0;
  for (unsigned int i=0; i< bestn.size(); ++i) {
    if (da) {
      f_->evaluate(*bestn[i].second, da);
    }
    score+= bestn[i].first;
  }

  return score;
}

void MinimumQuadScoreRestraint::show(std::ostream &out) const {
  out << "MinimumQuadScoreRestraint over ";
  c_->show(out);
  out << " using function ";
  f_->show(out);
  out << std::endl;
}


ParticlesList MinimumQuadScoreRestraint::get_interacting_particles() const
{
  MS bestn= find_minimal_set(c_->particle_quads_begin(),
                             c_->particle_quads_end(), f_.get(), n_);
  ParticlesList ret;
  for (unsigned int i=0; i< bestn.size(); ++i) {
    ParticlesList pt=f_->get_interacting_particles(*bestn[i].second);
    if (!pt.empty()) {
      ret.push_back(IMP::internal::get_union(pt));
    }
  }

  return ret;
}


ParticlesTemp MinimumQuadScoreRestraint::get_input_particles() const
{
  return IMP::internal::get_input_particles(c_.get(), f_.get());
}

ObjectsTemp MinimumQuadScoreRestraint::get_input_objects() const
{
  return ObjectsTemp(1, c_);
}


IMPCORE_END_NAMESPACE
