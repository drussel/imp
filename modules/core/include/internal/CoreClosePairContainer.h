/**
 *  \file core/internal/CoreClosePairContainer.h
 *  \brief Return all pairs from a SingletonContainer
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2013 IMP Inventors. Close rights reserved.
 */

#ifndef IMPCORE_INTERNAL_CORE_CLOSE_PAIR_CONTAINER_H
#define IMPCORE_INTERNAL_CORE_CLOSE_PAIR_CONTAINER_H

#include <IMP/core/core_config.h>
#include "../ClosePairsFinder.h"
#include "MovedSingletonContainer.h"
#include "../PairRestraint.h"
#include <IMP/PairContainer.h>
#include <IMP/PairPredicate.h>
#include <IMP/generic.h>
#include <IMP/SingletonContainer.h>
#include <IMP/internal/ListLikePairContainer.h>

IMPCORE_BEGIN_INTERNAL_NAMESPACE


class IMPCOREEXPORT CoreClosePairContainer :
  public IMP::internal::ListLikePairContainer
{
  IMP::OwnerPointer<SingletonContainer> c_;
  IMP::OwnerPointer<ClosePairsFinder> cpf_;
  IMP::OwnerPointer<internal::MovedSingletonContainer> moved_;
  unsigned int moved_count_;
  bool first_call_;
  double distance_, slack_;
  IMP_LISTLIKE_PAIR_CONTAINER_2(CoreClosePairContainer);
  void initialize(SingletonContainer *c, double distance,
                  double slack, ClosePairsFinder *cpf);

  void check_duplicates_input() const;
  void check_list(bool include_slack) const;
  void do_first_call();
  void do_incremental();
  void do_rebuild();
public:
  CoreClosePairContainer(SingletonContainer *c, double distance,
                         ClosePairsFinder *cpf,
                         double slack=1);

  IMP_LIST_ACTION(public, PairFilter, PairFilters,
                  pair_filter, pair_filters,
                  PairPredicate*, PairPredicates,
                  obj->set_was_used(true);,
                  ,);

  void clear_caches() {first_call_=true;}
public:
  double get_slack() const {return slack_;}
  double get_distance() const {return distance_;}
  void update() {
    do_before_evaluate();
  }
  SingletonContainer*get_singleton_container() const {return c_;}
  ClosePairsFinder *get_close_pairs_finder() const {return cpf_;}
  void set_slack(double d);
  Restraints create_decomposition(PairScore *ps) const {
    ParticleIndexPairs all= get_range_indexes();
    Restraints ret(all.size());
    for (unsigned int i=0; i< all.size(); ++i) {
      ret[i]= new PairRestraint(ps, IMP::internal::get_particle(get_model(),
                                                                all[i]));
    }
    return ret;
  }
  template <class PS>
  Restraints create_decomposition_t(PS *ps) const {
    ParticleIndexPairs all= get_range_indexes();
    Restraints ret(all.size());
    for (unsigned int i=0; i< all.size(); ++i) {
      ret[i]= IMP::create_restraint(ps,
                                     IMP::internal::get_particle(get_model(),
                                                                 all[i]));
    }
    return ret;
  }
};

IMP_OBJECTS(CoreClosePairContainer, CoreClosePairContainers);

IMPCORE_END_INTERNAL_NAMESPACE

#endif  /* IMPCORE_INTERNAL_CORE_CLOSE_PAIR_CONTAINER_H */
