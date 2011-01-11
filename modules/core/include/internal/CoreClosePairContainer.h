/**
 *  \file core/internal/CoreClosePairContainer.h
 *  \brief Return all pairs from a SingletonContainer
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2011 IMP Inventors. Close rights reserved.
 */

#ifndef IMPCORE_INTERNAL_CORE_CLOSE_PAIR_CONTAINER_H
#define IMPCORE_INTERNAL_CORE_CLOSE_PAIR_CONTAINER_H

#include "../core_config.h"
#include "../ClosePairsFinder.h"
#include "MovedSingletonContainer.h"
#include <IMP/PairContainer.h>
#include <IMP/PairFilter.h>
#include <IMP/SingletonContainer.h>
#include "pair_helpers.h"

IMPCORE_BEGIN_INTERNAL_NAMESPACE


class IMPCOREEXPORT CoreClosePairContainer :
public internal::ListLikePairContainer
{
  IMP::internal::OwnerPointer<SingletonContainer> c_;
  IMP::internal::OwnerPointer<ClosePairsFinder> cpf_;
  IMP::internal::OwnerPointer<internal::MovedSingletonContainer> moved_;
  ParticlesTemp previous_moved_;
  bool first_call_;
  double distance_, slack_;
  IMP_ACTIVE_CONTAINER_DECL(CoreClosePairContainer);
  void initialize(SingletonContainer *c, double distance,
                  double slack, ClosePairsFinder *cpf);
public:
  CoreClosePairContainer(SingletonContainer *c, double distance,
                         ClosePairsFinder *cpf,
                         double slack=1);

  IMP_LIST(public, PairFilter, pair_filter,
           PairFilter*, PairFilters);
#ifndef IMP_DOXYGEN
  bool get_is_up_to_date() const {
    if (get_model()->get_stage() != Model::NOT_EVALUATING) {
      return get_last_update_evaluation() == get_model()->get_evaluation();
    } else {
      if (!c_->get_is_up_to_date()) return false;
      bool ret=true;
      IMP_FOREACH_SINGLETON(c_,
                            ret= !(imp_foreach_break
                                   =_1->get_is_changed()););
      return ret;
    }
  }
#endif
public:
  double get_distance() const {return distance_;}
  virtual std::string get_type_name() const {return "CoreClosePairContainer";}
  virtual ::IMP::VersionInfo get_version_info() const {
    return get_module_version_info();
  }
  void update() {
    do_before_evaluate();
  }
  SingletonContainer*get_singleton_container() const {return c_;}
  bool get_contained_particles_changed() const;
  ParticlesTemp get_contained_particles() const;
  ClosePairsFinder *get_close_pairs_finder() const {return cpf_;}
  void set_slack(double d);
  IMP_NO_DOXYGEN(virtual void do_show(std::ostream &out) const);
  IMP_REF_COUNTED_NONTRIVIAL_DESTRUCTOR(CoreClosePairContainer);
};

IMP_OBJECTS(CoreClosePairContainer, CoreClosePairContainers);

IMPCORE_END_INTERNAL_NAMESPACE

#endif  /* IMPCORE_INTERNAL_CORE_CLOSE_PAIR_CONTAINER_H */
