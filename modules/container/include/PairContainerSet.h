/**
 *  \file IMP/container/PairContainerSet.h
 *  \brief Store a set of PairContainers
 *
 *  This file is generated by a script (tools/maintenance/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */

#ifndef IMPCONTAINER_PAIR_CONTAINER_SET_H
#define IMPCONTAINER_PAIR_CONTAINER_SET_H

#include "container_config.h"
#include <IMP/PairContainer.h>
#include <IMP/container_macros.h>
#include <IMP/internal/container_helpers.h>
#include <IMP/scoped.h>

IMPCONTAINER_BEGIN_NAMESPACE

//! Stores a set of PairContainers
/** The input sets must be disjoint. This can change if there is
    demand for it.

    \usesconstraint
*/
class IMPCONTAINEREXPORT PairContainerSet
  : public PairContainer
{
  static PairContainerSet* get_set(PairContainer* c) {
    return dynamic_cast<PairContainerSet*>(c);
  }
 public:
  //! Construct and empty set
  PairContainerSet(Model *m,
                        std::string name="PairContainerSet %1%");

  PairContainerSet(const PairContainersTemp &pc,
                        std::string name="PairContainerSet %1%");

  /** \brief apply modifer sm to all pair containers */
  void apply(const PairModifier *sm) const;

  /** \brief apply derivative modifer sm to all pair containers

      @param[in]   sm the derivate modifier to be applied
      @param[out]  da derivative accumulator when applying sm
   */
  void apply(const PairDerivativeModifier *sm,
             DerivativeAccumulator &da) const;

 /** \brief evaluates all pair containers using pair score

     @param[in]   s  pair score to evaluate each pair container
     @param[out]  da derivative accumulator when scoring each pair

     @return the sum of evaluation over all pair containers
  */
  double evaluate(const PairScore *s,
                  DerivativeAccumulator *da) const;

 /** \brief evaluates all pair containers as long as below some maximal
            score threshold

     Evaluates all pair containers using pair score s
     until the specified maximal total evaluation score is breached,
     in order to save futile computation time.

     @param[in]   s   pair score for evaluating each pair
                      container
     @param[out]  da  derivative accumulator when scoring each pair
     @param[in]   max the maximal total evaluation score that is allowed

     @return the sum of evaluation at the end of evaluation, or right after
             the maximal value was first breached
  */
  double evaluate_if_good(const PairScore *s,
                          DerivativeAccumulator *da,
                          double max) const;

  /** \brief apply template derivative modifer sm to all pair containers

      @param[in]   sm the template derivate modifier to be applied
      @param[out]  da derivative accumulator when applying sm
  */
  template <class SM>
    void template_apply(const SM *sm,
                        DerivativeAccumulator &da) const {
    for (unsigned int i=0; i< get_number_of_pair_containers(); ++i)
      {
        get_pair_container(i)->apply(sm, da);
      }
  }

  /** \brief apply template modifer sm to all pair containers */
  template <class SM>
    void template_apply(const SM *sm) const {
    for (unsigned int i=0; i< get_number_of_pair_containers(); ++i)
      {
        get_pair_container(i)->apply(sm);
      }
  }

  /** \brief evaluates all pair containers using template pair
             score s

      @param[in]   s  the template for scoring each pair container
      @param[out]  da derivative accumulator when scoring each pair

      @return the sum of evaluation over all pair containers
  */
  template <class SS>
    double template_evaluate(const SS *s,
                             DerivativeAccumulator *da) const {
    double ret=0;
    for (unsigned int i=0; i< get_number_of_pair_containers(); ++i)
      {
        ret+=get_pair_container(i)->evaluate(s, da);
      }
    return ret;
  }

 /** \brief evaluates all pair containers as long as below some maximal
            score threshold

     evaluates all pair containers using template pair score s,
     terminates if the specified maximal total evaluation score is breached
     in order to save futile computation time.

     @param[in]   s   the template for scoring each pair container
     @param[out]  da  derivative accumulator when scoring each pair
     @param[in]   max the maximal total evaluation score that is allowed

     @return the sum of evaluation at the end of the evaluation, or right after
             the maximum value was first breached
  */
  template <class SS>
    double template_evaluate_if_good(const SS *s,
                                 DerivativeAccumulator *da, double max) const {
    double ret=0;
    for (unsigned int i=0; i< get_number_of_pair_containers(); ++i)
      {
        double cur=
          get_pair_container(i)->evaluate_if_good(s, da, max);
        ret+=cur;
        max-=cur;
        if (max < 0) break;
      }
    return ret;
  }

  bool get_is_changed() const;
  ParticleIndexes get_all_possible_indexes() const;
  IMP_OBJECT(PairContainerSet);

  /** @name Methods to control the nested container

      This container merges a set of nested containers. To add
      or remove nested containers, use the methods below.
  */
  /**@{*/
  IMP_LIST_ACTION(public, PairContainer, PairContainers,
                  pair_container, pair_containers,
                  PairContainer*, PairContainers,
                  {
                    obj->set_was_used(true);
                    set_is_changed(true);
                  },{},
                  );
  /**@}*/
#ifndef IMP_DOXYGEN
  ParticleIndexPairs get_indexes() const;
  ParticleIndexPairs get_range_indexes() const;
  ModelObjectsTemp do_get_inputs() const;
  void do_before_evaluate();
#endif
};


IMPCONTAINER_END_NAMESPACE

#endif  /* IMPCONTAINER_PAIR_CONTAINER_SET_H */
