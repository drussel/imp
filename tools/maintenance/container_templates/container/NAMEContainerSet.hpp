/**
 *  \file IMP/container/CLASSNAMEContainerSet.h
 *  \brief Store a set of CLASSNAMEContainers
 *
 *  This file is generated by a script (tools/maintenance/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */

#ifndef IMPCONTAINER_HEADERNAME_CONTAINER_SET_H
#define IMPCONTAINER_HEADERNAME_CONTAINER_SET_H

#include "container_config.h"
#include <IMP/CLASSNAMEContainer.h>
#include <IMP/container_macros.h>
#include <IMP/internal/container_helpers.h>
#include <IMP/scoped.h>

IMPCONTAINER_BEGIN_NAMESPACE

//! Stores a set of CLASSNAMEContainers
/** The input sets must be disjoint. This can change if there is
    demand for it.

    \usesconstraint
*/
class IMPCONTAINEREXPORT CLASSNAMEContainerSet
  : public CLASSNAMEContainer
{
  static CLASSNAMEContainerSet* get_set(CLASSNAMEContainer* c) {
    return dynamic_cast<CLASSNAMEContainerSet*>(c);
  }
 public:
  //! Construct and empty set
  CLASSNAMEContainerSet(Model *m,
                        std::string name="CLASSNAMEContainerSet %1%");

  CLASSNAMEContainerSet(const CLASSNAMEContainersTemp &pc,
                        std::string name="CLASSNAMEContainerSet %1%");

  /** \brief apply modifer sm to all LCCLASSNAME containers */
  void apply(const CLASSNAMEModifier *sm) const;

  /** \brief apply derivative modifer sm to all LCCLASSNAME containers

      @param[in]   sm the derivate modifier to be applied
      @param[out]  da derivative accumulator when applying sm
   */
  void apply(const CLASSNAMEDerivativeModifier *sm,
             DerivativeAccumulator &da) const;

 /** \brief evaluates all LCCLASSNAME containers using LCCLASSNAME score

     @param[in]   s  LCCLASSNAME score to evaluate each LCCLASSNAME container
     @param[out]  da derivative accumulator when scoring each LCCLASSNAME

     @return the sum of evaluation over all LCCLASSNAME containers
  */
  double evaluate(const CLASSNAMEScore *s,
                  DerivativeAccumulator *da) const;

 /** \brief evaluates all LCCLASSNAME containers as long as below some maximal
            score threshold

     Evaluates all LCCLASSNAME containers using LCCLASSNAME score s
     until the specified maximal total evaluation score is breached,
     in order to save futile computation time.

     @param[in]   s   LCCLASSNAME score for evaluating each LCCLASSNAME
                      container
     @param[out]  da  derivative accumulator when scoring each LCCLASSNAME
     @param[in]   max the maximal total evaluation score that is allowed

     @return the sum of evaluation at the end of evaluation, or right after
             the maximal value was first breached
  */
  double evaluate_if_good(const CLASSNAMEScore *s,
                          DerivativeAccumulator *da,
                          double max) const;

  /** \brief apply template derivative modifer sm to all LCCLASSNAME containers

      @param[in]   sm the template derivate modifier to be applied
      @param[out]  da derivative accumulator when applying sm
  */
  template <class SM>
    void template_apply(const SM *sm,
                        DerivativeAccumulator &da) const {
    for (unsigned int i=0; i< get_number_of_CLASSFUNCTIONNAME_containers(); ++i)
      {
        get_CLASSFUNCTIONNAME_container(i)->apply(sm, da);
      }
  }

  /** \brief apply template modifer sm to all LCCLASSNAME containers */
  template <class SM>
    void template_apply(const SM *sm) const {
    for (unsigned int i=0; i< get_number_of_CLASSFUNCTIONNAME_containers(); ++i)
      {
        get_CLASSFUNCTIONNAME_container(i)->apply(sm);
      }
  }

  /** \brief evaluates all LCCLASSNAME containers using template LCCLASSNAME
             score s

      @param[in]   s  the template for scoring each LCCLASSNAME container
      @param[out]  da derivative accumulator when scoring each LCCLASSNAME

      @return the sum of evaluation over all LCCLASSNAME containers
  */
  template <class SS>
    double template_evaluate(const SS *s,
                             DerivativeAccumulator *da) const {
    double ret=0;
    for (unsigned int i=0; i< get_number_of_CLASSFUNCTIONNAME_containers(); ++i)
      {
        ret+=get_CLASSFUNCTIONNAME_container(i)->evaluate(s, da);
      }
    return ret;
  }

 /** \brief evaluates all LCCLASSNAME containers as long as below some maximal
            score threshold

     evaluates all LCCLASSNAME containers using template LCCLASSNAME score s,
     terminates if the specified maximal total evaluation score is breached
     in order to save futile computation time.

     @param[in]   s   the template for scoring each LCCLASSNAME container
     @param[out]  da  derivative accumulator when scoring each LCCLASSNAME
     @param[in]   max the maximal total evaluation score that is allowed

     @return the sum of evaluation at the end of the evaluation, or right after
             the maximum value was first breached
  */
  template <class SS>
    double template_evaluate_if_good(const SS *s,
                                 DerivativeAccumulator *da, double max) const {
    double ret=0;
    for (unsigned int i=0; i< get_number_of_CLASSFUNCTIONNAME_containers(); ++i)
      {
        double cur=
          get_CLASSFUNCTIONNAME_container(i)->evaluate_if_good(s, da, max);
        ret+=cur;
        max-=cur;
        if (max < 0) break;
      }
    return ret;
  }

  bool get_is_changed() const;
  ParticlesTemp get_all_possible_particles() const;
  IMP_OBJECT(CLASSNAMEContainerSet);

  /** @name Methods to control the nested container

      This container merges a set of nested containers. To add
      or remove nested containers, use the methods below.
  */
  /**@{*/
  IMP_LIST_ACTION(public, CLASSNAMEContainer, CLASSNAMEContainers,
                  CLASSFUNCTIONNAME_container, CLASSFUNCTIONNAME_containers,
                  CLASSNAMEContainer*, CLASSNAMEContainers,
                  {
                    obj->set_was_used(true);
                    set_is_changed(true);
                  },{},
                  );
  /**@}*/
#ifndef IMP_DOXYGEN
  PLURALINDEXTYPE get_indexes() const;
  PLURALINDEXTYPE get_range_indexes() const;
  ModelObjectsTemp do_get_inputs() const;
  void do_before_evaluate();
#endif
};


IMPCONTAINER_END_NAMESPACE

#endif  /* IMPCONTAINER_HEADERNAME_CONTAINER_SET_H */
