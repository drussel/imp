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

  template <class M>
      void apply_generic(const M*m) const {
    apply(m);
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
