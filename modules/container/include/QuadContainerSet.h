/**
 *  \file IMP/container/QuadContainerSet.h
 *  \brief Store a set of QuadContainers
 *
 *  This file is generated by a script (tools/maintenance/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */

#ifndef IMPCONTAINER_QUAD_CONTAINER_SET_H
#define IMPCONTAINER_QUAD_CONTAINER_SET_H

#include <IMP/container/container_config.h>
#include <IMP/QuadContainer.h>
#include <IMP/container_macros.h>
#include <IMP/internal/container_helpers.h>
#include <IMP/scoped.h>

IMPCONTAINER_BEGIN_NAMESPACE

//! Stores a set of QuadContainers
/** The input sets must be disjoint. This can change if there is
    demand for it.

    \usesconstraint
*/
class IMPCONTAINEREXPORT QuadContainerSet
  : public QuadContainer
{
  static QuadContainerSet* get_set(QuadContainer* c) {
    return dynamic_cast<QuadContainerSet*>(c);
  }
 public:
  //! Construct and empty set
  QuadContainerSet(Model *m,
                        std::string name="QuadContainerSet %1%");

  QuadContainerSet(const QuadContainersTemp &pc,
                        std::string name="QuadContainerSet %1%");

  /** \brief apply modifer sm to all quad containers */
  IMP_IMPLEMENT(void do_apply(const QuadModifier *sm) const);

  template <class M>
      void apply_generic(const M*m) const {
    apply(m);
  }

  ParticleIndexes get_all_possible_indexes() const;
  IMP_OBJECT(QuadContainerSet);

  /** @name Methods to control the nested container

      This container merges a set of nested containers. To add
      or remove nested containers, use the methods below.
  */
  /**@{*/
  IMP_LIST_ACTION(public, QuadContainer, QuadContainers,
                  quad_container, quad_containers,
                  QuadContainer*, QuadContainers,
                  {
                    obj->set_was_used(true);
                    set_is_changed(true);
                  },{},
                  );
  /**@}*/

#ifndef IMP_DOXYGEN
  ParticleIndexQuads get_indexes() const;
  ParticleIndexQuads get_range_indexes() const;
  ModelObjectsTemp do_get_inputs() const;
#endif

  IMP_IMPLEMENT(void do_before_evaluate());
};


IMPCONTAINER_END_NAMESPACE

#endif  /* IMPCONTAINER_QUAD_CONTAINER_SET_H */
