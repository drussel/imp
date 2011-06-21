/**
 *  \file PairContainerSet.h
 *  \brief Store a set of PairContainers
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 */

#ifndef IMPCONTAINER_PAIR_CONTAINER_SET_H
#define IMPCONTAINER_PAIR_CONTAINER_SET_H

#include "container_config.h"
#include <IMP/PairContainer.h>
#include <IMP/container_macros.h>
#include <IMP/internal/container_helpers.h>

IMPCONTAINER_BEGIN_NAMESPACE

//! Stores a set of PairContainers
/** The input sets must be disjoint. This can change if there is
    demand for it.

    \usesconstraint
*/
class IMPCONTAINEREXPORT PairContainerSet
  : public PairContainer
{
  IMP_CONTAINER_DEPENDENCIES(PairContainerSet,
                             {
                               ret.insert(ret.end(),
                                          back_->pair_containers_begin(),
                                          back_->pair_containers_end());
                             });
  // to not have added and removed
  PairContainerSet();
  PairContainerPair get_added_and_removed_containers() const {
    PairContainerSet *added= create_untracked_container();
    PairContainerSet *removed=create_untracked_container();
    for (unsigned int i=0; i< get_number_of_pair_containers(); ++i) {
      added->add_pair_container(get_pair_container(i)
                                     ->get_added_container());
      removed->add_pair_container(get_pair_container(i)
                                       ->get_removed_container());
    }
    return PairContainerPair(added, removed);
  }
 public:
  //! Construct and empty set
  PairContainerSet(Model *m,
                        std::string name="PairContainerSet %1%");

  PairContainerSet(const PairContainersTemp &pc,
                        std::string name="PairContainerSet %1%");

  bool get_contains_particle_pair(const ParticlePair&) const;
  unsigned int get_number_of_particle_pairs() const;
  ParticlePair get_particle_pair(unsigned int i) const;
  void apply(const PairModifier *sm);
  void apply(const PairModifier *sm,
             DerivativeAccumulator &da);
  double evaluate(const PairScore *s,
                  DerivativeAccumulator *da) const;
 template <class SM>
  void template_apply(const SM *sm,
                      DerivativeAccumulator &da) {
   for (unsigned int i=0; i< get_number_of_pair_containers(); ++i) {
     get_pair_container(i)->apply(sm, da);
   }
 }
  template <class SM>
  void template_apply(const SM *sm) {
    for (unsigned int i=0; i< get_number_of_pair_containers(); ++i) {
      get_pair_container(i)->apply(sm);
    }
  }
  template <class SS>
  double template_evaluate(const SS *s,
                           DerivativeAccumulator *da) const {
    double ret=0;
    for (unsigned int i=0; i< get_number_of_pair_containers(); ++i) {
      ret+=get_pair_container(i)->evaluate(s, da);
    }
    return ret;
  }

  ParticlesTemp get_contained_particles() const;
  IMP_OBJECT(PairContainerSet);

  /** @name Methods to control the nested container

      This container merges a set of nested containers. To add
      or remove nested containers, use the methods below.
  */
  /**@{*/
  IMP_LIST(public, PairContainer, pair_container,
           PairContainer*, PairContainers);
  /**@}*/

  static PairContainerSet *create_untracked_container() {
    PairContainerSet *lsc = new PairContainerSet();
    return lsc;
  }
#ifndef IMP_DOXYGEN
  bool get_is_up_to_date() const {
    for (unsigned int i=0;
         i< get_number_of_pair_containers(); ++i) {
      if (!get_pair_container(i)->get_is_up_to_date()) return false;
    }
    return true;
  }
#endif
};


IMPCONTAINER_END_NAMESPACE

#endif  /* IMPCONTAINER_PAIR_CONTAINER_SET_H */
