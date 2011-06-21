/**
 *  \file TripletContainerSet.h
 *  \brief Store a set of TripletContainers
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 */

#ifndef IMPCONTAINER_TRIPLET_CONTAINER_SET_H
#define IMPCONTAINER_TRIPLET_CONTAINER_SET_H

#include "container_config.h"
#include <IMP/TripletContainer.h>
#include <IMP/container_macros.h>
#include <IMP/internal/container_helpers.h>

IMPCONTAINER_BEGIN_NAMESPACE

//! Stores a set of TripletContainers
/** The input sets must be disjoint. This can change if there is
    demand for it.

    \usesconstraint
*/
class IMPCONTAINEREXPORT TripletContainerSet
  : public TripletContainer
{
  IMP_CONTAINER_DEPENDENCIES(TripletContainerSet,
                             {
                               ret.insert(ret.end(),
                                          back_->triplet_containers_begin(),
                                          back_->triplet_containers_end());
                             });
  // to not have added and removed
  TripletContainerSet();
  TripletContainerPair get_added_and_removed_containers() const {
    TripletContainerSet *added= create_untracked_container();
    TripletContainerSet *removed=create_untracked_container();
    for (unsigned int i=0; i< get_number_of_triplet_containers(); ++i) {
      added->add_triplet_container(get_triplet_container(i)
                                     ->get_added_container());
      removed->add_triplet_container(get_triplet_container(i)
                                       ->get_removed_container());
    }
    return TripletContainerPair(added, removed);
  }
 public:
  //! Construct and empty set
  TripletContainerSet(Model *m,
                        std::string name="TripletContainerSet %1%");

  TripletContainerSet(const TripletContainersTemp &pc,
                        std::string name="TripletContainerSet %1%");

  bool get_contains_particle_triplet(const ParticleTriplet&) const;
  unsigned int get_number_of_particle_triplets() const;
  ParticleTriplet get_particle_triplet(unsigned int i) const;
  void apply(const TripletModifier *sm);
  void apply(const TripletModifier *sm,
             DerivativeAccumulator &da);
  double evaluate(const TripletScore *s,
                  DerivativeAccumulator *da) const;
 template <class SM>
  void template_apply(const SM *sm,
                      DerivativeAccumulator &da) {
   for (unsigned int i=0; i< get_number_of_triplet_containers(); ++i) {
     get_triplet_container(i)->apply(sm, da);
   }
 }
  template <class SM>
  void template_apply(const SM *sm) {
    for (unsigned int i=0; i< get_number_of_triplet_containers(); ++i) {
      get_triplet_container(i)->apply(sm);
    }
  }
  template <class SS>
  double template_evaluate(const SS *s,
                           DerivativeAccumulator *da) const {
    double ret=0;
    for (unsigned int i=0; i< get_number_of_triplet_containers(); ++i) {
      ret+=get_triplet_container(i)->evaluate(s, da);
    }
    return ret;
  }

  ParticlesTemp get_contained_particles() const;
  IMP_OBJECT(TripletContainerSet);

  /** @name Methods to control the nested container

      This container merges a set of nested containers. To add
      or remove nested containers, use the methods below.
  */
  /**@{*/
  IMP_LIST(public, TripletContainer, triplet_container,
           TripletContainer*, TripletContainers);
  /**@}*/

  static TripletContainerSet *create_untracked_container() {
    TripletContainerSet *lsc = new TripletContainerSet();
    return lsc;
  }
#ifndef IMP_DOXYGEN
  bool get_is_up_to_date() const {
    for (unsigned int i=0;
         i< get_number_of_triplet_containers(); ++i) {
      if (!get_triplet_container(i)->get_is_up_to_date()) return false;
    }
    return true;
  }
#endif
};


IMPCONTAINER_END_NAMESPACE

#endif  /* IMPCONTAINER_TRIPLET_CONTAINER_SET_H */
