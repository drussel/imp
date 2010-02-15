/**
 *  \file ListTripletContainer.h    \brief Store a list of ParticleTriplets
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2010 Sali Lab. All rights reserved.
 */

#ifndef IMPCONTAINER_LIST_TRIPLET_CONTAINER_H
#define IMPCONTAINER_LIST_TRIPLET_CONTAINER_H

#include "config.h"
#include <IMP/core/internal/CoreListTripletContainer.h>

IMPCONTAINER_BEGIN_NAMESPACE

//! Store a list of ParticleTriplets
/** \note The indexes can change when particles are inserted
    as the list is maintained in sorted order.
 */
class IMPCONTAINEREXPORT ListTripletContainer:
#if defined(IMP_DOXYGEN) || defined(SWIG)
public TripletContainer
#else
public core::internal::CoreListTripletContainer
#endif
{
  typedef core::internal::CoreListTripletContainer P;
  // for the change versions
  ListTripletContainer(bool);
public:
  //! construct and pass an initial set of particle_triplets
  ListTripletContainer(const ParticleTriplets &ps,
                         std::string name= "ListTripletContainer %1%");

  ListTripletContainer(std::string name= "ListTripletContainer %1%");
  ListTripletContainer(const char *name);

#if defined(IMP_DOXYGEN) || defined(SWIG)
 /** @name Methods to control the contained objects

     This container stores a list of ParticleTriplet objects. To manipulate
     the list use these methods.
  */
  /**@{*/
  void add_particle_triplet(const ParticleTriplet& vt);
  void add_particle_triplets(const ParticleTripletsTemp &c);
  void set_particle_triplets(ParticleTripletsTemp c);
  IMP_NO_DOXYGEN(void add_particle_triplets(const ParticleTriplets &c) {
      add_particle_triplets(static_cast<const ParticleTripletsTemp&>(c));
    })
  IMP_NO_DOXYGEN(void set_particle_triplets(const ParticleTriplets &c) {
      set_particle_triplets(static_cast<ParticleTripletsTemp>(c));
    })
  void clear_particle_triplets();
  /**@}*/
  IMP_TRIPLET_CONTAINER(ListTripletContainer);
#else
  static ListTripletContainer *create_untracked_container() {
    ListTripletContainer *lsc = new ListTripletContainer(false);
    return lsc;
  }
  IMP_OBJECT(ListTripletContainer);
#endif
};


IMPCONTAINER_END_NAMESPACE

#endif  /* IMPCONTAINER_LIST_TRIPLET_CONTAINER_H */
