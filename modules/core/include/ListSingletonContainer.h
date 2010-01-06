/**
 *  \file ListSingletonContainer.h    \brief Store a list of Particles
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2010 Sali Lab. All rights reserved.
 */

#ifndef IMPCORE_LIST_SINGLETON_CONTAINER_H
#define IMPCORE_LIST_SINGLETON_CONTAINER_H

#include "config.h"
#include <IMP/SingletonContainer.h>
#include <IMP/internal/container_helpers.h>
#include <IMP/core/internal/singleton_helpers.h>
#include <IMP/ScoreState.h>

IMPCORE_BEGIN_NAMESPACE

//! Store a list of Particles
/** \note The indexes can change when particles are inserted
    as the list is maintained in sorted order.
 */
class IMPCOREEXPORT ListSingletonContainer:
#if defined(IMP_DOXYGEN) || defined(SWIG)
public SingletonContainer
#else
public internal::ListLikeSingletonContainer
#endif
{
  IMP_ACTIVE_CONTAINER_DECL(ListSingletonContainer);
  // for the change versions
  ListSingletonContainer(bool);
public:
  //! construct and pass an initial set of particles
  ListSingletonContainer(const Particles &ps,
                         std::string name= "ListSingletonContainer %1%");

  ListSingletonContainer(std::string name= "ListSingletonContainer %1%");
  ListSingletonContainer(const char *name);

 /** @name Methods to control the contained objects

     This container stores a list of Particle objects. To manipulate
     the list use these methods.
  */
  /**@{*/
  void add_particle(Particle* vt);
  void add_particles(const ParticlesTemp &c);
  void set_particles(ParticlesTemp c);
  IMP_NO_DOXYGEN(void add_particles(const Particles &c) {
      add_particles(static_cast<const ParticlesTemp&>(c));
    })
  IMP_NO_DOXYGEN(void set_particles(const Particles &c) {
      set_particles(static_cast<ParticlesTemp>(c));
    })
  void clear_particles();
  /**@}*/

  static ListSingletonContainer *create_untracked_container() {
    ListSingletonContainer *lsc = new ListSingletonContainer(false);
    return lsc;
  }
#if defined(IMP_DOXYGEN) || defined(SWIG)
  IMP_SINGLETON_CONTAINER(ListSingletonContainer, get_module_version_info());
#else
  IMP_LISTLIKE_SINGLETON_CONTAINER(ListSingletonContainer,
                                   get_module_version_info());
#endif
};


IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_LIST_SINGLETON_CONTAINER_H */
