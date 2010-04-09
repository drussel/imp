/**
 *  \file ListQuadContainer.h    \brief Store a list of ParticleQuads
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 */

#ifndef IMPCONTAINER_LIST_QUAD_CONTAINER_H
#define IMPCONTAINER_LIST_QUAD_CONTAINER_H

#include "container_config.h"
#include <IMP/core/internal/CoreListQuadContainer.h>

IMPCONTAINER_BEGIN_NAMESPACE

//! Store a list of ParticleQuads
/** \note The indexes can change when particles are inserted
    as the list is maintained in sorted order.
 */
class IMPCONTAINEREXPORT ListQuadContainer:
#if defined(IMP_DOXYGEN) || defined(SWIG)
public QuadContainer
#else
public core::internal::CoreListQuadContainer
#endif
{
  typedef core::internal::CoreListQuadContainer P;
  // for the change versions
  ListQuadContainer();
public:
  //! construct and pass an initial set of particle_quads
  ListQuadContainer(const ParticleQuads &ps,
                         std::string name= "ListQuadContainer %1%");

  ListQuadContainer(Model *m,
                         std::string name= "ListQuadContainer %1%");
  ListQuadContainer(Model *m, const char *name);

#if defined(IMP_DOXYGEN) || defined(SWIG)
 /** @name Methods to control the contained objects

     This container stores a list of ParticleQuad objects. To manipulate
     the list use these methods.
  */
  /**@{*/
  void add_particle_quad(const ParticleQuad& vt);
  void add_particle_quads(const ParticleQuadsTemp &c);
  void set_particle_quads(ParticleQuadsTemp c);
  IMP_NO_DOXYGEN(void add_particle_quads(const ParticleQuads &c) {
      add_particle_quads(static_cast<const ParticleQuadsTemp&>(c));
    })
  IMP_NO_DOXYGEN(void set_particle_quads(const ParticleQuads &c) {
      set_particle_quads(static_cast<ParticleQuadsTemp>(c));
    })
  void clear_particle_quads();
  /**@}*/
  IMP_QUAD_CONTAINER(ListQuadContainer);
#else
  static ListQuadContainer *create_untracked_container() {
    ListQuadContainer *lsc = new ListQuadContainer(false);
    return lsc;
  }
  IMP_OBJECT(ListQuadContainer);
#endif
};

IMP_OBJECTS(ListQuadContainer,ListQuadContainers);

IMPCONTAINER_END_NAMESPACE

#endif  /* IMPCONTAINER_LIST_QUAD_CONTAINER_H */
