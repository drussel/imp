/**
 *  \file IMP/container/ListPairContainer.h
 *  \brief Store a list of ParticlePairsTemp
 *
 *  WARNING This file was generated from ListNAMEContainer.hpp
 *  in tools/maintenance/container_templates/container
 *  by tools/maintenance/make-container.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */

#ifndef IMPCONTAINER_LIST_PAIR_CONTAINER_H
#define IMPCONTAINER_LIST_PAIR_CONTAINER_H

#include <IMP/container/container_config.h>
#include <IMP/internal/InternalListPairContainer.h>

IMPCONTAINER_BEGIN_NAMESPACE

//! Store a list of ParticlePairsTemp
/** \note The indexes can change when particles are inserted
    as the list is maintained in sorted order.
 */
class IMPCONTAINEREXPORT ListPairContainer:
#if defined(IMP_DOXYGEN) || defined(SWIG)
public PairContainer
#else
public IMP::internal::InternalListPairContainer
#endif
{
  typedef IMP::internal::InternalListPairContainer P;
public:
  ListPairContainer(const ParticlePairsTemp &ps);

  //! construct and pass an initial set of Pairs
  ListPairContainer(const ParticlePairsTemp &ps,
                         std::string name);

  ListPairContainer(Model *m,
                         std::string name= "ListPairContainer %1%");
  ListPairContainer(Model *m, const char *name);

 /** @name Methods to control the contained objects

     This container stores a list of Pair objects. To manipulate
     the list use these methods.
  */
  /**@{*/
  void add_particle_pair(const ParticlePair& vt);
  void add_particle_pairs(const ParticlePairsTemp &c);
  void set_particle_pairs(ParticlePairsTemp c);
  void clear_particle_pairs();
  /**@}*/
#ifdef SWIG
  IMP_PAIR_CONTAINER(ListPairContainer);
#endif
};

IMP_OBJECTS(ListPairContainer,ListPairContainers);

IMPCONTAINER_END_NAMESPACE

#endif  /* IMPCONTAINER_LIST_PAIR_CONTAINER_H */
