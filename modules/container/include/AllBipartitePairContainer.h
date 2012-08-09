/**
 *  \file IMP/container/AllBipartitePairContainer.h
 *  \brief Return all pairs from a SingletonContainer
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */

#ifndef IMPCONTAINER_ALL_BIPARTITE_PAIR_CONTAINER_H
#define IMPCONTAINER_ALL_BIPARTITE_PAIR_CONTAINER_H

#include "container_config.h"

#include <IMP/PairContainer.h>
#include <IMP/SingletonContainer.h>
#include <IMP/container/ListPairContainer.h>
#include <IMP/container/PairContainerSet.h>
#include <IMP/pair_macros.h>
#include <IMP/singleton_macros.h>

IMPCONTAINER_BEGIN_NAMESPACE

//! Return all bipartite pairs between two containers
/** \see AllPairContainer

    \usesconstraint
 */
class IMPCONTAINEREXPORT AllBipartitePairContainer : public PairContainer
{
  IMP::OwnerPointer<SingletonContainer> a_, b_;
  friend class AllPairContainer;
#define IMP_ABP_LOOP(body)                              \
  ParticleIndexes ib= b_->get_indexes();                \
  IMP_FOREACH_SINGLETON_INDEX(a_, {                     \
      for (unsigned int j=0; j < ib.size(); ++j) {      \
        ParticleIndexPair item(_1, ib[j]);              \
        body;                                           \
      }                                                 \
    }                                                   \
    );
  IMP_IMPLEMENT_PAIR_CONTAINER_OPERATIONS(AllBipartitePairContainer,
                                               IMP_ABP_LOOP);
#undef IMP_ABP_LOOP
public:
  AllBipartitePairContainer(SingletonContainerAdaptor a,
                            SingletonContainerAdaptor b,
                            std::string name="AllBipartitePairContainer%1%");

  IMP_PAIR_CONTAINER(AllBipartitePairContainer);
};

IMP_OBJECTS(AllBipartitePairContainer,AllBipartitePairContainers);


IMPCONTAINER_END_NAMESPACE

#endif  /* IMPCONTAINER_ALL_BIPARTITE_PAIR_CONTAINER_H */
