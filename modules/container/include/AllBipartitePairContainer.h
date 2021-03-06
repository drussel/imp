/**
 *  \file IMP/container/AllBipartitePairContainer.h
 *  \brief Return all pairs from a SingletonContainer
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2013 IMP Inventors. All rights reserved.
 */

#ifndef IMPCONTAINER_ALL_BIPARTITE_PAIR_CONTAINER_H
#define IMPCONTAINER_ALL_BIPARTITE_PAIR_CONTAINER_H

#include <IMP/container/container_config.h>

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
  IMP::base::OwnerPointer<SingletonContainer> a_, b_;
  friend class AllPairContainer;
public:
  template <class F>
    void apply_generic(F* f) const {
    validate_readable();
    ParticleIndexes ib= b_->get_indexes();
    IMP_FOREACH_SINGLETON_INDEX(a_, {
        for (unsigned int j=0; j < ib.size(); ++j) {
          f->apply_index(get_model(), ParticleIndexPair(_1, ib[j]));
        }
      }
      );
  }
  AllBipartitePairContainer(SingletonContainerAdaptor a,
                            SingletonContainerAdaptor b,
                            std::string name="AllBipartitePairContainer%1%");

  IMP_PAIR_CONTAINER(AllBipartitePairContainer);
};

IMP_OBJECTS(AllBipartitePairContainer,AllBipartitePairContainers);


IMPCONTAINER_END_NAMESPACE

#endif  /* IMPCONTAINER_ALL_BIPARTITE_PAIR_CONTAINER_H */
