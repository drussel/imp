/**
 *  \file AllPairContainer.h
 *  \brief Return all pairs from a SingletonContainer
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 */

#ifndef IMPCONTAINER_ALL_PAIR_CONTAINER_H
#define IMPCONTAINER_ALL_PAIR_CONTAINER_H

#include "container_config.h"

#include <IMP/PairContainer.h>
#include <IMP/SingletonContainer.h>
#include <IMP/container/ListPairContainer.h>

IMPCONTAINER_BEGIN_NAMESPACE

//! Return all unordered pairs of particles taken from the SingletonContainer
/** Here is an example using this container to restrain all particles in a set
    to be within a a certain distance of one another.
    \verbinclude restrain_diameter.py

    \note Sequential access is much more efficient than random access which is
    suicidally slow for now. Complain if you want fast(er) random access.
    We might listen.

    \usesconstraint
 */
class IMPCONTAINEREXPORT AllPairContainer : public PairContainer
{
  IMP::internal::OwnerPointer<SingletonContainer> c_;
  IMP_CONTAINER_DEPENDENCIES(AllPairContainer, ret.push_back(back_->c_););
  friend class AllBipartitePairContainer;
  AllPairContainer(SingletonContainer *c, bool);

#define IMP_AP_LOOP(body)                       \
  ParticleIndexes pis= c_->get_indexes();       \
  for (unsigned int i=0; i< pis.size(); ++i) {  \
    for (unsigned int j=0; j< i; ++j) {         \
      ParticleIndexPair item(pis[i], pis[j]);   \
      body;                                     \
    }                                           \
  }

  IMP_IMPLEMENT_PAIR_CONTAINER_OPERATIONS(AllPairContainer,
                                               IMP_AP_LOOP);

#undef IMP_AP_LOOP

public:
  //! Get the individual particles from the passed SingletonContainer
  AllPairContainer(SingletonContainer *c,
                   std::string name="AllPairContainer%1%");

  static AllPairContainer *create_untracked_container(SingletonContainer *c) {
    AllPairContainer *lsc = new AllPairContainer(c, false);
    return lsc;
  }
#ifndef IMP_DOXYGEN
  bool get_is_up_to_date() const {
    return c_->get_is_up_to_date();
  }
#endif
  IMP_PAIR_CONTAINER(AllPairContainer);
};

IMP_OBJECTS(AllPairContainer,AllPairContainers);

IMPCONTAINER_END_NAMESPACE

#endif  /* IMPCONTAINER_ALL_PAIR_CONTAINER_H */
