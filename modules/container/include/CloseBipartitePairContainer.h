/**
 *  \file CloseBipartitePairContainer.h
 *  \brief Return all pairs from a SingletonContainer
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2012 IMP Inventors. Close rights reserved.
 */

#ifndef IMPCONTAINER_CLOSE_BIPARTITE_PAIR_CONTAINER_H
#define IMPCONTAINER_CLOSE_BIPARTITE_PAIR_CONTAINER_H

#include "container_config.h"
#include <IMP/core/ClosePairsFinder.h>
#include <IMP/core/internal/CoreCloseBipartitePairContainer.h>
#include <IMP/core/internal/MovedSingletonContainer.h>
#include <IMP/PairContainer.h>
#include <IMP/PairFilter.h>
#include <IMP/SingletonContainer.h>
#include <IMP/container/ListPairContainer.h>
#include <IMP/core/internal/pair_helpers.h>

IMPCONTAINER_BEGIN_NAMESPACE

/** \brief Return all close unordered pairs of particles taken from
    the SingletonContainer

    See ClosePairContainer for a more detailed description. This
    container lists all close pairs of particles where one particle is
    taken from each of the input sets.

    \note Any passed ClosePairsFinder is ignored.

    \usesconstraint
 */
class IMPCONTAINEREXPORT CloseBipartitePairContainer:
#if defined(IMP_DOXYGEN) || defined(SWIG)
public PairContainer
#else
public IMP::core::internal::CoreCloseBipartitePairContainer
#endif
{
  typedef IMP::core::internal::CoreCloseBipartitePairContainer P;
public:
  //! Get the individual particles from the passed SingletonContainer
  CloseBipartitePairContainer(SingletonContainer *a,
                              SingletonContainer *b,
                              double distance,
                              double slack=1);

  //! Get the individual particles from the passed SingletonContainer
  CloseBipartitePairContainer(SingletonContainer *a,
                              SingletonContainer *b,
                              double distance,
                              core::ClosePairsFinder *cpf,
                              double slack=1);

#if defined(IMP_DOXYGEN) || defined(SWIG)
  /** @name Methods to control the set of filters

     PairContainer objects can be used as filters to prevent
     the addition of pairs to the containeroutput list. Pairs
     which are contained in any container added to this list
     will be excluded from the close pairs list.
  */
  /**@{*/
  IMP_LIST_ACTION(public, PairFilter, PairFilters, pair_filter,
                  pair_filters,
                  PairFilter*, PairFilters, obj->set_was_used(true);,,);
   /**@}*/
  IMP_PAIR_CONTAINER(CloseBipartitePairContainer);
  bool get_is_up_to_date() const;
#endif
};

IMPCONTAINER_END_NAMESPACE

#endif  /* IMPCONTAINER_CLOSE_BIPARTITE_PAIR_CONTAINER_H */
