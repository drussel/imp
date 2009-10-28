/**
 *  \file CloseBipartitePairContainer.h
 *  \brief Return all pairs from a SingletonContainer
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-9 Sali Lab. Close rights reserved.
 */

#ifndef IMPCORE_CLOSE_BIPARTITE_PAIR_CONTAINER_H
#define IMPCORE_CLOSE_BIPARTITE_PAIR_CONTAINER_H

#include "config.h"
#include "ClosePairsFinder.h"
#include "internal/MovedSingletonContainer.h"
#include <IMP/PairContainer.h>
#include <IMP/PairFilter.h>
#include <IMP/SingletonContainer.h>
#include <IMP/core/ListPairContainer.h>
#include <IMP/core/PairContainerSet.h>

IMPCORE_BEGIN_NAMESPACE

/** \brief Return all close unordered pairs of particles taken from
    the SingletonContainer

    See ClosePairContainer for a more detailed description. This
    container lists all close pairs of particles where one particle is
    taken from each of the input sets.
 */
class IMPCOREEXPORT CloseBipartitePairContainer : public PairContainer
{
  IMP::internal::OwnerPointer<SingletonContainer> a_, b_;
  IMP::internal::OwnerPointer<ClosePairsFinder> cpf_;
  IMP::internal::OwnerPointer<internal::MovedSingletonContainer>
    moveda_, movedb_;
  bool first_call_;
  double distance_, slack_;
  ParticlePairsTemp data_;
  IMP_ACTIVE_CONTAINER_DECL(CloseBipartitePairContainer);
  void initialize(SingletonContainer *a,
                  SingletonContainer *b, double distance,
                  double slack, Model *m, ClosePairsFinder *cpf);
public:
  //! Get the individual particles from the passed SingletonContainer
  CloseBipartitePairContainer(SingletonContainer *a,
                                   SingletonContainer *b,
                                   double distance,
                          double slack=1);
  //! If the container is empty, you can pass the model explicitly
  CloseBipartitePairContainer(SingletonContainer *a,
                                   SingletonContainer *b,
                                   Model *m, double distance,
                          double slack=1);

  //! Get the individual particles from the passed SingletonContainer
  CloseBipartitePairContainer(SingletonContainer *a,
                                   SingletonContainer *b,
                                   double distance,
                                   ClosePairsFinder *cpf,
                                   double slack=1);
  //! If the container is empty, you can pass the model explicitly
  CloseBipartitePairContainer(SingletonContainer *a,
                                   SingletonContainer *b,
                                   Model *m, double distance,
                                   ClosePairsFinder *cpf,
                                   double slack=1);

  /** @name Methods to control the set of filters

     PairContainer objects can be used as filters to prevent
     the addition of pairs to the containeroutput list. Pairs
     which are contained in any container added to this list
     will be excluded from the close pairs list.
  */
  /**@{*/
  IMP_LIST(public, PairFilter, pair_filter,
           PairFilter*, PairFilters);
   /**@}*/


  IMP_PAIR_CONTAINER(CloseBipartitePairContainer, get_module_version_info());
};


IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_CLOSE_BIPARTITE_PAIR_CONTAINER_H */
