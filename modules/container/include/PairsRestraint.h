/**
 *  \file PairsRestraint.h
 *  \brief Apply a PairScore to each ParticlePair in a list.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPCONTAINER_PAIRS_RESTRAINT_H
#define IMPCONTAINER_PAIRS_RESTRAINT_H

#include "container_config.h"

#include <IMP/core/internal/CorePairsRestraint.h>

#include <iostream>

IMPCONTAINER_BEGIN_NAMESPACE

//! Applies a PairScore to each ParticlePair in a list.
/** This restraint stores the used particles in a ParticlePairs.
    The container used can be set so that the list can be shared
    with other containers (or a nonbonded list can be used). By default
    a ListPairContainer is used and the
    {add_, set_, clear_}particle_pair{s} methodas can be used.

    Examples using various multiplicity containers:
    \htmlinclude restrain_in_sphere.py.html
    \htmlinclude nonbonded_interactions.py.html

    \see PairRestraint
 */
class IMPCONTAINEREXPORT PairsRestraint :
#if defined(SWIG) || defined(IMP_DOXYGEN)
public Restraint
#else
public core::internal::CorePairsRestraint
#endif
{
  typedef core::internal::CorePairsRestraint P;
  IMP::internal::OwnerPointer<PairScore> ss_;
  IMP::internal::OwnerPointer<PairContainer> pc_;
  mutable double score_;
public:

 //! Create the restraint with a shared container
  /** \param[in] ss The function to apply to each particle.
      \param[in] pc The container containing the stored particles. This
      container is not copied.
      \param[in] name The object name
   */
  PairsRestraint(PairScore *ss,
                      PairContainer *pc,
                      std::string name="PairsRestraint %1%");

#if defined(IMP_DOXYGEN) || defined(SWIG)
  IMP_INCREMENTAL_RESTRAINT(PairsRestraint);

  //! Get the container used to store Particles
  PairContainer* get_pair_container() {
    return pc_;
  }

  PairScore* get_pair_score() const {
    return ss_;
  }
#else
  IMP_OBJECT(PairsRestraint);
#endif
};

IMP_OBJECTS(PairsRestraint);

IMPCONTAINER_END_NAMESPACE

#endif  /* IMPCONTAINER_PAIRS_RESTRAINT_H */
