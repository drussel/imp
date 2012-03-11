/**
 *  \file TripletsRestraint.h
 *  \brief Apply a TripletScore to each Triplet in a list.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPCONTAINER_TRIPLETS_RESTRAINT_H
#define IMPCONTAINER_TRIPLETS_RESTRAINT_H

#include "container_config.h"

#include <IMP/internal/InternalTripletsRestraint.h>

#include <iostream>

IMPCONTAINER_BEGIN_NAMESPACE

//! Applies a TripletScore to each Triplet in a list.
/** This restraint stores the used particles in a ParticleTripletsTemp.
    The container used can be set so that the list can be shared
    with other containers (or a nonbonded list can be used). By default
    a ListTripletContainer is used and the
    {add_, set_, clear_}particle_triplet{s} methodas can be used.

    Examples using various multiplicity containers:
    \pythonexample{restrain_in_sphere}
    \pythonexample{nonbonded_interactions}

    \see TripletRestraint
 */
class IMPCONTAINEREXPORT TripletsRestraint :
#if defined(SWIG) || defined(IMP_DOXYGEN)
public Restraint
#else
public IMP::internal::InternalTripletsRestraint
#endif
{
  typedef IMP::internal::InternalTripletsRestraint P;
public:

 //! Create the restraint with a shared container
  /** \param[in] ss The function to apply to each particle.
      \param[in] pc The container containing the stored particles. This
      container is not copied.
      \param[in] name The object name
   */
  TripletsRestraint(TripletScore *ss,
                      TripletContainer *pc,
                      std::string name="TripletsRestraint %1%");

#if defined(IMP_DOXYGEN) || defined(SWIG)
  IMP_RESTRAINT(TripletsRestraint);

  //! Get the container used to store Particles
  ParticleTripletsTemp get_arguments() const;

  TripletContainer* get_container() const;

  TripletScore* get_score() const;
#else
  IMP_OBJECT(TripletsRestraint);
#endif
};

IMP_OBJECTS(TripletsRestraint,TripletsRestraints);

IMPCONTAINER_END_NAMESPACE

#endif  /* IMPCONTAINER_TRIPLETS_RESTRAINT_H */
