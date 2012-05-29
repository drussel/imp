/**
 *  \file QuadsRestraint.h
 *  \brief Apply a QuadScore to each Quad in a list.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPCONTAINER_QUADS_RESTRAINT_H
#define IMPCONTAINER_QUADS_RESTRAINT_H

#include "container_config.h"

#include <IMP/internal/InternalQuadsRestraint.h>

#include <iostream>

IMPCONTAINER_BEGIN_NAMESPACE

//! Applies a QuadScore to each Quad in a list.
/** This restraint stores the used particles in a ParticleQuadsTemp.
    The container used can be set so that the list can be shared
    with other containers (or a nonbonded list can be used). By default
    a ListQuadContainer is used and the
    {add_, set_, clear_}particle_quad{s} methodas can be used.

    Examples using various multiplicity containers:
    \pythonexample{restrain_in_sphere}
    \pythonexample{nonbonded_interactions}

    \see IMP::core::QuadRestraint
 */
class IMPCONTAINEREXPORT QuadsRestraint :
#if defined(SWIG) || defined(IMP_DOXYGEN)
public Restraint
#else
public IMP::internal::InternalQuadsRestraint
#endif
{
  typedef IMP::internal::InternalQuadsRestraint P;
public:

 //! Create the restraint with a shared container
  /** \param[in] ss The function to apply to each particle.
      \param[in] pc The container containing the stored particles. This
      container is not copied.
      \param[in] name The object name
   */
  QuadsRestraint(QuadScore *ss,
                      QuadContainerInput pc,
                      std::string name="QuadsRestraint %1%");

#if defined(IMP_DOXYGEN) || defined(SWIG)
  IMP_RESTRAINT(QuadsRestraint);

  //! Get the container used to store Particles
  ParticleQuadsTemp get_arguments() const;

  QuadContainer* get_container() const;

  QuadScore* get_score() const;
#else
  IMP_OBJECT(QuadsRestraint);
#endif
};

IMP_OBJECTS(QuadsRestraint,QuadsRestraints);

IMPCONTAINER_END_NAMESPACE

#endif  /* IMPCONTAINER_QUADS_RESTRAINT_H */
