/**
 *  \file QuadsRestraint.h
 *  \brief Apply a QuadScore to each ParticleQuad in a list.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2010 Sali Lab. All rights reserved.
 *
 */

#ifndef IMPCONTAINER_QUADS_RESTRAINT_H
#define IMPCONTAINER_QUADS_RESTRAINT_H

#include "container_config.h"

#include <IMP/core/internal/CoreQuadsRestraint.h>

#include <iostream>

IMPCONTAINER_BEGIN_NAMESPACE

//! Applies a QuadScore to each ParticleQuad in a list.
/** This restraint stores the used particles in a ParticleQuads.
    The container used can be set so that the list can be shared
    with other containers (or a nonbonded list can be used). By default
    a ListQuadContainer is used and the
    {add_, set_, clear_}particle_quad{s} methodas can be used.

    Examples using various multiplicity containers:
    \htmlinclude restrain_in_sphere.py.html
    \htmlinclude nonbonded_interactions.py.html

    \see QuadRestraint
 */
class IMPCONTAINEREXPORT QuadsRestraint :
#if defined(SWIG) || defined(IMP_DOXYGEN)
public Restraint
#else
public core::internal::CoreQuadsRestraint
#endif
{
  typedef core::internal::CoreQuadsRestraint P;
  IMP::internal::OwnerPointer<QuadScore> ss_;
  IMP::internal::OwnerPointer<QuadContainer> pc_;
  mutable double score_;
public:

 //! Create the restraint with a shared container
  /** \param[in] ss The function to apply to each particle.
      \param[in] pc The container containing the stored particles. This
      container is not copied.
      \param[in] name The object name
   */
  QuadsRestraint(QuadScore *ss,
                      QuadContainer *pc,
                      std::string name="QuadsRestraint %1%");

#if defined(IMP_DOXYGEN) || defined(SWIG)
  IMP_INCREMENTAL_RESTRAINT(QuadsRestraint);

  //! Get the container used to store Particles
  QuadContainer* get_quad_container() {
    return pc_;
  }

  QuadScore* get_quad_score() const {
    return ss_;
  }
#else
  IMP_OBJECT(QuadsRestraint);
#endif
};

IMPCONTAINER_END_NAMESPACE

#endif  /* IMPCONTAINER_QUADS_RESTRAINT_H */
