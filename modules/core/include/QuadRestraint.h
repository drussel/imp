/**
 *  \file QuadRestraint.h
 *  \brief Apply a QuadScore to a Quad.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPCORE_QUAD_RESTRAINT_H
#define IMPCORE_QUAD_RESTRAINT_H

#include "core_config.h"

#include <IMP/Restraint.h>
#include <IMP/Pointer.h>
#include <IMP/QuadScore.h>
#include "internal/quad_helpers.h"

#include <iostream>

IMPCORE_BEGIN_NAMESPACE

//! Applies a QuadScore to a Quad.
/** This restraint stores a Quad.
    \see QuadRestraint
 */
class IMPCOREEXPORT QuadRestraint :
  public QuadScoreRestraint
{
  IMP::internal::OwnerPointer<QuadScore> ss_;
  ParticleQuad v_;
public:
  //! Create the restraint.
  /** This function takes the function to apply to the
      stored Quad and the Quad.
   */
  QuadRestraint(QuadScore *ss,
                     const ParticleQuad& vt,
                     std::string name="QuadRestraint %1%");

  QuadScore* get_score() const {
    return ss_;
  }
  ParticleQuad get_argument() const {
    return v_;
  }

  IMP_RESTRAINT(QuadRestraint);

  Restraints get_instant_decomposition() const;
};

IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_QUAD_RESTRAINT_H */
