/**
 *  \file IMP/container/QuadsConstraint.h
 *  \brief Use a QuadModifier applied to a ParticleQuadsTemp to
 *  maintain an invariant
 *
 *  WARNING This file was generated from NAMEsConstraint.hpp
 *  in tools/maintenance/container_templates/container
 *  by tools/maintenance/make-container.
 *
 *  Copyright 2007-2013 IMP Inventors. All rights reserved.
 */

#ifndef IMPCONTAINER_QUADS_CONSTRAINT_H
#define IMPCONTAINER_QUADS_CONSTRAINT_H

#include <IMP/container/container_config.h>
#include <IMP/QuadContainer.h>
#include <IMP/QuadModifier.h>
#include <IMP/Constraint.h>
#include <IMP/score_state_macros.h>
#include <IMP/internal/ContainerConstraint.h>

IMP_BEGIN_NAMESPACE
// for swig
class QuadContainer;
class QuadModifier;
IMP_END_NAMESPACE

IMPCONTAINER_BEGIN_NAMESPACE
//! Apply a QuadFunction to a QuadContainer to maintain an invariant
/** The score state is passed up to two QuadModifiers, one to
    apply before evaluation and the other after. The one after
    should take a DerivativeAccumulator as its last argument for
    QuadModifier::apply() and will only be called if
    the score was computed with derivatives.

    An example showing a how to use such a score state to maintain a cover
    of the atoms of a protein by a sphere per residue.
    \verbinclude cover_particles.py

    \see core::QuadConstraint
 */
class IMPCONTAINEREXPORT QuadsConstraint :
#if defined(SWIG) || defined(IMP_DOXYGEN)
 public Constraint
#else
 public IMP::internal::ContainerConstraint<QuadModifier,
                                           QuadDerivativeModifier,
                                           QuadContainer>
#endif
{
  typedef IMP::internal::ContainerConstraint<QuadModifier,
                                           QuadDerivativeModifier,
      QuadContainer> P;
public:
  /** \param[in] c The Container to hold the elements to process
      \param[in] before The QuadModifier to apply to all elements
      before evaluate.
      \param[in] after The QuadModifier to apply to all elements
      after evaluate.
      \param[in] name The object name
   */
  QuadsConstraint(QuadModifier *before,
                       QuadDerivativeModifier *after,
                       QuadContainerAdaptor c,
                       std::string name="QuadConstraint %1%"):
      P(before, after, c, name)
      {}
#if defined(IMP_DOXYGEN) || defined(SWIG)
  IMP_CONSTRAINT(QuadsConstraint);
#endif
};

IMP_OBJECTS(QuadsConstraint,QuadsConstraints);


IMPCONTAINER_END_NAMESPACE

#endif  /* IMPCONTAINER_QUADS_CONSTRAINT_H */
