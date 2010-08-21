/**
 *  \file Constraint.h   \brief A base class for constraints.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMP_CONSTRAINT_H
#define IMP_CONSTRAINT_H

#include "ScoreState.h"

IMP_BEGIN_NAMESPACE

//! Implement a constraint on the Model.
/** The solution model can be restricted two ways, either by
    penalizing "bad" conformations (a restraint) or by forcing some
    set of attributes to a function of other attributes (a
    constraint). For example, rigid bodies consisting of a number of
    particles could be implemented either way.

    - As a restraint: the particles in the rigid body are each moved
      independently by the optimizer. The scoring function has a term
      for how far each particle diverges from its rigid position.

    - As a constraint: the optimizer only changes the position of the
      rigid body itself and the position of the particles in the body
      are computed from the position of the rigid body.

    In IMP, constraints are implemented as a type of
    ScoreState. Before evaluation, the constraint updates the
    attributes of some of the particles to ensure that the constraint
    is satisfied. Since this update creates implicit relationships
    between the particles, after the derivatives are computed, the
    constraint can move them around to make sure the derivatives of
    the optimizer parameters are correct.

    In general, constraints are associated with Decorator objects
    and created invisibly when needed.

    \note Constraint invariants will not necessarily hold if
    involved particles have been called and Model::evaluate()
    has not been called. For example, if you change a
    particle's coordinates, a IMP::core::Centroid of a set
    containing the particle will not be correct until the
    Model is evaluated.

    \implementationwithoutexample{Constraint, IMP_CONSTRAINT}
 */
class IMPEXPORT Constraint : public ScoreState
{
public:
  Constraint(std::string name="Constraint %1%");

  IMP_REF_COUNTED_DESTRUCTOR(Constraint);
};


IMP_OBJECTS(Constraint,Constraints);

IMP_END_NAMESPACE

#endif  /* IMP_CONSTRAINT_H */
