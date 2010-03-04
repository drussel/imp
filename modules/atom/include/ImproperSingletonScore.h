/**
 *  \file atom/ImproperSingletonScore.h
 *  \brief A score on the deviation of an improper angle from ideality.
 *
 *  Copyright 2007-2010 Sali Lab. All rights reserved.
 */

#ifndef IMPATOM_IMPROPER_SINGLETON_SCORE_H
#define IMPATOM_IMPROPER_SINGLETON_SCORE_H

#include "atom_config.h"
#include "bond_decorators.h"
#include <IMP/SingletonScore.h>
#include <IMP/UnaryFunction.h>
#include <IMP/Pointer.h>

IMPATOM_BEGIN_NAMESPACE

//! Score the improper dihedral based on a UnaryFunction,
/** This scores the improper dihedral using information stored in its
    Dihedral decorator. The score is based on the difference between the
    stored ideal improper angle and the actual angle and scaled by the
    stiffness. That is stiffness * (improper_angle-ideal_value). The
    difference is in radians between -pi and +pi; it is the shortest
    distance from one angle to the other.

    \note The multiplicity of the the Dihedral is not used.

    \see CHARMMTopology::add_impropers(), Dihedral.
 */
class IMPATOMEXPORT ImproperSingletonScore : public SingletonScore
{
  IMP::internal::OwnerPointer<UnaryFunction> f_;
public:
  //! Use f to penalize deviations in angle
  ImproperSingletonScore(UnaryFunction *f);
  IMP_SINGLETON_SCORE(ImproperSingletonScore);
};

IMPATOM_END_NAMESPACE

#endif  /* IMPATOM_IMPROPER_SINGLETON_SCORE_H */
