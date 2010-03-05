/**
 *  \file example/ExampleRestraint.h
 *  \brief A restraint on a list of particle pairs.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPEXAMPLE_EXAMPLE_RESTRAINT_H
#define IMPEXAMPLE_EXAMPLE_RESTRAINT_H

#include "example_config.h"
#include <IMP/SingletonScore.h>
#include <IMP/Restraint.h>
#include <IMP/PairContainer.h>
#include <IMP/PairScore.h>

IMPEXAMPLE_BEGIN_NAMESPACE

//! Apply a PairScore to a list of particle pairs
/** This restraint could be used, in conjunction with a
    ClosePairsScoreState and a SphereDistancePairScore,
    to prevent particles from interpenetrating.

    \note Be sure to check out the swig wrapper file and how it
    wraps this class.

    The source code is as follows:
    \include ExampleRestraint.h
    \include ExampleRestraint.cpp
*/
class IMPEXAMPLEEXPORT ExampleRestraint : public Restraint
{
  /** IMP::Objects should be stored using Pointer objects
      to make sure that they are reference counted properly.
  */
  Pointer<PairContainer> pc_;
  Pointer<PairScore> f_;
public:
  //! Create the restraint.
  /** Restraints should store the particles they are to act on,
      preferably in a Singleton or PairContainer as appropriate.
      They should also take a score function or a UnaryFunction
      allowing the form of the scoring function to be changed.
   */
  ExampleRestraint(PairScore* score_func,
                   PairContainer *pc);

  /** This macro declares the basic needed methods: evaluate and show
   */
  IMP_RESTRAINT(ExampleRestraint);
};

IMPEXAMPLE_END_NAMESPACE

#endif  /* IMPEXAMPLE_EXAMPLE_RESTRAINT_H */
