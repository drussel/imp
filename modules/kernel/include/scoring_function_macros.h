/**
 *  \file IMP/scoring_function_macros.h
 *  \brief Various general useful macros for IMP.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPKERNEL_SCORING_FUNCTION_MACROS_H
#define IMPKERNEL_SCORING_FUNCTION_MACROS_H
#include "kernel_config.h"
#include <IMP/base/doxygen_macros.h>
#include <IMP/base/object_macros.h>
#include "ScoringFunction.h"


/** Declare the methods needed for an IMP::ScoringFunction object. It
    declares the following methods that you need to implement:
    - IMP::ScoringFunction::do_add_score_and_derivatives()
    - IMP::ScoringFunction::create_restraints()
    - IMP::ScoringFunction::get_required_score_states()
    in addition to the IMP_OBJECT() methods.*/
#define IMP_SCORING_FUNCTION(Name)                                      \
  IMP_IMPLEMENT(void                                                    \
                do_add_score_and_derivatives(IMP::ScoreAccumulator sa,  \
                                             const ScoreStatesTemp &ss)); \
  IMP_IMPLEMENT(Restraints create_restraints() const);                  \
  IMP_IMPLEMENT(ScoreStatesTemp                                         \
                get_required_score_states(const DependencyGraph &dg,    \
                                          const DependencyGraphVertexIndex&i) \
                const);                                                 \
  IMP_OBJECT(Name)
//! @}


#endif  /* IMPKERNEL_SCORING_FUNCTION_MACROS_H */
