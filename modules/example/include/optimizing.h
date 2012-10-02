/**
 *  \file IMP/example/optimizing.h
 *  \brief A simple unary function.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPEXAMPLE_OPTIMIZING_H
#define IMPEXAMPLE_OPTIMIZING_H

#include "example_config.h"
#include <IMP/log.h>
#include <IMP/base_types.h>
#include <IMP/base/enums.h>
#include <IMP/core/Mover.h>

IMPEXAMPLE_BEGIN_NAMESPACE
/** Create a serial mover from a list of core::XYZR particles.
 */
IMPEXAMPLEEXPORT core::Mover* create_serial_mover(const ParticlesTemp &ps);


/** Take a set of core::XYZR particles and relax them relative to a set of
    restraints. Excluded volume is handle separately, so don't include it
in the passed list of restraints. */
IMPEXAMPLEEXPORT void optimize_balls(const ParticlesTemp &ps,
                           const RestraintsTemp &rs=RestraintsTemp(),
                           const PairPredicates &excluded=PairPredicates(),
                           const OptimizerStates &opt_states=OptimizerStates(),
                                     base::LogLevel ll=DEFAULT);

IMPEXAMPLE_END_NAMESPACE

#endif  /* IMPEXAMPLE_OPTIMIZING_H */
