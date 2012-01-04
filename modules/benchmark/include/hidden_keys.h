/**
 *  \file benchmark/hidden_keys.h
 *  \brief Various utilities for benchmarking
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPBENCHMARK_HIDDEN_KEYS_H
#define IMPBENCHMARK_HIDDEN_KEYS_H

#include "benchmark_config.h"
#include <IMP/base_types.h>
#include <string>

IMPBENCHMARK_BEGIN_NAMESPACE
#ifndef IMP_DOXYGEN
//! Cartesian coordinate keys which shouldn't be inlined.
IMPBENCHMARKEXPORT extern FloatKey hidden_keys[];
#endif
IMPBENCHMARK_END_NAMESPACE

#endif  /* IMPBENCHMARK_HIDDEN_KEYS_H */
