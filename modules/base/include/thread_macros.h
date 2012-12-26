/**
 *  \file IMP/base/thread_macros.h
 *  \brief Control for OpenMP
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPBASE_THREAD_MACROS_H
#define IMPBASE_THREAD_MACROS_H

#include "threads.h"
#include "utility_macros.h"
#ifdef _OPENMP
#include <omp.h>
#endif

IMPBASE_BEGIN_NAMESPACE

#if defined(IMP_DOXYGEN)

/** Start a new OpenMP task with the next block passing the
    list of passed variables.*/
#define IMP_TASK(privatev, action)

/** Start a new OpenMP task with the next block passing the
    list of passed variables.*/
#define IMP_TASK_SHARED(privatev, sharedv, action)

/** Start a parallel section if one is not already started.
 */
#define IMP_THREADS(variables, action)

#elif !defined(IMP_USE_PRAGMA) || !defined(_OPENMP)
#define IMP_TASK(privatev, action)                                      \
  action

#define IMP_TASK_SHARED(privatev, sharedv, action)                      \
  action


#define IMP_THREADS(variables, action)                                  \
  action

#else

#define IMP_TASK(privatev, action)                                      \
  if (IMP::base::get_number_of_threads() > 1) {                         \
    IMP_PRAGMA(omp task default(none) firstprivate privatev             \
               if (omp_in_parallel())                                   \
                                                                        \
               )                                                        \
      {                                                                 \
        action;                                                         \
      }                                                                 \
  } else {                                                              \
    action;                                                             \
  }

#define IMP_TASK_SHARED(privatev, sharedv, action)                      \
  if (IMP::base::get_number_of_threads() > 1) {                         \
    IMP_PRAGMA(omp task default(none) firstprivate privatev             \
               shared sharedv                                           \
               if (omp_in_parallel())                                   \
                                                                        \
             )                                                          \
    {                                                                   \
      action;                                                           \
    }                                                                   \
  } else {                                                              \
    action;                                                             \
  }


#define IMP_THREADS(variables, action)                                  \
  if (IMP::base::get_number_of_threads() > 1) {                         \
    IMP_PRAGMA(omp parallel firstprivate(sf)                            \
               num_threads(IMP::base::get_number_of_threads())          \
               )                                                        \
      {                                                                 \
        IMP_PRAGMA(omp single)                                          \
          {                                                             \
            action;                                                     \
          }                                                             \
      }                                                                 \
  } else {                                                              \
    action;                                                             \
  }
#endif

IMPBASE_END_NAMESPACE

#endif /* IMPBASE_THREAD_MACROS_H */
