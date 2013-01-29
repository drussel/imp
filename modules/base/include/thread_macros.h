/**
 *  \file IMP/base/thread_macros.h
 *  \brief Control for OpenMP
 *
 *  Copyright 2007-2013 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPBASE_THREAD_MACROS_H
#define IMPBASE_THREAD_MACROS_H

#include "threads.h"
#include "utility_macros.h"
#include "log_macros.h"
#ifdef _OPENMP
#include <IMP/base/CreateLogContext.h>
#include <omp.h>
#endif

#if defined(IMP_DOXYGEN) || !defined(_OPENMP)

/** Start a new OpenMP task with the next block passing the
    list of passed variables.*/
#define IMP_TASK(privatev, action, name) action

/** Start a new OpenMP task with the next block passing the
    list of passed variables.*/
#define IMP_TASK_SHARED(privatev, sharedv, action, name) action

/** Start a parallel section if one is not already started.
 */
#define IMP_THREADS(variables, action) action

#else

#define IMP_TASK(privatev, action, name)                                \
  if (IMP::base::get_number_of_threads() > 1) {                         \
    IMP_PRAGMA(omp task default(none) firstprivate privatev             \
               if (omp_in_parallel())                                   \
                                                                        \
               )                                                        \
      {                                                                 \
        IMP::base::CreateLogContext task_context(name);                 \
        action;                                                         \
      }                                                                 \
  } else {                                                              \
    action;                                                             \
  }

#define IMP_TASK_SHARED(privatev, sharedv, action, name)                \
  if (IMP::base::get_number_of_threads() > 1) {                         \
    IMP_PRAGMA(omp task default(none) firstprivate privatev             \
               shared sharedv                                           \
               if (omp_in_parallel()))                                  \
    {                                                                   \
      IMP::base::CreateLogContext task_context(name);                   \
      action;                                                           \
    }                                                                   \
  } else {                                                              \
    action;                                                             \
  }


#define IMP_THREADS(variables, action)                                  \
  if (IMP::base::get_number_of_threads() > 1) {                         \
    IMP_PRAGMA(omp parallel shared variables                            \
               num_threads(IMP::base::get_number_of_threads()))         \
      {                                                                 \
        IMP_PRAGMA(omp single)                                          \
          {                                                             \
            IMP::base::CreateLogContext parallel_context("parallel");   \
            action;                                                     \
          }                                                             \
      }                                                                 \
  } else {                                                              \
    action;                                                             \
  }
#endif

#endif /* IMPBASE_THREAD_MACROS_H */
