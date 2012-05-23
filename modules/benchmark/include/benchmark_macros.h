/**
 *  \file benchmark_macros.h
 *  \brief Various general useful macros for IMP.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPBENCHMARK_MACROS_H
#define IMPBENCHMARK_MACROS_H

#include "command_line.h"
#include "profile_macros.h"
#include <boost/timer.hpp>
#include <IMP/base/exception.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/date_time/posix_time/posix_time_duration.hpp>


//! Time the given command and assign the time of one iteration to the variable
/** The units for the time are in seconds. See also IMP_TIME_N */
#define IMP_TIME(block, timev)                   \
  {                                              \
  boost::timer imp_timer;                        \
    unsigned int imp_reps=0;                     \
    if (IMP::benchmark::profile_benchmark) {     \
      IMP_BENCHMARK_START_PROFILING;             \
    }                                            \
    try {                                        \
      do {                                       \
        block;                                   \
        ++imp_reps;                              \
      } while (imp_timer.elapsed() < 2.5);       \
    } catch (const IMP::base::Exception &e) {    \
      std::cerr<< "Caught exception "            \
               << e.what() << std::endl;         \
    }                                            \
    if (IMP::benchmark::profile_benchmark) {     \
       IMP_BENCHMARK_END_PROFILING;              \
    }                                            \
    timev= imp_timer.elapsed()/imp_reps;         \
  }

//! Time the given command and assign the time of one iteration to the variable
/** The units for the time are in seconds. See also IMP_TIME_N */
#define IMP_WALLTIME(block, timev)                                      \
  {                                                                     \
    using namespace boost::posix_time;                                  \
    ptime start=microsec_clock::local_time();                           \
    unsigned int imp_reps=0;                                            \
    if (IMP::benchmark::profile_benchmark) {                            \
      IMP_BENCHMARK_START_PROFILING;                                    \
    }                                                                   \
    try {                                                               \
      do {                                                              \
        block;                                                          \
        ++imp_reps;                                                     \
      } while (microsec_clock::local_time()-start < seconds(2.5));      \
    } catch (const IMP::base::Exception &e) {                           \
      std::cerr<< "Caught exception "                                   \
               << e.what() << std::endl;                                \
    }                                                                   \
    timev= (microsec_clock::local_time()-start)                         \
      .total_milliseconds()/1000.0                                      \
      /imp_reps;                                                        \
    if (IMP::benchmark::profile_benchmark) {                            \
      IMP_BENCHMARK_END_PROFILING;                                      \
    }                                                                   \
  }

//! Time the given command and assign the time of one iteration to the variable
/** The units for the time are in seconds. The bit of code is run
    exact N times. See also IMP_TIME */
#define IMP_TIME_N(block, timev, N)              \
  {                                              \
    boost::timer imp_timer;                      \
    if (IMP::benchmark::profile_benchmark) {     \
      IMP_BENCHMARK_START_PROFILING;             \
    }                                            \
    for (unsigned int i=0; i< (N); ++i) {        \
      try {                                      \
        block;                                   \
      } catch (const IMP::base::Exception &e) {  \
        std::cerr<< "Caught exception "          \
                 << e.what() << std::endl;       \
        break;                                   \
      }                                          \
    }                                            \
    if (IMP::benchmark::profile_benchmark) {     \
       IMP_BENCHMARK_END_PROFILING;              \
    }                                            \
    timev= imp_timer.elapsed()/(N);              \
  }


#endif  /* IMPBENCHMARK_MACROS_H */
