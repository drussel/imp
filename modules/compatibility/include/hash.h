/**
 *  \file IMP/compatibility/hash.h
 *  \brief Make sure that we avoid errors in specialization of boost hash
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */

#ifndef IMPCOMPATIBILITY_INTERNAL_HASH_H
#define IMPCOMPATIBILITY_INTERNAL_HASH_H

#include <IMP/compatibility/compatibility_config.h>
#include "vector.h"

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmismatched-tags"
#endif
#include <boost/functional/hash.hpp>
#include <boost/functional/hash/hash.hpp>
#ifdef __clang__
#pragma clang diagnostic pop
#endif

// this specializes some hash methods
#include <boost/graph/adjacency_list.hpp>


#endif  /* IMPCOMPATIBILITY_INTERNAL_HASH_H */
