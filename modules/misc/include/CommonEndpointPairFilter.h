/**
 *  \file CommonEndpointPairFilter.h
 *  \brief A fake filter that returns true for any pair of bonds with
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */

#ifndef IMPMISC_COMMON_ENDPOINT_PAIR_FILTER_H
#define IMPMISC_COMMON_ENDPOINT_PAIR_FILTER_H

#include "misc_config.h"

#include <IMP/PairFilter.h>

IMPMISC_BEGIN_NAMESPACE

//! Return true for any pair of bonds sharing an endpoint
/** XXXXXX.
 */
class IMPMISCEXPORT CommonEndpointPairFilter: public PairFilter
{
public:
  CommonEndpointPairFilter();

  IMP_PAIR_FILTER(CommonEndpointPairFilter);
};


IMPMISC_END_NAMESPACE

#endif  /* IMPMISC_COMMON_ENDPOINT_PAIR_FILTER_H */
