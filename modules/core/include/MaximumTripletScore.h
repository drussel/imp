/**
 *  \file MaximumTripletScore.h    \brief Define TripletScore.
 *
 *  This file is generated by a script (tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2010 Sali Lab. All rights reserved.
 */

#ifndef IMPCORE_MAXIMUM_TRIPLET_SCORE_H
#define IMPCORE_MAXIMUM_TRIPLET_SCORE_H

#include "config.h"
#include <IMP/TripletScore.h>

IMPCORE_BEGIN_NAMESPACE

//! Evaluate the maximum n triplet scores of the passed set of TripletScores
/** Each of the set of TripletScores is evaluated and the sum of the
    minimum n is returned.
*/
class IMPCOREEXPORT MaximumTripletScore : public TripletScore
{
  TripletScores scores_;
  unsigned int n_;
public:
  MaximumTripletScore(const TripletScoresTemp &scores,
                       unsigned int n=1,
                       std::string name="TripletScore %1%");
  IMP_TRIPLET_SCORE(MaximumTripletScore, get_module_version_info());
};

IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_MAXIMUM_TRIPLET_SCORE_H */
