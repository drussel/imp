/**
 *  \file TripletContainerStatistics.h    \brief A container for Triplets.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */

#ifndef IMPCONTAINER_TRIPLET_CONTAINER_STATISTICS_H
#define IMPCONTAINER_TRIPLET_CONTAINER_STATISTICS_H

#include "container_config.h"
#include <IMP/TripletContainer.h>
#include <IMP/ScoreState.h>
#include <IMP/score_state_macros.h>
#include <IMP/compatibility/set.h>

IMPCONTAINER_BEGIN_NAMESPACE

//! Track statistics on a TripletContainer
/** The current statistics are average and min/max occupancy. Other
    statistics can be added on request, but we probably want to
    restrict it to ones that are cheap to gather. */
class IMPCONTAINEREXPORT TripletContainerStatistics : public ScoreState
{
  Pointer<TripletContainer> container_;
  unsigned int total_;
  unsigned int checks_;
  unsigned int max_;
  unsigned int min_;
  bool track_unique_;
  IMP::compatibility::set<ParticleTriplet> unique_;
public:
  TripletContainerStatistics(TripletContainerInput c);
  void show_statistics(std::ostream &out) const;
  /** Keeping track of the number of unique entries seen is
      expensive, so it is not done by default.
  */
  void set_track_unique(bool tf);
  IMP_SCORE_STATE(TripletContainerStatistics);
};


IMPCONTAINER_END_NAMESPACE

#endif  /* IMPCONTAINER_TRIPLET_CONTAINER_STATISTICS_H */
