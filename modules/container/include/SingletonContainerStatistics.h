/**
 *  \file SingletonContainerStatistics.h    \brief A container for Singletons.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */

#ifndef IMPCONTAINER_SINGLETON_CONTAINER_STATISTICS_H
#define IMPCONTAINER_SINGLETON_CONTAINER_STATISTICS_H

#include "container_config.h"
#include <IMP/SingletonContainer.h>
#include <IMP/ScoreState.h>
#include <IMP/compatibility/set.h>

IMPCONTAINER_BEGIN_NAMESPACE

//! Track statistics on a SingletonContainer
/** The current statistics are average and min/max occupancy. Other
    statistics can be added on request, but we probably want to
    restrict it to ones that are cheap to gather. */
class IMPCONTAINEREXPORT SingletonContainerStatistics : public ScoreState
{
  Pointer<SingletonContainer> container_;
  unsigned int total_;
  unsigned int checks_;
  unsigned int max_;
  unsigned int min_;
  bool track_unique_;
  IMP::compatibility::set<Particle*> unique_;
public:
  SingletonContainerStatistics(SingletonContainerInput c);
  void show_statistics(std::ostream &out) const;
  /** Keeping track of the number of unique entries seen is
      expensive, so it is not done by default.
  */
  void set_track_unique(bool tf);
  IMP_SCORE_STATE(SingletonContainerStatistics);
};


IMPCONTAINER_END_NAMESPACE

#endif  /* IMPCONTAINER_SINGLETON_CONTAINER_STATISTICS_H */
