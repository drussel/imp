/**
 *  \file QuadContainerStatistics.h    \brief A container for Quads.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 */

#ifndef IMPCONTAINER_QUAD_CONTAINER_STATISTICS_H
#define IMPCONTAINER_QUAD_CONTAINER_STATISTICS_H

#include "container_config.h"
#include <IMP/QuadContainer.h>
#include <IMP/ScoreState.h>
#include <IMP/compatibility/set.h>

IMPCONTAINER_BEGIN_NAMESPACE

//! Track statistics on a QuadContainer
/** The current statistics are average and min/max occupancy. Other
    statistics can be added on request, but we probably want to
    restrict it to ones that are cheap to gather. */
class IMPCONTAINEREXPORT QuadContainerStatistics : public ScoreState
{
  Pointer<QuadContainer> container_;
  unsigned int total_;
  unsigned int checks_;
  unsigned int max_;
  unsigned int min_;
  bool track_unique_;
  IMP::compatibility::set<ParticleQuad> unique_;
public:
  QuadContainerStatistics(QuadContainer *c);
  void show_statistics(std::ostream &out) const;
  /** Keeping track of the number of unique entries seen is
      expensive, so it is not done by default.
  */
  void set_track_unique(bool tf);
  IMP_SCORE_STATE(QuadContainerStatistics);
};


IMPCONTAINER_END_NAMESPACE

#endif  /* IMPCONTAINER_QUAD_CONTAINER_STATISTICS_H */
