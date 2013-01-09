/**
 *  \file IMP/container/PairContainerStatistics.h
 *  \brief A container for Pairs.
 *
 *  WARNING This file was generated from NAMEContainerStatistics.hpp
 *  in tools/maintenance/container_templates/container
 *  by tools/maintenance/make-container.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */

#ifndef IMPCONTAINER_PAIR_CONTAINER_STATISTICS_H
#define IMPCONTAINER_PAIR_CONTAINER_STATISTICS_H

#include <IMP/container/container_config.h>
#include <IMP/PairContainer.h>
#include <IMP/ScoreState.h>
#include <IMP/score_state_macros.h>
#include <IMP/compatibility/set.h>

IMPCONTAINER_BEGIN_NAMESPACE

//! Track statistics on a PairContainer
/** The current statistics are average and min/max occupancy. Other
    statistics can be added on request, but we probably want to
    restrict it to ones that are cheap to gather. */
class IMPCONTAINEREXPORT PairContainerStatistics : public ScoreState
{
  base::Pointer<PairContainer> container_;
  unsigned int total_;
  unsigned int checks_;
  unsigned int max_;
  unsigned int min_;
  bool track_unique_;
  IMP::compatibility::set<ParticlePair> unique_;
public:
  PairContainerStatistics(PairContainerAdaptor c);
  void show_statistics(std::ostream &out) const;
  /** Keeping track of the number of unique entries seen is
      expensive, so it is not done by default.
  */
  void set_track_unique(bool tf);
  IMP_SCORE_STATE(PairContainerStatistics);
};


IMPCONTAINER_END_NAMESPACE

#endif  /* IMPCONTAINER_PAIR_CONTAINER_STATISTICS_H */
