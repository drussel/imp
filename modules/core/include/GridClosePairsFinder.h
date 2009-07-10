/**
 *  \file GridClosePairsFinder.h
 *  \brief Use a hierarchy of grids to find close pairs.
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 */

#ifndef IMPCORE_GRID_CLOSE_PAIRS_FINDER_H
#define IMPCORE_GRID_CLOSE_PAIRS_FINDER_H

#include "ClosePairsFinder.h"

IMPCORE_BEGIN_NAMESPACE

//! Find all nearby pairs by testing all pairs
/**
   \see CloserPairsScoreState
 */
class IMPCOREEXPORT GridClosePairsFinder : public ClosePairsFinder
{
 public:
  /** */
  GridClosePairsFinder();
  ~GridClosePairsFinder();

  void add_close_pairs(SingletonContainer *pc,
                       ListPairContainer *out) const;

  void add_close_pairs(SingletonContainer *pca,
                       SingletonContainer *pcb,
                       ListPairContainer *out) const;


  void show(std::ostream &out= std::cout) const {
    out << "GridClosePairsFinder" << std::endl;
  }
  VersionInfo get_version_info() const {
    return internal::version_info;
  }
};

IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_GRID_CLOSE_PAIRS_FINDER_H */
