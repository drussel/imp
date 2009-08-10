/**
 *  \file BoxSweepClosePairsFinder.h
 *  \brief Test all pairs of particles to find close pairs.
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 */

#ifndef IMPCORE_BOX_SWEEP_CLOSE_PAIRS_FINDER_H
#define IMPCORE_BOX_SWEEP_CLOSE_PAIRS_FINDER_H

#include "ClosePairsFinder.h"


#ifdef IMP_USE_CGAL
IMPCORE_BEGIN_NAMESPACE

//! Find all nearby pairs by sweeping the bounding boxes
/** This method is much faster than the quadratic one when
    there are are large sets of points.

    \note This method requires \ref CGAL "CGAL" to work.
    \see ClosePairsScoreState
    \ingroup CGAL
*/
class IMPCOREEXPORT BoxSweepClosePairsFinder : public ClosePairsFinder
{
 public:
  BoxSweepClosePairsFinder();
  ~BoxSweepClosePairsFinder();

  void add_close_pairs(SingletonContainer *pc,
                       ListPairContainer *out) const;

  void add_close_pairs(SingletonContainer *pca,
                       SingletonContainer *pcb,
                       ListPairContainer *out) const;


  void show(std::ostream &out= std::cout) const {
    out << "BoxSweepClosePairsFinder" << std::endl;
  }
  VersionInfo get_version_info() const {
    return internal::version_info;
  }
};

IMPCORE_END_NAMESPACE

#endif /* IMP_USE_CGAL */

#endif  /* IMPCORE_BOX_SWEEP_CLOSE_PAIRS_FINDER_H */
