/**
 *  \file RestraintGraph.h
 *  \brief creates a MRF from a set of particles and restraints
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPDOMINO2_INTERNAL_LOOPY_INFERENCE_H
#define IMPDOMINO2_INTERNAL_LOOPY_INFERENCE_H

#include "../domino2_config.h"
#include "../subset_filters.h"

IMPDOMINO2_BEGIN_NAMESPACE
class SubsetEvaluatorTable;
class SubsetStatesTable;
IMPDOMINO2_END_NAMESPACE

IMPDOMINO2_BEGIN_INTERNAL_NAMESPACE

IMPDOMINO2EXPORT
SubsetStatesList loopy_get_best_conformations(const SubsetGraph &sg,
                                          const Subset& all_particles,
                                          const SubsetFilterTables &filters,
                                              const SubsetStatesTable *sst);


IMPDOMINO2_END_INTERNAL_NAMESPACE

#endif  /* IMPDOMINO2_INTERNAL_LOOPY_INFERENCE_H */
