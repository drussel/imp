/**
 *  \file ClusteringEngine.h
 *  \brief Virtual anchor points clustering engine
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPSTATISTICS_CLUSTERING_ENGINE_H
#define IMPSTATISTICS_CLUSTERING_ENGINE_H

#include "DataPoints.h"
#include "statistics_config.h"
IMPSTATISTICS_BEGIN_NAMESPACE

class ClusteringEngine {
public:
  virtual ~ClusteringEngine(){}
  virtual bool is_part_of_cluster(int data_point_ind,int cluster_ind) const=0;
  // TODO: convert Array1DD to standard IMP base types?
  virtual Array1DD get_center(int center_ind) const=0;
  virtual int get_number_of_clusters() const=0;
};
IMPSTATISTICS_END_NAMESPACE
#endif /* IMPSTATISTICS_CLUSTERING_ENGINE_H */