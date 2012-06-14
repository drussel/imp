/**
 *  \file PartitionalClusteringWithCenter.h
 *  \brief Cluster sets of points.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */
#ifndef IMPSTATISTICS_PARTITIONAL_CLUSTERING_WITH_CENTER_H
#define IMPSTATISTICS_PARTITIONAL_CLUSTERING_WITH_CENTER_H

#include "statistics_config.h"
#include "PartitionalClustering.h"
#include "partitional_clustering_macros.h"

IMPSTATISTICS_BEGIN_NAMESPACE
/** In addition to the information in the Clustering base class,
    PartitionalClusteringWithCenter stores a cluster center for
    each cluster.
    The cluster center is a point in the space defined by the
    embedding.

    The representative for each cluster is the member whose
    location in the embedding is closest to the cluster center.
*/
class IMPSTATISTICSEXPORT PartitionalClusteringWithCenter:
  public PartitionalClustering {
  IMP::base::Vector<Ints> clusters_;
  Ints reps_;
  algebra::VectorKDs centers_;
public:
#if !defined(SWIG) && !defined(IMP_DOXYGEN)
  template <int D>
    PartitionalClusteringWithCenter(const
              IMP::base::Vector<Ints> &clusters,
 const IMP::base::Vector<algebra::VectorD<D> > &centers,
                     const Ints &reps):
    PartitionalClustering("k-means"),
    clusters_(clusters),
    reps_(reps),
    centers_(reps.size()){
    for (unsigned int i=0; i< centers_.size(); ++i) {
        centers_[i]= algebra::VectorKD(centers[i].coordinates_begin(),
                                       centers[i].coordinates_end());
      }
  }
#endif
  const algebra::VectorKD& get_cluster_center(unsigned int i) const {
    return centers_[i];
  }
  IMP_PARTITIONAL_CLUSTERING(PartitionalClusteringWithCenter);
};

IMPSTATISTICS_END_NAMESPACE

#endif /* IMPSTATISTICS_PARTITIONAL_CLUSTERING_WITH_CENTER_H */
