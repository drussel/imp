/**
 *  \file IMP/statistics/RecursivePartitionalClusteringEmbedding.h
 *  \brief Cluster sets of points.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */
#ifndef IMPSTATISTICS_RECURSIVE_PARTITIONAL_CLUSTERING_EMBEDDING_H
#define IMPSTATISTICS_RECURSIVE_PARTITIONAL_CLUSTERING_EMBEDDING_H

#include "statistics_config.h"
#include "PartitionalClustering.h"
#include "Embedding.h"
#include "embedding_macros.h"

IMPSTATISTICS_BEGIN_NAMESPACE


/** Represent an embedding for clustering data that has already been clustered
    once. To use it, cluster things once, create one of these with the embedding
    you want (created with the original data). When you pass this embedding to
    the clustering algorithm, it will cluster the centers. You can extract the
    clustering of the original elements using create_full_clustering().
*/
class IMPSTATISTICSEXPORT RecursivePartitionalClusteringEmbedding:
  public Embedding {
  IMP::OwnerPointer<Embedding> metric_;
  IMP::OwnerPointer<PartitionalClustering> clustering_;
 public:
  RecursivePartitionalClusteringEmbedding(Embedding *metric,
                                          PartitionalClustering *clustering);
  PartitionalClustering*
    create_full_clustering(PartitionalClustering *center_cluster);
  IMP_EMBEDDING(RecursivePartitionalClusteringEmbedding);
};


IMPSTATISTICS_END_NAMESPACE

#endif /* IMPSTATISTICS_RECURSIVE_PARTITIONAL_CLUSTERING_EMBEDDING_H */
