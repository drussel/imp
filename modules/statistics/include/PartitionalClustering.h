/**
 *  \file PartitionalClustering.h
 *  \brief Compute a distance metric between two points
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */
#ifndef IMPSTATISTICS_PARTITIONAL_CLUSTERING_H
#define IMPSTATISTICS_PARTITIONAL_CLUSTERING_H

#include "statistics_config.h"
#include <IMP/macros.h>
#include <IMP/base_types.h>
#include <IMP/Object.h>

IMPSTATISTICS_BEGIN_NAMESPACE

//! The base class for clusterings of data sets
/** Data items are represented by an index. In a partitional clustering
    each item can only be contained on one cluster. The mapping between
    data items and the information needed by the clustering algorithm is
    maintained by a helper object such as Embedding.
*/
class IMPSTATISTICSEXPORT PartitionalClustering: public IMP::base::Object {
 public:
  PartitionalClustering(std::string name): Object(name){}
  virtual unsigned int get_number_of_clusters() const=0;
  /** Return the list of indexes in the ith cluster.
   */
  virtual const Ints& get_cluster(unsigned int cluster_index) const =0;
  /** Return the index of an item that "typifies" the ith cluster.
   */
  virtual int get_cluster_representative(unsigned int cluster_index) const=0;
  IMP_REF_COUNTED_NONTRIVIAL_DESTRUCTOR(PartitionalClustering);
};

/** Check that the clustering is a valid clustering of n elements. An
 exception is thrown if it is not, if the build is not a fast build.*/
IMPSTATISTICSEXPORT
void validate_partitional_clustering(PartitionalClustering*pc,
                                     unsigned int n);

IMPSTATISTICS_END_NAMESPACE
#endif /* IMPSTATISTICS_PARTITIONAL_CLUSTERING_H */
