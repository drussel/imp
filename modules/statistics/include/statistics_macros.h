/**
 *  \file statistics_macros.h    \brief Various important macros
 *                           for implementing decorators.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPSTATISTICS_MACROS_H
#define IMPSTATISTICS_MACROS_H

#include <IMP/macros.h>


//! Define things needed for a Embedding
/** In addition to what is done by IMP_OBJECT() it declares
    - IMP::statistics::Embedding::get_embedding()
    - IMP::statistics::Embedding::get_number_of_items()
*/
#define IMP_EMBEDDING(Name)                                        \
  algebra::VectorKD get_point(unsigned int i) const;               \
  unsigned int get_number_of_items() const;                        \
  IMP_OBJECT(Name)

//! Define things needed for a Embedding
/** In addition to what is done by IMP_OBJECT() it declares
    - IMP::statistics::Distance::get_embedding()
*/
#define IMP_DISTANCE(Name)                                         \
  distance get_distance(unsigned int i,                            \
                        unsigned int j) const;                     \
  unsigned int get_number_of_items() const;                        \
  IMP_OBJECT(Name)


//! Define the methods needed by Clustering
/** In addition to what is defined/declared by IMP_OBJECT() it declares
    - IMP::statistics::Clustering::get_number_of_clusters()
    - IMP::statistics::Clustering::get_cluster()
    - IMP::statistics::Clustering::get_cluster_representative()
*/
#define IMP_CLUSTERING(Name)                                    \
  unsigned int get_number_of_clusters() const;                  \
  const Ints& get_cluster(unsigned int i) const;                \
  int get_cluster_representative(unsigned int i) const;         \
  IMP_OBJECT(Name)

#endif  /* IMPSTATISTICS_MACROS_H */
