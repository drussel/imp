/**
 *  \file vector_search.h   \brief Functions to generate vectors.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPALGEBRA_VECTOR_SEARCH_H
#define IMPALGEBRA_VECTOR_SEARCH_H

#include "VectorD.h"
#include <IMP/base_types.h>
#include "internal/ann.h"
#ifdef IMP_USE_CGAL
#include <IMP/cgal/internal/knn.h>
#endif
#include "internal/linear_knn.h"
#include <fstream>

#ifdef IMP_USE_CGAL
#include <IMP/cgal/internal/knn.h>
#define IMP_KNN_DATA IMP::cgal::internal::KNNData
#elif defined(IMP_USE_ANN)
#define IMP_KNN_DATA internal::ANNData
#else
#define IMP_KNN_DATA internal::LinearKNNData
#endif


IMPALGEBRA_BEGIN_NAMESPACE

/** @name Vector Search

    These functions classes create various search structures
    over sets of vectors.
    @{
*/

/** Build a structure for finding nearest neighbors. Different
    implementations are used depending on what optional dependencies
    are available.
    \uses{class NearestNeighborD, CGAL}
    \uses{class NearestNeighborD, ANN}
 */
template <unsigned int D>
class NearestNeighborD {
  IMP_KNN_DATA<D> data_;
  double eps_;
#if IMP_BUILD < IMP_FAST
  mutable std::ofstream query_log_;
#endif
  template <class It>
  void instantiate(It b, It e) {
    if (0) {
      // compile all of them
      Ints ret;
#ifdef IMP_USE_ANN
      internal::ANNData<D> ann(b,e);
      ann.fill_nearest_neighbors(*b, 3U, eps_, ret);
#endif
      internal::LinearKNNData<D> linear(b,e);
      linear.fill_nearest_neighbors(*b, 3U, eps_, ret);
    }
  }
public:
  template <class It>
  NearestNeighborD(It b, It e, double epsilon):
    data_(b,e), eps_(epsilon){
    instantiate(b,e);
  }
  NearestNeighborD(const std::vector<VectorD<D> > &vs,
                   double epsilon=0):
    data_(vs.begin(), vs.end()),
    eps_(epsilon) {
    instantiate(vs.begin(), vs.end());
  }
  //! Log the points and queries to a file for performance studies
  void set_query_log(std::string fname) {
#if IMP_BUILD < IMP_FAST
    query_log_.open(fname.c_str());
    for (unsigned int i=0; i< data_.get_number_of_points(); ++i) {
      query_log_ << spaces_io(data_.get_point(i)) << std::endl;
    }
    query_log_ << std::endl;
#endif
  }

  unsigned int get_nearest_neighbor(const VectorD<D> &q) const {
#if IMP_BUILD < IMP_FAST
    if (query_log_) {
      query_log_ << spaces_io(q) << " " << 1 << std::endl;
    }
#endif
    Ints ret(2);
    data_.fill_nearest_neighbors(q, 1U, eps_, ret);
    return ret[1];
  }
  /** Search using the ith point in the input set. */
  unsigned int get_nearest_neighbor(unsigned int i) const {
#if IMP_BUILD < IMP_FAST
    if (query_log_) {
      query_log_ << i << " " << 1 << std::endl;
    }
#endif
    Ints ret(2);
    data_.fill_nearest_neighbors(data_.get_point(i), 2U, eps_, ret);
    return ret[1];
  }
  /** Search using the ith point in the input set. Return the k
      nearest neighbors.*/
  Ints get_nearest_neighbors(unsigned int i, unsigned int k) const {
#if IMP_BUILD < IMP_FAST
    if (query_log_) {
      query_log_ << i << " " << k << std::endl;
    }
#endif
    Ints ret(k+1);
    data_.fill_nearest_neighbors(data_.get_point(i), k+1, eps_, ret);
    return Ints(++ret.begin(), ret.end());
  }
  /** Find all points within the provided distance. */
  Ints get_in_ball(unsigned int i, double distance) const {
    Ints ret;
    data_.fill_nearest_neighbors(data_.get_point(i), distance, eps_, ret);
    return Ints(++ret.begin(), ret.end());
  }
  /** Find all points within the provided distance. */
  Ints get_in_ball(const VectorD<D> &pt, double distance) const {
    Ints ret;
    data_.fill_nearest_neighbors(pt, distance, eps_, ret);
    return ret;
  }
};

/** @} */

IMPALGEBRA_END_NAMESPACE

#endif  /* IMPALGEBRA_VECTOR_SEARCH_H */
