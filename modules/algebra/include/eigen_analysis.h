/**
 *  \file eigen_analysis.h
 *  \brief Principal component analysis of a set of points
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 */


#ifndef IMPALGEBRA_EIGEN_ANALYSIS_H
#define IMPALGEBRA_EIGEN_ANALYSIS_H

#include "IMP/algebra/Vector3D.h"
#include <IMP/log.h>

IMPALGEBRA_BEGIN_NAMESPACE
/**    \ingroup uninitialized_default
 */
class IMPALGEBRAEXPORT PrincipalComponentAnalysis {
public:
  PrincipalComponentAnalysis(){}
  PrincipalComponentAnalysis(
    const VectorD<3> &pc1,const VectorD<3> &pc2,
    const VectorD<3> &pc3,VectorD<3> values,
    Vector3D centroid) : eigen_values_(values), centroid_(centroid){
    eigen_vecs_.push_back(pc1);
    eigen_vecs_.push_back(pc2);
    eigen_vecs_.push_back(pc3);
  }
  void show(std::ostream& out = std::cout) const {
    out << "Eigen values: ";
    eigen_values_.show(out);
    out<<std::endl<<"First eigen vector : ";
    eigen_vecs_[0].show(out);
    out<<std::endl<<"Second eigen vector : ";
    eigen_vecs_[1].show(out);
    out<<std::endl<<"Third eigen vector : ";
    eigen_vecs_[2].show(out);
    out<<std::endl<<"eigen values : ";
    eigen_values_.show(out);
    out<<std::endl<<"centroid : ";
    centroid_.show(out);
  }
  VectorD<3> get_principal_component(unsigned int i) const {
    IMP_INTERNAL_CHECK(i<3, "index is not between 0, 1 or 2");
    return eigen_vecs_[i];
  }
  double get_principal_value(unsigned int i) const {
    IMP_INTERNAL_CHECK(i<3, "index is not between 0, 1 or 2");
    return eigen_values_[i];
  }
  inline Vector3D get_centroid() const {return centroid_;}
protected:
  std::vector<VectorD<3> > eigen_vecs_;
  VectorD<3> eigen_values_;
  Vector3D centroid_;
};


IMP_OUTPUT_OPERATOR(algebra::PrincipalComponentAnalysis);

//! Perform principle components analysis on a set of vectors
/** \relatesalso PrincipalComponentAnalysis
 */
IMPALGEBRAEXPORT PrincipalComponentAnalysis get_principal_components(
                              const std::vector<VectorD<3> > &ps);

IMPALGEBRA_END_NAMESPACE
#endif  /* IMPALGEBRA_EIGEN_ANALYSIS_H */
