/**
 *  \file CnSymmAxisDetector.h
 *  \brief Adapted from cnmultifit
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPMULTIFIT_CN_SYMM_AXIS_DETECTOR_H
#define IMPMULTIFIT_CN_SYMM_AXIS_DETECTOR_H
#include <IMP/algebra/eigen_analysis.h>
#include <IMP/algebra/Vector3D.h>
#include <IMP/em/DensityMap.h>
#include <IMP/core/XYZ.h>
#include <IMP/atom/Hierarchy.h>
#include <vector>
#include "multifit_config.h"
IMPMULTIFIT_BEGIN_NAMESPACE
class IMPMULTIFITEXPORT CnSymmAxisDetector {
public:
  CnSymmAxisDetector(int symm_deg);
  ~CnSymmAxisDetector();
  algebra::PrincipalComponentAnalysis get_pca() const { return pca_;}
  float calc_symm_score(int symm_axis_ind) const;
  algebra::Vector3D get_symmetry_axis() const;
  int get_symmetry_axis_index() const;
  int get_non_symmetry_axis_length() const{
    int symm_axis_ind=get_symmetry_axis_index();
    int non_ind;
    if(symm_axis_ind == 0) {non_ind=1;}
    if(symm_axis_ind == 2) {non_ind=1;}
    if(symm_axis_ind == 1) {non_ind=0;}
    return std::sqrt(pca_.get_principal_value(non_ind));
  }
  void init_from_density(IMP::em::DensityMap *dmap,float density_threshold,
                         float top_p=0.8);
  void init_from_protein(const atom::Hierarchies &mhs);
  void show(std::ostream& out=std::cout) const {
    out<<"symm degree:"<<symm_deg_<<std::endl;
    out<<"symm axis:"<<get_symmetry_axis_index()<<std::endl;
    pca_.show(out);
  }
protected:
  Float symm_avg(const algebra::Vector3D &start_p,
                 const algebra::Vector3D &symm_vec) const;
  algebra::Vector3Ds vecs_;
  Pointer<em::DensityMap> dmap_;
  algebra::PrincipalComponentAnalysis pca_;
  int symm_deg_;
  bool initialized_;
  algebra::Transformation3D to_native_,from_native_;
  Floats values_;
};
IMPMULTIFIT_END_NAMESPACE
#endif /* IMPMULTIFIT_CN_SYMM_AXIS_DETECTOR_H */
