/**
 *  \file fitting_clustering.h
 *  \brief Cluster fitting solutions by rmsd
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */
#ifndef IMPMULTIFIT_FITTING_CLUSTERING_H
#define IMPMULTIFIT_FITTING_CLUSTERING_H

#include <string>
#include <vector>
#include <IMP/algebra/Transformation3D.h>
#include <IMP/algebra/Rotation3D.h>
#include <IMP/algebra/Vector3D.h>
#include <IMP/em/rigid_fitting.h>
#include "multifit_config.h"
IMPMULTIFIT_BEGIN_NAMESPACE

#if !defined(IMP_DOXYGEN) && !defined(SWIG)
/**
\param[in] pdb_filename cluster transformations with respect to this protein
\param[in] ts the transformations with respect to the transformation in the PDB
\param[out] clsutered_ts the clustered transformations
\param[in] spacing transformations with rmsd < spacing*2 are clustered together
*/
void IMPMULTIFITEXPORT fitting_clustering (
    const IMP::atom::Hierarchy &mh,
    const IMP::em::FittingSolutions &ts,
    IMP::em::FittingSolutions &clustered_ts,float spacing,
    int top_sols,float rmsd);
#endif

IMPMULTIFIT_END_NAMESPACE

#endif /* IMPMULTIFIT_FITTING_CLUSTERING_H */
