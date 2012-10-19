/**
 *  \file IMP/core/KinematicForest.cpp
 * \brief Wrapper class for a kinematic tree made of KinematicNode
           objects, interconnected by joints. This data structure
           allows for kinematic control of the tree and
           interconversion between internal and external coordinates.
 *  \authors Dina Schneidman, Barak Raveh
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */

#include <IMP/Model.h>
#include <IMP/core/KinematicForest.h>
#include <IMP/atom/Hierarchy.h>
#include <IMP/base/warning_macros.h>

IMPCORE_BEGIN_NAMESPACE


KinematicForest::KinematicForest(Model* m) :
  m_(m),
  is_internal_coords_updated_(true),
  is_external_coords_updated_(true)
{
}

// build an entire tree from an existing hierarchy
KinematicForest::KinematicForest(Model* m, IMP::atom::Hierarchy hierarchy) :
  m_(m){
  // TODO: implement
  IMP_NOT_IMPLEMENTED;
  IMP_UNUSED(hierarchy);
}


IMPCORE_END_NAMESPACE
