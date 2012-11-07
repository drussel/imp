/**
 *  \file TransformationJoint.cpp
 *  \brief a kinematic joints between rigid bodies that allows any
 *         transformation
 *  \authors Dina Schneidman, Barak Raveh
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */


#include "KinematicForest.h"
#include "KinematicNode.h"
#include "TransformationJoint.h"
#include <IMP/Object.h>
#include <IMP/compatibility/nullptr.h>
#include <IMP/exception.h>
#include <IMP/algebra/Transformation3D.h>

IMPCORE_BEGIN_NAMESPACE

/********************** Transformation Joint ***************/

TransformationJoint::TransformationJoint
(RigidBody parent, RigidBody child)
  :  Joint(parent, child)
{
  update_joint_from_cartesian_witnesses();
}


// Sets the transfromation from parent to child
void
TransformationJoint::set_transformation_child_to_parent
(IMP::algebra::Transformation3D transformation)
{
  if(get_owner_kf()){
    get_owner_kf()->update_all_internal_coordinates( );
  }
  Joint::set_transformation_child_to_parent_no_checks( transformation );
  if(get_owner_kf()){
    get_owner_kf()->mark_internal_coordinates_changed();
  }
}

void
TransformationJoint::update_joint_from_cartesian_witnesses()
{
  // TODO: make this efficient - indexing? lazy? update flag?
  using namespace IMP::algebra;

  ReferenceFrame3D parent_rf = get_parent_node().get_reference_frame();
  ReferenceFrame3D child_rf = get_child_node().get_reference_frame();
  const Transformation3D& tr_global_to_parent =
    parent_rf.get_transformation_from();
  const Transformation3D& tr_child_to_global =
    child_rf.get_transformation_to();
  set_transformation_child_to_parent_no_checks
    (tr_global_to_parent * tr_child_to_global);
}

IMPCORE_END_NAMESPACE
