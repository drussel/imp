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


IMPCORE_END_NAMESPACE
