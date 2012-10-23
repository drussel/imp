/**
 *  \file IMP/core/joints.cpp
 *  \brief functionality for defining kinematic joints between rigid bodies
 *         as part of a kinematic tree
 *  \authors Dina Schneidman, Barak Raveh
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */


#include <IMP/core/KinematicForest.h>
#include <IMP/core/KinematicNode.h>
#include <IMP/Object.h>
#include <IMP/compatibility/nullptr.h>
#include <IMP/exception.h>
#include <IMP/algebra/Transformation3D.h>

IMPCORE_BEGIN_NAMESPACE

// returns the transformation that should be applied to all rigid bodies
// downstream of the joint
IMP::algebra::Transformation3D
Joint::get_transformation() const {
  if( get_owner_kf() ) {
    get_owner_kf()->update_all_internal_coordinates();
  }
  return transformation_;
}

// Sets the transfromation from parent to child (TODO: vice versa?)
// to t
void
TransformationJoint::set_transformation
(IMP::algebra::Transformation3D transformation) {
  if(get_owner_kf()){
    get_owner_kf()->update_all_internal_coordinates( );
  }
  transformation_ = transformation;
  if(get_owner_kf()){
    get_owner_kf()->mark_internal_coordinates_changed();
  }
}


double
RevoluteJoint::get_angle() const {
  if(get_owner_kf()){
    get_owner_kf()->update_all_internal_coordinates( );
  }
  return angle_;
}


//   sets the angle of the revolute joint and update the joint
//   transformation accordingly
void
RevoluteJoint::set_angle(double angle) {
  if(get_owner_kf()){
    get_owner_kf()->update_all_internal_coordinates();
  }
  angle_ = angle;
  update_transformation_from_angle();
  if(get_owner_kf()){
    get_owner_kf()->mark_internal_coordinates_changed();
  }
}

double
PrismaticJoint::get_length() const{
  if(get_owner_kf()){
    get_owner_kf()->update_all_internal_coordinates( );
  }
  return l_;
}

void
PrismaticJoint::set_length(double l){
  IMP_USAGE_CHECK( l > 0 ,
                   "Only a strictly positive length is expected for"
                   << " prismatic joints" );
  if(get_owner_kf()){
    get_owner_kf()->update_all_internal_coordinates();
  }
  l_ = l;
  IMP::algebra::Vector3D v_diff =
    b_.get_coordinates() - a_.get_coordinates();
  IMP::algebra::Vector3D translation =
    l_ * v_diff.get_unit_vector();
  transformation_ = IMP::algebra::Transformation3D( translation );
  if(get_owner_kf()){
    get_owner_kf()->mark_internal_coordinates_changed();
  }
}


IMPCORE_END_NAMESPACE
