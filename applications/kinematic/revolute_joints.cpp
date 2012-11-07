/**
 *  \file revolute_joints.cpp
 *  \brief functionality for defining various revolute kinematic
 *         joints between rigid bodies as part of a kinematic tree,
 *         including RevoluteJoint, DihedralAngleRevoluteJoint, and
 *         BondAngleRevoluteJoint
 *  \authors Dina Schneidman, Barak Raveh
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */


#include "KinematicForest.h"
#include "KinematicNode.h"
#include "revolute_joints.h"
#include <IMP/Object.h>
#include <IMP/compatibility/nullptr.h>
#include <IMP/exception.h>
#include <IMP/algebra/Transformation3D.h>

IMPCORE_BEGIN_NAMESPACE


/********************** Revolute Joint ***************/

RevoluteJoint::RevoluteJoint
( RigidBody parent, RigidBody child,
  IMP::algebra::Vector3D origin,
  IMP::algebra::Vector3D axis_direction,
  double initial_angle )
  : Joint(parent, child)
{
  // TODO: who are the witnesses here exactly?
  set_revolute_joint_params
    ( origin,
      axis_direction,
      initial_angle);

  //      ss=new RevoluteJointScoreState(p, ...); // TODO: implement that?
  //p->get_model()->add_score_state(ss); // TODO: implement that?
}

// definition of dummy pure virtual, just for SWIG
RevoluteJoint::~RevoluteJoint()
{}

// assumes angle and parent reference frame are updated
void
RevoluteJoint::update_child_node_reference_frame() const
{
  using namespace IMP::algebra;
  std::cout
    << "Updating child node reference frame of RevoluteJoint with angle "
    << angle_ << std::endl;

  // update child to parent transformation:
  Transformation3D R = get_rotation_about_joint();
  ReferenceFrame3D parent_rf =
    get_parent_node().get_reference_frame();
  const Transformation3D& tr_parent_to_global =
    parent_rf.get_transformation_to();
 // TODO: can we really assume that tr_child_to_parent_no_rotation_
 // remains constant, given e.g. CompositeJoint
  const Transformation3D tr_child_to_global =
    R * tr_parent_to_global * tr_child_to_parent_no_rotation_ ;

  RigidBody child_rb = RigidBody(get_child_node().get_particle());
  child_rb.set_reference_frame
    ( ReferenceFrame3D( tr_child_to_global ) );

  // TODO: do we really need to store or calculate child to parent?
  const Transformation3D& tr_global_to_parent =
    parent_rf.get_transformation_from();
  const Transformation3D tr_child_to_parent =
    tr_global_to_parent * tr_child_to_global ;
  const_cast<RevoluteJoint*>(this)->set_transformation_child_to_parent_no_checks
    ( tr_child_to_parent );

}


double
RevoluteJoint::get_angle() const
{
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
  if(get_owner_kf()){
    get_owner_kf()->mark_internal_coordinates_changed();
  }
}

// Sets the revolute joint to revolve about the axis whose origin
// is rot_origin and direction is axis_v, and assuming the initial
// rotation angle is currently initial_angle
void
RevoluteJoint::set_revolute_joint_params
( IMP::algebra::Vector3D rot_origin,
  IMP::algebra::Vector3D axis_v,
  double initial_angle )
{
  using namespace IMP::algebra;

  rot_origin_ = rot_origin;
  rot_axis_unit_vector_ = axis_v.get_unit_vector();
  angle_ = initial_angle;

  // compute the transformation from child to global, if the
  // rotation was zero, that is - use the inverse of the
  // transformation by initial angle, to compute the transformation
  // for angle=0:
  Transformation3D R =
    get_rotation_about_joint();
  Transformation3D tr_child_to_global =
    get_child_node().get_reference_frame().get_transformation_to();
  Transformation3D tr_global_to_parent =
    get_parent_node().get_reference_frame().get_transformation_from();
  tr_child_to_parent_no_rotation_ =
    tr_global_to_parent * R.get_inverse() * tr_child_to_global;
  // update the transformation from child to parent
  Joint::set_transformation_child_to_parent_no_checks
    ( tr_global_to_parent * tr_child_to_global );
}

/********************** DihedralAngleRevoluteJoint ***************/

DihedralAngleRevoluteJoint
::DihedralAngleRevoluteJoint
(RigidBody parent, RigidBody child,
 XYZ a, XYZ b, XYZ c, XYZ d) :
  RevoluteJoint(parent, child,
                b.get_coordinates(),
                c.get_coordinates() - b.get_coordinates(),
                IMP::core::get_dihedral(a, b, c, d)
                ),
  a_(a), b_(b), c_(c), d_(d) // TODO: are b_ and c_ redundant?
{
  // TODO: scorestate for udpating the model? see revolute joint
  update_joint_from_cartesian_witnesses();
}


void
DihedralAngleRevoluteJoint::update_joint_from_cartesian_witnesses()
{
  // TODO: add derivative support?
  double angle = IMP::core::get_dihedral(a_, b_, c_, d_);
  RevoluteJoint::set_revolute_joint_params
    ( b_.get_coordinates(),
      c_.get_coordinates() - b_.get_coordinates(),
      angle );
  // TODO: perhaps the knowledge of normalized joint axis can accelerate
  // the dihedral calculation in get_angle_from_witnesses()?
}



/********************** BondAngleRevoluteJoint ***************/

// control the bond angle a-b-c
BondAngleRevoluteJoint
::BondAngleRevoluteJoint
(RigidBody parent, RigidBody child,
 XYZ a, XYZ b, XYZ c) :
  RevoluteJoint(parent,
                child,
                b.get_coordinates(),
                IMP::core::get_perpendicular_vector(a, b, c),
                IMP::core::get_angle(a,b,c)
                ),
  a_(a), b_(b), c_(c) // TODO: are b_ and c_ redundant?
{
  // TODO: scorestate for udpating the model? see revolute joint
  update_joint_from_cartesian_witnesses();
}


void
BondAngleRevoluteJoint::update_joint_from_cartesian_witnesses()
{
  // TODO: add derivative support?
  double angle = IMP::core::get_angle(a_, b_, c_);
  RevoluteJoint::set_revolute_joint_params
    ( b_.get_coordinates(),
      IMP::core::get_perpendicular_vector(a_, b_, c_),
      angle );
}



IMPCORE_END_NAMESPACE
