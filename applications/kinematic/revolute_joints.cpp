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
( RigidBody parent, RigidBody child  )
  : Joint(parent, child)
{
  //      ss=new RevoluteJointScoreState(p, ...); // TODO: implement that?
  //p->get_model()->add_score_state(ss); // TODO: implement that?
}

// definition of dummy pure virtual, just for SWIG
RevoluteJoint::~RevoluteJoint()
{}

// assumes angle and parent reference frame are updated
// and uses witnesses to get axis of rotation
// TODO: mergw with Joint::update_child... and separate
//       different parts only?
void
RevoluteJoint::update_child_node_reference_frame() const
{
  using namespace IMP::algebra;
  std::cout
    << "Updating child node reference frame of RevoluteJoint with angle "
    << angle_ * 180 / 3.151426 <<
    " and last updated angle " <<
    last_updated_angle_* 180 / 3.151426
    << std::endl;

  // Preparations:
  Transformation3D R = get_rotation_about_joint();
  ReferenceFrame3D parent_rf =
    get_parent_node().get_reference_frame();
  ReferenceFrame3D child_rf =
    get_child_node().get_reference_frame();
  const Transformation3D& tr_parent_to_global =
    parent_rf.get_transformation_to();
  const Transformation3D& tr_global_to_parent =
    parent_rf.get_transformation_from();
  const Transformation3D& tr_child_to_parent_old =
    get_transformation_child_to_parent_no_checks();
  RigidBody child_rb = RigidBody(get_child_node().get_particle());

  // Actual stuff - propagate parent transformation + rotation to child:
  std::cout << "old child to parent " << tr_child_to_parent_old << std::endl;
  std::cout << "old parent to global " << tr_parent_to_global << std::endl;
  std::cout << "old ref frame " << child_rf.get_transformation_to()
            << std::endl;
  std::cout << "rotation " << R << std::endl;
  Transformation3D tr_child_to_global_new =
    R * tr_parent_to_global * tr_child_to_parent_old;
  std::cout << "new ref frame " << tr_child_to_global_new << std::endl;
  // TODO: should we add a set_reference_frame_lazy() variant? this
  // has effects that need to be thought through
  child_rb.set_reference_frame
    ( ReferenceFrame3D( tr_child_to_global_new ) );
  last_updated_angle_ = angle_;
  const_cast<RevoluteJoint*>(this)
    ->set_transformation_child_to_parent_no_checks
    (tr_global_to_parent * tr_child_to_global_new);

  // IMP_USAGE_CHECK( angle_ == get_current_angle_from_cartesian_witnesses() ,
  //                  "It was expected that the actual angle "
  //                  << get_current_angle_from_cartesian_witnesses()
  //                  << " and the set angle " << angle_
  //                  << " would be equal");
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

/********************** DihedralAngleRevoluteJoint ***************/

DihedralAngleRevoluteJoint
::DihedralAngleRevoluteJoint
(RigidBody parent, RigidBody child,
 XYZ a, XYZ b, XYZ c, XYZ d) :
  RevoluteJoint(parent, child),
  a_(a), b_(b), c_(c), d_(d) // TODO: are b_ and c_ redundant?
{
  // TODO: scorestate for udpating the model? see revolute joint
  update_axis_of_rotation_from_cartesian_witnesses();
  update_joint_from_cartesian_witnesses(); // angle only
}


double
DihedralAngleRevoluteJoint
::get_current_angle_from_cartesian_witnesses() const
{
  // TODO: add derivative support?
  return IMP::core::get_dihedral(a_, b_, c_, d_);
}



/********************** BondAngleRevoluteJoint ***************/

// control the bond angle a-b-c
BondAngleRevoluteJoint
::BondAngleRevoluteJoint
(RigidBody parent, RigidBody child,
 XYZ a, XYZ b, XYZ c) :
  RevoluteJoint(parent, child),
  a_(a), b_(b), c_(c)
{
  // TODO: scorestate for udpating the model? see revolute joint
  update_axis_of_rotation_from_cartesian_witnesses();
  update_joint_from_cartesian_witnesses();
}

double
BondAngleRevoluteJoint
::get_current_angle_from_cartesian_witnesses() const
{
  // TODO: add derivative support?
  return IMP::core::get_angle(a_, b_, c_);
}



IMPCORE_END_NAMESPACE
