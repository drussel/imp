/**
 *  \file joints.h
 *  \brief functionality for defining various kinematic joints between
 *         rigid bodies as part of a kinematic tree
 *  \authors Dina Schneidman, Barak Raveh
 *

 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */

#ifndef IMPCORE_JOINTS_H
#define IMPCORE_JOINTS_H

#include "KinematicNode.h"
#include "Joint.h"
#include <IMP/base/Object.h>
#include <IMP/compatibility/nullptr.h>
#include <IMP/exception.h>
#include <IMP/core/internal/dihedral_helpers.h>
#include <IMP/algebra/Vector3D.h>
#include <IMP/base/check_macros.h>

IMPCORE_BEGIN_NAMESPACE

class KinematicForest;

/********************** RevoluteJoint ***************/


/** Abstract class for all revolute joints **/
class IMPCOREEXPORT RevoluteJoint : public Joint{
 public:
  /**
     constructs a revolute joint on the line connecting a and b,
     with an initial angle 'angle'

     @param parent,child kinematic nodes upstream and downstream (resp.) of this
                    joint
     @param origin origin of revolute joint rotation
     @param axis_direction direction of rotation axis, relative to the origin of rotation
     @param angle initial rotation angle assumed for the current state of parent and child
  **/
 RevoluteJoint(RigidBody parent,
               RigidBody child,
               IMP::algebra::Vector3D origin,
               IMP::algebra::Vector3D axis_direction,
               double angle = 0.0);


  // pure virtual dtr to declare as abstrat class for SWIG
 virtual ~RevoluteJoint() = 0;
 public:
 /******* public getter / setter methods *********/

  /**
     sets the angle of the revolute joint and update the joint
     transformation accordingly
   */
  void set_angle(double angle);

  double get_angle() const;

#ifndef SWIG
  IMP::algebra::Vector3D const& get_rot_origin() const
    { return rot_origin_; }

  IMP::algebra::Vector3D const& get_rot_axis_unit_vector() const
    { return rot_axis_unit_vector_; }
#endif

 protected:
  /****************** general methods ***************/

  /**
     Returns the transformation matrix for roatating a vector
     in global coordinates about the vector passing through the joint origin
     get_rot_origin() and in the direction of get_axis_unit_vector().
   */
  IMP::algebra::Transformation3D
    get_rotation_about_joint()
    {
      IMP::algebra::Rotation3D R =
        IMP::algebra::get_rotation_about_normalized_axis
        ( rot_axis_unit_vector_, angle_ );
      IMP::algebra::Transformation3D R_origin =
        IMP::algebra::get_rotation_about_point(rot_origin_, R);
      return R_origin;
    }


  /**
     updates the transformation from child to parent coordinated that
     is stored within the joint, based on the current angle_ value
     and the stored axis of rotation
   */
  void update_child_to_parent_transformation_from_angle(){
    using namespace IMP::algebra;
    Transformation3D R =
      get_rotation_about_joint();
    ReferenceFrame3D parent_rf =
      get_parent_node().get_reference_frame();
    const Transformation3D& tr_global_to_parent =
      parent_rf.get_transformation_from();
    Joint::set_transformation_child_to_parent_no_checks
      ( tr_global_to_parent
        * R
        * tr_child_to_global_without_rotation_ );
  }

  /*********** protected setter methods **********/

  /**
      Sets the revolute joint to revolve about the axis whose origin
      is rot_origin and direction is axis_v, and assuming the initial
      rotation angle is currently initial_angle
      @note it is assumed that all external coordinates are updated
            before a call to this method

      @param rot_origin the global origin of rotation
      @param axis_v the axis direction about which this joint revolves
      @param angle as the joint initial angle
   */
  void set_revolute_joint_params
    ( IMP::algebra::Vector3D rot_origin,
      IMP::algebra::Vector3D axis_v,
      double initial_angle );

 private:
  // the angle in Radians about the joint axis ("unit vector")
  double angle_;

  // the unit vector around which the joint revolves
  IMP::algebra::Vector3D rot_axis_unit_vector_;

  // joint start
  IMP::algebra::Vector3D rot_origin_;

  // The transformation that would bring a vector
  // from child coordinates to global coordinates, were it
  // that the rotation angle would have been zero
  IMP::algebra::Transformation3D tr_child_to_global_without_rotation_;
};



/********************** DihedralAngleRevoluteJoint ***************/


class  IMPCOREEXPORT
DihedralAngleRevoluteJoint : public RevoluteJoint{
 public:
  /**
     constructs a dihedral angle that revolves around the axis b-c,
     using a,b,c,d as witnesses for the dihedral angle
     // TODO1: use core/internal/dihedral_helpers.h + move to algebra?
     // TODO2: do we want to handle derivatives?

     @param parent,child kinematic nodes upstream and downstream (resp.) of
                    this joint
     @param a,b,c,d 'witnesses' whose coordinates define the dihedral
                    angle, as the angle between a-b and c-d, with
                    respect to the revolute axis b-c
     */
  DihedralAngleRevoluteJoint
    (RigidBody parent, RigidBody child,
     XYZ a, XYZ b, XYZ c, XYZ d);

 protected:
  /**
     Update the stored dihedral angle (and the resulting joint transformation)
     to fit the positions of the four cartesian witnesses given upon
     construction.

     @note It is assumed that external coordinates are updated before
           calling this function.
   */
  virtual void update_joint_from_cartesian_witnesses();

 private:
    XYZ a_;
    XYZ b_;
    XYZ c_;
    XYZ d_;
};

/********************** BondAngleRevoluteJoint ***************/

class  IMPCOREEXPORT BondAngleRevoluteJoint : public RevoluteJoint{
 public:
  /**
     constructs a joint that controls the angle a-b-c. The joint
     revolves around the axis that passes through b, normal to the
     plane containing a, b and c. a,b and c are the witnesses for the
     bond angle.
     // TODO: do we want to handle derivatives?

     @param parent,child kinematic nodes upstream and downstream (resp.) of
                    this joint
     @param a,b,c 'witnesses' whose coordinates define the joint angle a-b-c
  */
  BondAngleRevoluteJoint
    (RigidBody parent, RigidBody child,
     XYZ a, XYZ b, XYZ c);

 protected:
  /**
     Update the stored bind angle (and the resulting joint transformation)
     to fit the positions of the three cartesian witnesses given upon
     construction.

     @note It is assumed that external coordinates are updated before
           calling this function.
   */
  virtual void update_joint_from_cartesian_witnesses();

 private:
    XYZ a_;
    XYZ b_;
    XYZ c_;
};



IMP_OBJECTS(RevoluteJoint, RevoluteJoints);
IMP_OBJECTS(DihedralAngleRevoluteJoint, DihedralAngleRevoluteJoints);
IMP_OBJECTS(BondAngleRevoluteJoint, BondAngleRevolteJoints);

IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_JOINTS_H */
