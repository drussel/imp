/**
 *  \file revolute_joints.h
 *  \brief functionality for defining various revolute kinematic
 *         joints between rigid bodies as part of a kinematic tree,
 *         including RevoluteJoint, DihedralAngleRevoluteJoint, and
 *         BondAngleRevoluteJoint
 *  \authors Dina Schneidman, Barak Raveh
 *

 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */

#ifndef IMPCORE_REVOLUTE_JOINTS_H
#define IMPCORE_REVOLUTE_JOINTS_H

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
class IMPCOREEXPORT RevoluteJoint :
public Joint{
 public:
  /**
     constructs a revolute joint on the line connecting a and b,
     with an initial angle 'angle'

     @param parent,child kinematic nodes upstream and downstream (resp.) of this
                    joint
  **/
 RevoluteJoint(RigidBody parent,
               RigidBody child);


  // pure virtual dtr to declare as abstrat class for SWIG
 virtual ~RevoluteJoint() = 0;

 /****** general public methods **********/
 public:

 /**
    Updates the reference frame of the child node by this joint
    angle, assuming the parent reference frame and the witnesses
    that affect update_axis_of_rotation_from_cartesian_witnesses()
    are all updated already
  */
 virtual void update_child_node_reference_frame() const;


 /******* getter / setter methods *********/
 public:

  /**
     sets the angle of the revolute joint and marks the internal
     coordinates as changed in the kinematic forest object
   */
  void set_angle(double angle);

  /**
     gets the angle of the revolute joint. This method is kinematically
     safe (it triggers an update to internal coordinates if needed)
  */
  double get_angle() const;

 protected:
#ifndef SWIG
  // in global coordinates
  IMP::algebra::Vector3D const& get_rot_axis_origin() const
    { return rot_axis_origin_; }

  // in global coordinates
  IMP::algebra::Vector3D const& get_rot_axis_unit_vector() const
    { return rot_axis_unit_vector_; }
#endif

  /****************** general protected methods ***************/

 protected:
  /**
      this protected method updates the rot_axis_unit_vector_
      and rot_axis_origin_ variables based on the cartesian witnesses
      appropriate for a specific implementation of this abstract class,
      using global coordinates (assuming the parent ref frame and the
      cartesian witnesses global coordinates are all updated)
  */
  virtual void update_axis_of_rotation_from_cartesian_witnesses() = 0;

  /**
      this protected method uses the cartesian witnesses to compute
      the actual current angle of this joint (assuming external coordinates
      of required cartesian witnesses are up to date).
      @note this method does not update the angle stored in this joint,
            which may be strictly different (if external or internal coords
            are outdated)
  */
  virtual double get_current_angle_from_cartesian_witnesses() const = 0;


  /**
     Update the stored angle from the four cartesian witnesses given upon
     construction.

     @note It is assumed that external coordinates are updated before
           calling this function.
   */
  virtual void update_joint_from_cartesian_witnesses(){
    angle_ = get_current_angle_from_cartesian_witnesses();
    last_updated_angle_ = angle_;
  }

  /**
     Returns the transformation matrix for roatating a vector in
     global coordinates about the axis of the joint, in a way that
     would bring the cartesian witnesses to the correct joint angle
     (as measured by get_angle_from_cartesian_witnesses() ).

     @note it is assumed that the cartesian witnesses for the joint axis of
     rotation (used in update_axis_of_rotation_from_cartesian_witnesses())
     all have updated XYZ coordinates.
   */
  IMP::algebra::Transformation3D
    get_rotation_about_joint() const
    {
      const_cast<RevoluteJoint*>(this)
        ->update_axis_of_rotation_from_cartesian_witnesses();
      std::cout << "get_rotation " << angle_ << ", last_updated_angle = "
                << last_updated_angle_ << std::endl;
      // rotate by the difference from last_updated_angle_
      IMP::algebra::Rotation3D R =
        IMP::algebra::get_rotation_about_normalized_axis
        ( rot_axis_unit_vector_, angle_ - last_updated_angle_ );
      IMP::algebra::Transformation3D R_origin =
        IMP::algebra::get_rotation_about_point(rot_axis_origin_, R);
      std::pair< IMP::algebra::Vector3D, double > aa;
      aa = IMP::algebra::get_axis_and_angle( R );
      std::cout << "R: "
                << "axis = " << aa.first
                << "; angle = " << aa.second * 180.0 / 3.141256 << " deg"
                << std::endl;
      aa = IMP::algebra::get_axis_and_angle( R_origin.get_rotation() );
      std::cout << "R_origin: "
                << "axis = " << aa.first
                << "; angle = " << aa.second * 180.0 / 3.141256 << " deg"
                << "; translation = " << R_origin.get_translation()
                << std::endl;

   return R_origin;
    }


 protected:
  // the angle in Radians about the joint axis ("unit vector")
  double angle_;

  // the angle used when the child reference frame was last up-to-date
  // mutable cause logically does not change the state of this object
  // (or is that so?)
  mutable double last_updated_angle_;

  // the unit vector around which the joint revolves in global coords
  IMP::algebra::Vector3D rot_axis_unit_vector_;

  // the joint origin of rotation in global coords
  IMP::algebra::Vector3D rot_axis_origin_;
};



/********************** DihedralAngleRevoluteJoint ***************/


class  IMPCOREEXPORT
DihedralAngleRevoluteJoint : public RevoluteJoint{
 public:
  /**
     constructs a dihedral angle that revolves around the axis b-c,
     using a,b,c,d as witnesses for the dihedral angle
     // TODO: do we want to handle derivatives?

     @param parent,child kinematic nodes upstream and downstream (resp.) of
                    this joint
     @param a,b,c,d 'witnesses' whose coordinates define the dihedral
                    angle between the planes containing a-b-c and
                    b-c-d)

     @note It is assumed that neither a, b and c are downstream of child,
           and also that d is not upstream of it
     */
  DihedralAngleRevoluteJoint
    (RigidBody parent, RigidBody child,
     XYZ a, XYZ b, XYZ c, XYZ d);

 protected:
  /**
      updates the rot_axis_unit_vector_ and rot_axis_origin_ variables
      in global coordinates based on the witnesses b_ and c_,
      using b_-c_ as the axis of rotation
  */
  virtual void update_axis_of_rotation_from_cartesian_witnesses(){
    IMP_USAGE_CHECK
      ( IMP::algebra::get_distance( b_.get_coordinates(), c_.get_coordinates() )
        > 1e-12 ,
        "witnesses b and c must be non identical beyone numerical error" );
    rot_axis_origin_ = b_.get_coordinates();
    IMP::algebra::Vector3D v = c_.get_coordinates() - b_.get_coordinates();
    rot_axis_unit_vector_ = v.get_unit_vector();
  };

  /**
      this protected method uses the cartesian witnesses to compute
      the actual current dihedral angle of this joint (assuming
      external coordinates of required cartesian witnesses are up to
      date)
  */
  virtual double get_current_angle_from_cartesian_witnesses() const;


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

     @note It is assumed that a and b are upstream of or inside
           this joint's child rigid body, and that c is downstream of
           it or inside it.
  */
  BondAngleRevoluteJoint
    (RigidBody parent, RigidBody child,
     XYZ a, XYZ b, XYZ c);

 protected:

  /**
     this protected method uses the cartesian witnesses to compute the
     actual current bond angle of this joint (assuming external
     coordinates of required cartesian witnesses are up to date)
  */
  virtual double get_current_angle_from_cartesian_witnesses() const;

  /**
     updates the rot_axis_unit_vector_ and rot_axis_origin_ variables
     using b_ as origin of rotation and a vector perpendicular to the
     plane containing a_,b_,c_ as axis of rotation, in global
     coordinates
  */
  virtual void update_axis_of_rotation_from_cartesian_witnesses(){
    rot_axis_unit_vector_ =
      IMP::core::get_perpendicular_vector(a_, b_, c_).get_unit_vector();
    rot_axis_origin_ = b_.get_coordinates() ;
  };

 private:
    XYZ a_;
    XYZ b_;
    XYZ c_;
};



IMP_OBJECTS(RevoluteJoint, RevoluteJoints);
IMP_OBJECTS(DihedralAngleRevoluteJoint, DihedralAngleRevoluteJoints);
IMP_OBJECTS(BondAngleRevoluteJoint, BondAngleRevolteJoints);

IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_REVOLUTE_JOINTS_H */
