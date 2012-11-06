/**
 *  \file joints.h
 *  \brief functionality for defining kinematic joints between rigid bodies
 *         as part of a kinematic tree
 *  \authors Dina Schneidman, Barak Raveh
 *

 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */

#ifndef IMPCORE_JOINTS_H
#define IMPCORE_JOINTS_H

#include "KinematicNode.h"
#include <IMP/base/Object.h>
#include <IMP/compatibility/nullptr.h>
#include <IMP/exception.h>
#include <IMP/core/internal/dihedral_helpers.h>
#include <IMP/algebra/Vector3D.h>
#include <IMP/base/check_macros.h>

IMPCORE_BEGIN_NAMESPACE

class KinematicForest;

class  IMPCOREEXPORT Joint
: public IMP::base::Object
{
  friend class KinematicForest;

  IMP_OBJECT(Joint);

public:
  /**
     An abstract class for a joint between a parent and a child

     @param parent rigid body upstream of this joint
     @param child rigid body downstream of this joint
     @note we currently assume that a parent cannot be switched
   */
  Joint(RigidBody parent, RigidBody child);

  /***************** getter methods: ***************/

  KinematicForest* get_owner_kf() const{
    return owner_kf_;
  }

#ifndef SWIG
  /**
     returns the transformation of a vector from the child
     reference frame to the parent reference frame
  */
  virtual const IMP::algebra::Transformation3D&
    get_transformation_child_to_parent() const;

#endif

  RigidBody  get_parent_node() const
  { return parent_; }

  RigidBody get_child_node() const
  { return child_; }

 protected:

  /***************** setter methods: ***************/

  void set_owner_kf(KinematicForest* kf) {
    owner_kf_ = kf;
  }

  /**
     Sets the transfromation from parent to child reference frame
     (without any checks that internal coords are updated, and without
      marking the owner internal coords as changed)
  */
  void set_transformation_child_to_parent_no_checks
    (IMP::algebra::Transformation3D transformation) {
    tr_child_to_parent_ = transformation;
  }


  /**************** general methods: **************/

  /**
     Updates the reference frame of the rigid body directly downstream
     of this joint

    // TODO: validate test - must be buggy :)
  */
  virtual void
    update_child_node_reference_frame() const;

  /**
     Updates the joint transformation based on external coordinates
     of 'witness' particles, assuming all external coordinates are
     up-to-date.

     @note Witness particles do not necessarily belong to the child or
           parent rigid bodes.
   */
  virtual void update_joint_from_cartesian_witnesses() = 0;

private:
    RigidBody parent_;
    RigidBody child_;
    IMP::algebra::Transformation3D tr_child_to_parent_;
    KinematicForest* owner_kf_; // the tree that manages updates to this joint
};

/********************** TransformationJoint ***************/


/* /\** A joint with a completely non-constrained transformation */
/*     between parent and child nodes reference frames */
/* *\/ */
class  IMPCOREEXPORT
TransformationJoint : public Joint{
 public:
  TransformationJoint(RigidBody parent,
                      RigidBody child);

  /**
     Sets the transfromation from parent to child reference frame,
     in a safe way - that is, after updating all intrnal coordinates
     from external if needed, and marking the owner internal coordinates
     as changed.
  */
  void set_transformation_child_to_parent
    (IMP::algebra::Transformation3D transformation);

 protected:
  /**
     sets the joint transformation to the transformation from the
     parent reference frame to the child reference frame (TODO: or
     vide versa), assuming the external coordinates are
     up-to-date.
   */
  virtual void update_joint_from_cartesian_witnesses() ;

};

/********************** RevoluteJoint ***************/


/** Abstract class for all revolute joints **/
class IMPCOREEXPORT RevoluteJoint : public Joint{
 public:
  /**
     constructs a revolute joint on the line connecting a and b,
     with an initial angle 'angle'

     @param parent,child kinematic nodes upstream and downstream (resp.) of this
                    joint
     @param a,b end points of revolute joint axis
     @param angle initial rotation angle
  **/
 RevoluteJoint(RigidBody parent,
               RigidBody child,
               XYZ a,
               XYZ b,
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
      double initial_angle ) {
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
    tr_child_to_global_without_rotation_ =
      R.get_inverse() * tr_child_to_global;
    // update the transformation from child to parent
    Transformation3D tr_global_to_parent =
      get_parent_node().get_reference_frame().get_transformation_from();
    Joint::set_transformation_child_to_parent_no_checks
      ( tr_child_to_global * tr_global_to_parent );
  }

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
     constructs a dihedral angle that revolutes around the axis b-c,
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
     construction, assuming all external coords are updated.
   */
  virtual void update_joint_from_cartesian_witnesses();

 private:
    XYZ a_;
    XYZ b_;
    XYZ c_;
    XYZ d_;
};

/********************** BondAngleRevoluteJoint ***************/

/* class  IMPCOREEXPORT BondAngleRevoluteJoint : public RevoluteJoint{ */
/*     // rotation of rigid body around the plane defined by three atoms */
/*     // TODO: atom is not the place for Angle, either algebra, or */
/*               kinematics */
/*       //       module */
/*     BondAngleRevoluteJoint(IMP::atom::Angle a); */

/*     virtual Transformation3D get_transformation() const; */

/*     virtual void update_joint_from_cartesian_witnesses(); */

/*     void set_angle(double angle); */

/*     double get_angle(); */
/* private: */
/*     IMP::atom::Angle a; */
/* }; */


/********************** PrismaticJoint ***************/

/**
   joint in which too rigid bodies may slide along a line
*/
class  IMPCOREEXPORT PrismaticJoint : public Joint{
 public:
  /********* Constructors ********/
  /**
     Create a prismatic joint whose axis of translation is from a
     to b, which serve as witnesses for the joint transformation
     (a is associated with the parent and b with the child)
  */
 PrismaticJoint(RigidBody parent, RigidBody child,
                XYZ a, XYZ b);

  /**
     Create a prismatic joint whose axis of translation is between
     the reference framess of parent and child, who also
     serve as witnesses for the joint transformation
  */
 PrismaticJoint(RigidBody parent, RigidBody child) :
  PrismaticJoint(parent, child, parent, child) {  }

 public:
  /************* Public getters / setters *************/
  /**
     gets the length of the prismatic joint, that is the length
     between the two witnesses
  */
  double get_length() const;

  /**
     sets the length of the prismatic joint to l, that is
     the length between the two witnesses set up upon construction
     (in a lazy fashion, without updating external coords).
     Updates the owner of the change in internal coordinates.

     @note it is assumed that l > 0, and for efficiency, runtime
           checks for this are not made in fast mode
  */
  void set_length(double l);

 protected:
  /************ General protected methods *************/

  /**
      Update the length and transformation of the prismatic joint
      based on the distance and relative orientation of the witnesses
      given upon construction, assuming all external coordinates are
      up-to-date.
  */
  virtual void update_joint_from_cartesian_witnesses();

 private:

  XYZ a_; // prismatic joint from point, associated with parent
  XYZ b_; // prismatic joint to point, associated with child
  double l_; // the length of the prismatic joint

};


/********************** Joint ***************/

/* class  IMPCOREEXPORT CompositeJoint : public Joint{ */
/*     void add_joint(Joint j); */

/*     virtual Transformation3D get_transformation() const; */

/*     virtual void update_joint_from_cartesian_witnesses() = 0; */

/*     Joints& get_inner_joints(); */
/* private: */
/*     Joints joints; */
/* }; */

IMP_OBJECTS(Joint,Joints);
IMP_OBJECTS(RevoluteJoint,RevoluteJoints);
IMP_OBJECTS(DihedralAngleRevoluteJoint, DihedralAngleRevoluteJoints);
IMP_OBJECTS(TransformationJoint,TransformationJoints);
IMP_OBJECTS(PrismaticJoint,PrismaticJoints);

IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_JOINTS_H */
