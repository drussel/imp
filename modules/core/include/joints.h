/**
 *  \file IMP/core/joints.h
 *  \brief functionality for defining kinematic joints between rigid bodies
 *         as part of a kinematic tree
 *  \authors Dina Schneidman, Barak Raveh
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */

#ifndef IMPCORE_JOINTS_H
#define IMPCORE_JOINTS_H

#include <IMP/core/KinematicNode.h>
#include <IMP/Object.h>
#include <IMP/compatibility/nullptr.h>
#include <IMP/exception.h>
#include <IMP/core/internal/dihedral_helpers.h>
#include <IMP/algebra/Vector3D.h>
#include <IMP/base/check_macros.h>

IMPCORE_BEGIN_NAMESPACE

class KinematicForest;

class  IMPCOREEXPORT Joint
: public Object
{ // TODO: should it be ModelObject
                             //       or Object, or base::Object?
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
     of 'witness' particles.

     @note Witness particles do not necessarily belong to the child or
           parent rigid bodes.
   */
  virtual void update_joint_from_cartesian_witnesses() = 0;

protected:
    RigidBody parent_;
    RigidBody child_;
    IMP::algebra::Transformation3D transformation_child_to_parent_;
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
     Sets the transfromation from parent to child reference frame
  */
  void set_transformation_child_to_parent
    (IMP::algebra::Transformation3D transformation);

 protected:
  /**
     sets the joint transformation to the transformation from the
     parent reference frame to the child reference frame (TODO: or
     vide versa), after making sure the external coordinates are
     updated from the owner KinematicForest object)
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
  IMP::algebra::Vector3D const& get_joint_unit_vector() const
    { return joint_unit_vector_; }
#endif

 protected:
  /****************** general methods ***************/

  /**
     updates the transformation stored within the joint
     based on the current angle_ value
   */
  void update_transformation_from_angle(){
    double x = joint_unit_vector_[0];
    double y = joint_unit_vector_[1];
    double z = joint_unit_vector_[2];

    double cos_angle= cos(angle_);
    double sin_angle = sin(angle_);
    double s = 1 - cos_angle;
    IMP::algebra::Rotation3D r =
      IMP::algebra::get_rotation_from_matrix
      ( x*x*s + cos_angle,   y*x*s - z*sin_angle, z*x*s + y * sin_angle,
        x*y*s + z*sin_angle, y*y*s + cos_angle,   z*y*s - x*sin_angle,
        x*z*s - y*sin_angle, y*z*s + x*sin_angle, z*z*s + cos_angle );
    transformation_child_to_parent_ =
      IMP::algebra::Transformation3D(r, (r*(-a_))+a_);
  }

  /*********** protected setter methods **********/
  /** sets v to the axis around which this joint revolves
      sets a to the starting point of the joint
   */
  void set_joint(IMP::algebra::Vector3D v, IMP::algebra::Vector3D a){
    joint_unit_vector_ = v.get_unit_vector();
    a_ = a;
  }

 private:
  // the angle in Radians about the joint axis ("unit vector")
  double angle_;

  // the unit vector around which the joint revolves
  IMP::algebra::Vector3D joint_unit_vector_;

  // joint start
  IMP::algebra::Vector3D a_;
};

// inline for speed, dummy pure virtual definition, just for SWIG
RevoluteJoint::~RevoluteJoint()
{}



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
     construction.
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
      given upon construction
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
