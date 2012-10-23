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

class Joint : public Object{ // TODO: should it be ModelObject or Object?
  friend class KinematicForest;

  IMP_OBJECT(Joint);

public:
  /**
     An abstract class for a joint between a parent and a child

     @param parent rigid body upstream of this joint
     @param child rigid body downstream of this joint
     @note we currently assume that a parent cannot be switched
   */
 Joint(RigidBody parent, RigidBody child) :
  Object("IMP_CORE_JOINT"),
    parent_(parent), child_(child), owner_kf_(nullptr)
  {
    if(!owner_kf_){
      IMP_THROW("IMP supports only joints that are managed by a"
                << " kinematic forest",
                IMP::ValueException);
    }
  }

  /***************** getter methods: ***************/

  KinematicForest* get_owner_kf() const{
    return owner_kf_;
  }

  // returns the transformation that should be applied to all rigid bodies
  // downstream of the joint
  virtual IMP::algebra::Transformation3D get_transformation() const;

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
    update_child_node_reference_frame() const
  {
    // TODO: make this efficient - indexing? lazy? update flag?
    using namespace IMP::algebra;

    Transformation3D parent_tr =
      parent_.get_reference_frame().get_transformation_to();
    Transformation3D composed_tr =
      parent_tr * get_transformation();
    RigidBody(child_.get_particle()).set_reference_frame
       ( ReferenceFrame3D( composed_tr ));
  }

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
    IMP::algebra::Transformation3D transformation_;
    KinematicForest* owner_kf_; // the tree that manages updates to this joint
};

/* /\** A joint with a completely non-constrained transformation */
/*     between parent and child nodes reference frames */
/* *\/ */
class TransformationJoint : public Joint{
 public:
 TransformationJoint(RigidBody parent,
                     RigidBody child) :
  Joint(parent, child)
    {
      update_joint_from_cartesian_witnesses();
    }

  /**
     Sets the transfromation from parent to child (TODO: vice versa?)
     to t
  */
  void set_transformation(IMP::algebra::Transformation3D transformation);

 protected:
  /**
     sets the joint transformation to the transformation from the
     parent reference frame to the child reference frame (TODO: or
     vide versa), after making sure the external coordinates are
     updated from the owner KinematicForest object)
   */
  virtual void update_joint_from_cartesian_witnesses() {
    // TODO: implement , is it from parent to child or other way?
  }

};

/** Abstract class for all revolute joints **/
class RevoluteJoint : public Joint{
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
               double angle = 0.0)
   : Joint(parent, child),
    angle_(angle)
    {
      // TODO: who are the witnesses here exactly?
      set_joint( b.get_coordinates() - a.get_coordinates(),
                 a.get_coordinates());
      // TODO: is angle useful for anything
      //      ss=new RevoluteJointScoreState(p, ...); // TODO: implement that?
      //p->get_model()->add_score_state(ss); // TODO: implement that?
    }

 public:
  /******************* public getter / setter methods **************/

  /**
     sets the angle of the revolute joint and update the joint
     transformation accordingly
   */
  void set_angle(double angle);

  double get_angle() const;

  IMP::algebra::Vector3D const& get_joint_unit_vector() const
    { return joint_unit_vector_; }

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
    transformation_ = IMP::algebra::Transformation3D(r, (r*(-a_))+a_);
  }

  /******************* protected setter methods **************/
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

class DihedralAngleRevoluteJoint : public RevoluteJoint{
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
  DihedralAngleRevoluteJoint(RigidBody parent,
                             RigidBody child,
                             XYZ a,
                             XYZ b,
                             XYZ c,
                             XYZ d) :
  RevoluteJoint(parent, child, b, c),
    a_(a), b_(b), c_(c), d_(d) // TODO: are b_ and c_ redundant?
  {
    // TODO: scorestate for udpating the model? see revolute joint
    update_joint_from_cartesian_witnesses();
  }

 protected:
  /**
     Update the stored dihedral angle (and the resulting joint transformation)
     to fit the positions of the four cartesian witnesses given upon
     construction.
   */
  virtual void update_joint_from_cartesian_witnesses()
  {
    set_joint( c_.get_coordinates() - b_.get_coordinates(),
               b_.get_coordinates());
    // TODO: perhaps the knowledge of normalized joint axis can accelerate
    // the dihedral calculation in next line?
    set_angle( internal::dihedral
               (a_, b_, c_, d_,
                nullptr, // derivatives - TODO: support?
                nullptr,
                nullptr,
                nullptr)
               );
  }

 private:
    XYZ a_;
    XYZ b_;
    XYZ c_;
    XYZ d_;
};

/* class BondAngleRevoluteJoint : public RevoluteJoint{ */
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

/**
   joint in which too rigid bodies may slide along a line
*/
class PrismaticJoint : public Joint{
 public:
  /******************* Constructors ***********************/
  /**
     Create a prismatic joint whose axis of translation is from a
     to b, which serve as witnesses for the joint transformation
  */
 PrismaticJoint(RigidBody parent, RigidBody child,
                XYZ a, XYZ b) :
  Joint(parent, child), a_(a), b_(b) {
    double tiny_double = 1e-12;
    if( (a.get_coordinates() - b_.get_coordinates()).get_magnitude()
       < tiny_double ) {
      IMP_THROW( "cannot create a prismatic joint with witnesses of"
                 << " identical coordinates " << a << " and " << b,
                 IMP::ValueException );
    }
    update_joint_from_cartesian_witnesses();
  }

  /**
     Create a prismatic joint whose axis of translation is between
     the reference framess of parent and child, who also
     serve as witnesses for the joint transformation
  */
 PrismaticJoint(RigidBody parent, RigidBody child) :
  PrismaticJoint(parent, child, parent, child) {  }

 public:
  /******************* Public getters / setters ***********************/
  /**
     gets the length of the prismatic joint, that is the length
     between the two witnesses
  */
  double get_length() const;

  /**
     sets the length of the prismatic joint to l, that is
     the length between the two witnesses set up upon construction
     (in a lazy fashion)

     @note it is assumed that l > 0, and for efficiency, runtime
           checks for this are not made in fast mode
  */
  void set_length(double l);

 protected:
  /****************** General protected methods *********************/

  /**
      Update the length and transformation of the prismatic joint
      based on the distance and relative orientation of the witnesses
      given upon construction
  */
  virtual void update_joint_from_cartesian_witnesses() {
    using namespace IMP::algebra;

    IMP_USAGE_CHECK
      ( a_.get_coordinates() != b_.get_coordinates(),
        "witnesses of prismatic joint should have different"
        << " coordinates" );
    Vector3D v_diff = b_.get_coordinates() - a_.get_coordinates();
    double mag = v_diff.get_magnitude();
    l_ = mag;
    transformation_ =  // TODO: should implement set_transformation instead?
       IMP::algebra::Transformation3D( v_diff ) ; // TODO: or -v?
  }



 private:

  XYZ a_; // prismatic joint from point

  XYZ b_; // prismatic joint to point

  double l_; // the length of the prismatic joint

};

/* class CompositeJoint : public Joint{ */
/*     void add_joint(Joint j); */

/*     virtual Transformation3D get_transformation() const; */

/*     virtual void update_joint_from_cartesian_witnesses() = 0; */

/*     Joints& get_inner_joints(); */
/* private: */
/*     Joints joints; */
/* }; */

IMP_OBJECTS(Joint,Joints);

IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_JOINTS_H */
