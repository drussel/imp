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
#include <IMP/object.h>
#include <IMP/compatibility/include/nullptr.h>
#include <IMP/exception.h>


IMPCORE_BEGIN_NAMESPACE

class KinematicForest;

class Joint : public Object{ // TODO: should it be ModelObject or Object?
  friend class KinematicForest;

public:
  /**
     An abstract class for a joint between a parent and a child

     @param parent rigid body upstream of this joint
     @param child rigid body downstream of this joint
     @note we currently assume that a parent cannot be switched
   */
  Joint(RigidBody parent, RigidBody child)
    parent_(parent), child_(child), owner_kf_(nullptr)
  {
    if(!owner_kf_){
      IMP_THROW("IMP supports only joints that are managed by a"
                + " kinematic forest",
                IMP::ValueException);
    }
  }

  /***************** getter methods: ***************/

  KinematicForest* get_owner_kf() const{
    return owner_kf_;
  }

  // returns the transformation that should be applied to all rigid bodies
  // downstream of the joint
  virtual IMP::algebra::Transformation3D get_transformation() const {
    if( get_owner_kf() ) {
      get_owner_kf()->update_all_internal_coordinates();
    }
    return transformation_;
  }

  RigidBody  get_parent_node() const
  { return parent_; }

  RigidBody get_child_node() const
  { return child_; }

 private:

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
      parent_.get_reference_frame()->get_transformation_to();
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

private:
    RigidBody parent_;
    RigidBody child_;
    Transformation3D transformation_;
    KinematicForest* owner_kf_; // the tree that manages updates to this joint
};

/* /\** A joint with a completely non-constrained transformation */
/*     between parent and child nodes */
/* *\/ */
/* class TransformationJoint : public Joint{ */
/*   // TODO */
/* } */

/** Abstract class for all revolute joints **/
class RevoluteJoint : public Joint{

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
               IMP::core::XYZ a,
               IMP::core::XYZ b,
               double angle = 0.0)
   : Joint(parent, child),
    angle_(angle)
    {
      // TODO: who are the witnesses here exactly?
      set_joint_vector( b.get_coordinates() - a.get_coordinates() );
      // TODO: is angle useful for anything
      ss=new RevoluteJointScoreState(p, ...); // TODO: implement that?
      p->get_model()->add_score_state(ss); // TODO: implement that?
    }

 public:
  /******************* getter methods **************/
  double get_angle() const {
    if(get_owner_kf()){
      get_owner_kf()->update_all_external_coordinates( this );
    }
    return angle_;
  }

  IMP::algebra::Vector3D const& get_joint_unit_vector() const
    { return joint_unit_vector_; }

 private:
  /****************** general methods ***************/

  /**
     updates the transformation stored within the joint
     based on the current angle_ value
   */
  void update_transformation_from_angle() const{
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
    transformation_ = IMP::algebra::Transformation3D(r, (r*(-d1_))+d1_);
  }

  /******************* setter methods **************/
  /**
     sets the angle of the revolute joint and update the joint
     transformation accordingly
   */
  void set_angle(double angle) {
    if(get_owner_kf()){
      get_owner_kf()->update_all_internal_coordinates();
    }
    angle_ = angle;
    update_transformation_from_angle();
    if(get_owner_kf()){
      get_owner_kf()->mark_internal_coordinate_changed();
    }
  }

  /** sets v to the axis around which this joint revolves
   */
  void set_joint_vector(IMP::algebra::Vector3D v)
  { joint_unit_vector_ = v.get_unit_vector(); }

 private:
  // the angle in Radians about the joint axis ("unit vector")
  double angle_;

  // the unit vector around which the joint revolves
  IMP::algebra::Vector3D joint_unit_vector_;
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
                             IMP::core::XYZ a,
                             IMP::core::XYZ b,
                             IMP::core::XYZ c,
                             IMP::core::XYZ d) :
  RevoluteJoint(parent, child, b, c),
    a_(a), b_(b), c_(c), d_(d) // TODO: are b_ and c_ redundant?
  {
    // TODO: scorestate for udpating the model? see revolute joint
    update_joint_from_cartesian_witnesses();
  }

 private:
  virtual void update_joint_from_cartesian_witnesses()
  {
    set_joint_axis
      ( c_.get_coordinates() - b_.get_coordinates() );
    // TODO: perhaps the knowledge of normalized joint axis can accelerate
    // the dihedral calculation in next line?
    set_angle( IMP::core::dihedral
               (a_, b_, c_, d_,
                nullptr, // derivatives - TODO: support?
                nullptr,
                nullptr,
                nullptr)
               );
  }

 private:
    IMP::core::XYZ a_;
    IMP::core::XYZ b_;
    IMP::core::XYZ c_;
    IMP::core::XYZ d_;
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

/* // joint in which too rigid bodies may slide along a line */
/* class PrismaticJoint : public Joint{ */
/*     // TODO create prismatic decorator, probably in algebra or kinematics */
/*     PrismaticJoint(IMP::XXX::Prismatic p); */

/*     virtual Transformation3D get_transformation() const; */

/*     virtual void update_joint_from_cartesian_witnesses() = 0; */

/*     set_length(double l); */

/*     double get_length() const; */
/* private: */
/* }; */

/* class CompositeJoint : public Joint{ */
/*     void add_joint(Joint j); */

/*     virtual Transformation3D get_transformation() const; */

/*     virtual void update_joint_from_cartesian_witnesses() = 0; */

/*     Joints& get_inner_joints(); */
/* private: */
/*     Joints joints; */
/* }; */



IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_JOINTS_H */
