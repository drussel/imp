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

IMPCORE_BEGIN_NAMESPACE


class Joint : public Object{ // TODO: should it be ModelObject or Object?
public:
  /**
     An abstract class for a joint between a parent and a child

     @note we currently assume that a parent cannot be switched
   */
 Joint(RigidBody parent, RigidBody child)
  {
    // setup parent as a kinematic node asscoiated with this joint
    Particle* parent_p = parent.get_particle();
    if(!KinematicNode::particle_is_instance( parent_p ) ) {
      Joints joints();
      joints.push_back(this);
      KinematicNode::setup_particle( parent_p, null, joints );
    } else {
      KinematicNode( parent_p ).add_child_joint( this );
    }
    parent_ = KinematicNode( parent_p );
    // setup child as a kinematic node assosicated with this joint
    Particle* child_p = child.get_particle();
    if(!KinematicNode::particle_is_instance( child_p ) ) {
      KinematicNode::setup_particle( child_p, this );
    } else {
      if( KinematicNode( child_p ).get_parent_joint() != this ) {
        IMP_THROW( "IMP currently does not support switching of "
                   + " parents in a kinematic tree",
                   IMP::ValueException );
        // TODO: do we want to allow parent switching? not for now
        //       in this case, should we remove this child from its old parent?
        //       see note above
        }
    }
    child_ = KinematicNode( child_p );
  }


    // returns the transformation that should be applied to all rigid bodies
    // downstream of the joint
    virtual IMP::algebra::Transformation3D get_transformation() const
    { return transformation_}

    /**
       // TODO: this should probably not be here anymore
       Computes the reference frame of the rigid body downstream of
       this joint, which is the reference frame of the upstream rigid
       body, transformed by this joint's transformation.
    */
    virtual void
      update_child_node_reference_frame() const
    {
     // TODO: make this efficient - indexing? lazy? update flag?
     using namespace IMP::algebra;

     parent_.update_reference_frame_from_joint(); // TODO: should we cache
                                                  // this for next line?
     Transformation3D parent_tr =
       parent_.get_reference_frame()->get_transformation_to();
     Transformation3D composed_tr = compose
       ( parent_tr, get_transformation() );
     RigidBody(child_.get_particle()).set_reference_frame
       ( ReferenceFrame3D( composed_tr ));
    }



    /**
       // TODO: this should probably not be here anymore
    */
    virtual IMP::algebra::Transformation3D
      update_downstream_nodes_reference_frame() const
    {
     // TODO: make this efficient - indexing? lazy? update flag?
     using namespace IMP::algebra;

     update_child_node_reference_frame();
     child_.update_downstream_nodes_from_joints();
    }


    virtual void update_joint_from_cartesian_witnesses() = 0;

    void set_transformation(IMP::algebra::Transformation3D t)
    { transformation_ = t; }

    KinematicNode get_parent_node() { return parent_; }

    KinematicNode get_child_node() { return child_; }

private:
    KinematicNode parent_;
    KinematicNode child_;
    Transformation3D transformation_;
};

/** A joint with a completely non-constrained transformation
    between parent and child nodes
*/
class TransformationJoint : public Joint{
  // TODO
}

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

  virtual void Transformation3D get_transformation() const
  {
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
    return IMP::algebra::Transformation3D(r, (r*(-d1_))+d1_);
  }

  // abstract
  virtual void update_joint_from_cartesian_witnesses() = 0;

  void set_angle(double angle) { angle_ = angle; }

  double get_angle() const { return angle_; }

  /** sets v to the axis around which this joint revolves
   */
  void set_joint_vector(IMP::algebra::Vector3D v)
  { joint_unit_vector_ = v.get_unit_vector(); }

  IMP::algebra::Vector3D const& get_joint_unit_vector() const
    { return joint_unit_vector_; }
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


  virtual Transformation3D get_transformation() const
  {
    return RevoluteJoint::get_transformation();
  }

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

class BondAngleRevoluteJoint : public RevoluteJoint{
    // rotation of rigid body around the plane defined by three atoms
    // TODO: atom is not the place for Angle, either algebra, or kinematics
      //       module
    BondAngleRevoluteJoint(IMP::atom::Angle a);

    virtual Transformation3D get_transformation() const;

    virtual void update_joint_from_cartesian_witnesses();

    void set_angle(double angle);

    double get_angle();
private:
    IMP::atom::Angle a;
};

// joint in which too rigid bodies may slide along a line
class PrismaticJoint : public Joint{
    // TODO create prismatic decorator, probably in algebra or kinematics
    PrismaticJoint(IMP::XXX::Prismatic p);

    virtual Transformation3D get_transformation() const;

    virtual void update_joint_from_cartesian_witnesses() = 0;

    set_length(double l);

    double get_length() const;
private:
};

class CompositeJoint : public Joint{
    void add_joint(Joint j);

    virtual Transformation3D get_transformation() const;

    virtual void update_joint_from_cartesian_witnesses() = 0;

    Joints& get_inner_joints();
private:
    Joints joints;
};



IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_JOINTS_H */
