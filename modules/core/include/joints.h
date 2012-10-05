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


class Joint : public Object{
public:
    Joint(KinematicNode from, KinematicNode to) :
        from_(from), to_(to)
    {
     // compute transformation from from_rb to to_rb
    }

    // returns the transformation that should be applied to all rigid bodies
    // downstream of the joint
    virtual IMP::algebra::Transformation3D get_transformation() const
    { return transformation_}

    virtual void update_joint_from_cartesian_witnesses();

    void set_transformation(IMP::algebra::Transformation3D t)
    { transformation_ = t; }

private:
    KinematicNode from_;
    KinematicNode to_;
    Transformation3D transformation_;
};

class RevoluteJoint : public Joint{

  // constructs a revolute joint on the line connecting a and b,
  // with an initial angle 'angle'
  // TODO: cstr from two rigid body reference frames?
  RevoluteJoint(IMP::core::XYZ a, IMP::core::XYZ b, double angle)
    {
      // TODO: who are the witnesses here exactly?
      set_joint_vector( b.get_coordinates() - a.get_coordinates() );
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

  virtual void update_joint_from_cartesian_witnesses();

  void set_angle(double angle) { angle_ = angle; }

  double get_angle() const { return angle_; }

  /** sets v to the axis around which this joint revolves
   */
  void set_joint_vector(IMP::algebra::Vector3D v)
  { joint_unit_vector_ = v.get_unit_vector(); }

  IMP::algebra::Vector3D const& get_joint_unit_vector() const
    { return joint_unit_vector_; }
 private:
  angle_;

  // the unit vector around which the joint revolves
  IMP::algebra::Vector3D joint_unit_vector_;
};

class DihedralAngleRevoluteJoint : public RevoluteJoint{
  /**
     constructs a dihedral angle that revolutes around the axis b-c,
     using a,b,c,d as witnesses for the dihedral angle
     // TODO1: use core/internal/dihedral_helpers.h + move to algebra?
     // TODO2: do we want to handle derivatives?

     @param a,b,c,d 'witnesses' whose coordinates define the dihedral
                    angle, as the angle between a-b and c-d, with
                    respect to the revolute axis b-c
     */
  DihedralAngleRevoluteJoint(IMP::core::XYZ a,
                             IMP::core::XYZ b,
                             IMP::core::XYZ c,
                             IMP::core::XYZ d) :
  RevoluteJoint(b, c),
  a_(a), b_(b), c_(c), d_(d)
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
    set_joint_axis( c_.get_coordinates() - b_.get_coordinates() );
    // TODO: perhaps the knowledge of normalized joint axis can accelerate
    // the dihedral calculation in next line?
    angle_ = IMP::core::dihedral(a_, b_, c_, d_,
                                 nullptr, // derivatives - TODO: support?
                                 nullptr,
                                 nullptr,
                                 nullptr);
  }

  void set_angle(double angle) { angle_ = angle; }

  double get_angle() const { return angle_; }

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
