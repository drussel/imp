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
  // constructs a revolute joint on the line connecting the reference frames
  // from_rb and to_rb


  static RevoluteJoint setup_particle(Particle*p, RigidBody rb,
                                      Vector3D offset,
                                      Vector3D axis, double angle) {
    add offset, axis, angle to p
      ss=new RevoluteJointScoreState(p, rb)
      p->get_model()->add_score_state(ss)
      store score state in Particle
      add p to rb as the associated joint
      return RevoluteJoint(p)
      }

  virtual Transformation3D get_transformation() const;

  virtual void update_joint_from_cartesian_witnesses();

  void set_angle(double angle);

  double get_angle() const;
};

class DihedralAngleRevoluteJoint : public RevoluteJoint{
    // constructs a dihedral angle based on the last two particles in
    // rb1 and the first two particles rb2
    // TODO: atom is not the place for dihedral, either algebra, or kinematics
      //       module
    DihedralAngleRevoluteJoint(IMP::atom::Dihedral d);

    virtual Transformation3D get_transformation() const;


    virtual void update_joint_from_cartesian_witnesses();

    void set_angle(double angle);

    double get_angle() const;
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
