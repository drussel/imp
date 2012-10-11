/**
 *  \file IMP/core/KinematicTree.h
 * \brief Wrapper class for a kinematic tree made of KinematicNode
           objects, interconnected by joints. This data structure
           allows for kinematic control of the tree and
           interconversion between internal and external coordinates.
 *  \authors Dina Schneidman, Barak Raveh
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */

#ifndef IMPCORE_KINEMATIC_TREE_H
#define IMPCORE_KINEMATIC_TREE_H_

#include <IMP/core/KinematicNode.h>
#include <IMP/core/joints.h>
#include <IMP/object.h>
#include <IMP/compatibility/set.h>
#include <IMP/exception.h>

IMPCORE_BEGIN_NAMESPACE


// TODO: perhaps separate collision detection capabilities?
class KinematicTree : public CollisionDetector {
public:
    KinematicTree(Model* m);

    /**
        Builds a kinematic tree automatically from a hierarchy that
        contains rigid bodies and adds a ScoreState to the model, to
        make sure the internal and external coordinates are synced
        before model updating
       TODO: think about what foldtree scheme to use (star?),

     */
    KinematicTree(Model* m, IMP::Hierarchy hierarchy);

    void add_root_rigid_body(IMP::RigidBody rb){
      root_ = KinematicNode::setup_particle(rb.get_particle());
      nodes_.insert(root_)
    }

    /**
       Adds child_rb to the tree, using parent_rb as his father,
       and using a non-constrained joint
     */
    // NOTE: must have root on first call
    // TODO: verify parent_rb is in tree
    void add_child(IMP::RigidBody child_rb, IMP::RigidBody parent_rb)
    {
      add_rigid_body( child_rb, parent_rb, Joint(parent_rb, child_rb) );
    }

    void add_child_from_joint(Joint* joint) {
      // I. make sure parent_rb is in tree
      KinematicNode parent = joint->get_parent();
      if(!nodes_.find(parent)) {
        IMP_THROW("cannot find parent rigid body of joint in kinematic tree, "
                  + ", cannot add child rigid body to tree",
                  IMP::ValueException);
      }

    }

    // first will be root? use default TransformationJoint
    add_rigid_bodies_in_chain(IMP::RigidBodies rbs);

    // first will be root?
    add_rigid_bodies_in_chain(IMP::RigidBodies rbs, Joints joints);

    // rebuild tree (same topology but change directionality)
    reset_root(IMP::Particle* new_root);

    void set_joint_to_rigid_body(IMP::Particle* rb, Joint joint);

    Joint& get_joint_to_rigid_body(IMP::RigidBody rb) const;

    TransformationDOF get_transformation_to_rigid_body
    (IMP::RigidBody rb) const{
        // get joint and call joint::get_transformation()
    }

    // returns the coordinate of any particle in the hierarchy
    // that has a rigid-body ancestor in the kinematic chain
    Vector3D get_coordinates(IMP::Particle p) const;

private:
    IMP::Particles get_children(IMP::Particle parent) const;

    IMP::Particle get_parent(IMP::Particle child) const;

    friend std::ostream& operator<<(std::ostream& s,
                                    const KinematicTree& kt);
private:
    Model* m;
    bool is_cartesian_updated_;
    bool is_internal_updated_;

    /** the root node that serves as spatial anchor to the
        kinematic tree */
    KinematicNode root_;

    /** the set of nodes in the tree */
    // TODO: is vector more efficient memory access wise?
    IMP::compatibility::set<KinematicNode> nodes_;
};


IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_JOINTS_H */
