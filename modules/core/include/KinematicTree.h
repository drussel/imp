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
        KinematicNode n = KinematicNode::setup_particle(p, NULL);
    }

    // NOTE: must have root on first call
    void add_rigid_body(IMP::RigidBody parent_rb, IMP::RigidBody rb)
    {
        add_rigid_body( rb, parent_rb, TransformationJoint(rb, parent_rb) );
    }

    void add_rigid_body_with_dihedral(IMP::RigidBody rb,
                                      IMP::RigidBody parent_rb,
                                      Joint joint);

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
                                    const KinematicChain& kc);
private:
    Model* m;
    bool is_cartesian_updated_;
    bool is_internal_updated_;
    <XXX tree structure XXX> tree_;
    };


IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_JOINTS_H */
