/**
 *  \file IMP/core/KinematicNode.h
 *  \brief functionality for defining nodes on a kinematic chain
 *  \authors Dina Schneidman, Barak Raveh
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */

#ifndef IMPCORE_KINEMATIC_NODE_H
#define IMPCORE_KINEMATIC_NODE_H




#ifndef IMP_PROTEINCHAIN_H
#define IMP_PROTEINCHAIN_H

#include <IMP/core/rigid_bodies.h>
#include <IMP/exception.h>


IMPCORE_BEGIN_NAMESPACE

// TODO needs a mechanism to notify the joint if the particles in it have
//       changed position
// TODO: what can be moved to Model fast indexing system
// TODO: check for cycles
// TODO: At first step, verify that all rigid bodies are not RigidMembers
//       (because that would be a mess). We can extend it in the future.

// a rigid body that upstream from a joint
class KinematicNode : IMP::RigidBody{

  IMP_DECORATOR(KinematicNode, RigidBody);

private:
  static ObjectsKey get_children_joints_key()
    {
        static ObjectsKey k("children_joints");
        return k;
    }


    static ObjectKey get_parent_joint_key()
    {
        static ObjectKey k("parent_joint");
        return k;
    }

public:

    /** sets up a node in a kinematic tree

        @param parent_joint The joint upstream of this kinematic
                            node. Setting the the transformation of
                            any upstream joint is expected to affect
                            the reference frame of this node.  Use
                            nullptr for root nodes.
    */
    static Joint setup_particle(Particle*p,
                                Joint* parent_joint = nullptr,
                                Joints& children_joints = ParticlesTemp())  {
      if (RigidMember::particle_is_instance(p)) {
        // see also RigidBody::add_member
        IMP_THROW("RigidMemer cannot be set as KinematicNode at this point," +
                  " in order to guarantee coherent coordinates update",
                  IMP::ValueException);
      }
      if (!RigidBody::particle_is_instance(p)) {
          RigidBody::setup_particle(p, ParticlesTemp() );
      }
      p->add_attribute(get_parent_joint_key(), parent_joint)
      p->add_attribute(get_children_joints_key(), children_joints);
    }

    Joint* get_parent_joint() {
        // TODO: indexes system?
        Object* o = get_particle()->get_value( get_parent_joint_key() );
        return static_cast<Joint*>(o);
    }

    Joints get_children_joints() {
        Objects objs =
          get_model()->get_attribute( get_children_joints_key(),
                                      get_particle_index() );
        Joints joints;
        for(unsigned int i = 0; i <= objs.size(); i++){
            Joint* j = static_cast<Joint*>(objs[i]);
            joints.push_back(j);
        }
        return joints;
    }

    void set_children_joints(Joints children) {
      get_model()->set_attribute( get_children_joints_key(),
                                  get_particle_index(), children );
    }


    void add_child_joint(Joint* j) {
      Joints joints = get_children_joints();
      joints.push_back(this);
      kn.set_children_joints(joints);
    }

    void set_parent_joint(Joint* j) {
      get_model()->set_attribute( get_parent_joint_key(),
                                  get_particle_index(), j );
    }

    /**
       makes sure that the reference frame of this rigid body is
       up to date
       // TODO: do we need this or it is enough to have only the joint function?
    */
   void update_reference_frame_from_joint() {
     Joint* parent_joint = get_parent_joint();
     if(parent_joint != nullptr) {
       joint->update_child_node_reference_frame();
     }
   }

   void update_all_downstream_nodes_from_joints(){
     using namespace IMP::algebra;

     JointsTemp children_joints = get_children_joints();
     for(unsigned int i = 0; i < children_joints.size(); i++) {
       // TODO: make more efficient by e.g. caching this node's ref. frame?
       // TODO: efficiency?
       children_joints[i]->update_child_node_reference_frame();
       children_joints[i]->update_downstream_rigid_bodies_from_joints();
     }
    }

    // updates the transformation stored in the joint based on the
    // Cartesian coordinates of the appropriate particles that are
    // stored in it to 'witness' the joint transformation
    void update_all_downstream_joints_transformations_from_nodes() {
         if (RevoluteJoint::particle_is_instance(get_particle()) {
                RevoluteJoint(get_particle())
                  .update_joint_from_cartesian_witnesses();
         }
        // XXX and so on... XXX
    }
};

    IMP_DECORATORS_DEF(KinematicNode, KinematicNodes);RigidMember,RigidMembers);
IMP_DECORATORS(KinematicNode, KinematicNodes, RigidBodies);

IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_KINEMATIC_NODE_H */
