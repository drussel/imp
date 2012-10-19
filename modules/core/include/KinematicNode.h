/**
 *  \file IMP/core/KinematicNode.h
 *  \brief functionality for defining nodes on a kinematic chain
 *  \authors Dina Schneidman, Barak Raveh
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */

#ifndef IMPCORE_KINEMATIC_NODE_H
#define IMPCORE_KINEMATIC_NODE_H

#include <IMP/core/rigid_bodies.h>
#include <IMP/core/joints.h>
#include <IMP/exception.h>

IMPCORE_BEGIN_NAMESPACE

class KinematicForest;

// TODO: what can be moved to Model fast indexing system
// TODO: check for cycles
// TODO: privatize most methods INCLUDING constructor

/**
  A KinematicNode is a rigid body that is connected by a joint to other
  rigid bodies
*/
class KinematicNode : public RigidBody{
  friend class KinematicForest;

  IMP_DECORATOR(KinematicNode, RigidBody);

 public:

  /**
     @brief Return true if the particle is a kinematic nodea (has the
     appropriate properties).
  */
  static bool particle_is_instance(Particle *p) {
    ParticleIndex pi = p->get_index();
    Model* m = p->get_model();
    return
      m->get_has_attribute( get_children_joints_key(), pi) &&
      m->get_has_attribute( get_parent_joint_key(), pi);
  }


 private:

    /** sets up a node in a kinematic tree

        @param parent_joint The joint upstream of this kinematic
                            node. Setting the the transformation of
                            any upstream joint is expected to affect
                            the reference frame of this node.  Use
                            nullptr for root nodes.

        @note Private so as only KinematicForest can setup new kinematic nodes
    */
    static KinematicNode setup_particle(Particle*p,
                                Joint* parent_joint = nullptr,
                                Joints children_joints = Joints())  {
      if (RigidMember::particle_is_instance(p)) {
        // see also RigidBody::add_member
        IMP_THROW("RigidMemer cannot be set as KinematicNode at this point," <<
                  " in order to guarantee coherent coordinates update",
                  IMP::ValueException);
      }
      if (!RigidBody::particle_is_instance(p)) {
          RigidBody::setup_particle(p, ParticlesTemp() );
      }
      p->get_model()->add_attribute(get_parent_joint_key(),
                                    p->get_index(),
                                    parent_joint);
      p->get_model()->add_attribute(get_children_joints_key(),
                                    p->get_index(),
                                    children_joints);
      return KinematicNode(p);
    }

    Joint* get_parent_joint() {
        // TODO: indexes system?
        Object* o =
          get_model()->get_attribute( get_parent_joint_key(),
                                      get_particle_index() );
          // get_particle()->get_value( get_parent_joint_key() );
        return static_cast<Joint*>(o);
    }

    Joints get_children_joints() {
        Objects objs =
          get_model()->get_attribute( get_children_joints_key(),
                                      get_particle_index() );
        Joints joints;
        for(unsigned int i = 0; i <= objs.size(); i++){
          Object * o = objs[i];
          Joint* j = static_cast<Joint*>(o);
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
      joints.push_back(j);
      set_children_joints(joints);
    }

    void set_parent_joint(Joint* j) {
      get_model()->set_attribute( get_parent_joint_key(),
                                  get_particle_index(), j );
    }

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
};

//IMP_DECORATORS_DEF(KinematicNode, KinematicNodes);
IMP_DECORATORS(KinematicNode, KinematicNodes, RigidBody);

IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_KINEMATIC_NODE_H */
