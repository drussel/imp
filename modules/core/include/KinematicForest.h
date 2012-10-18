/**
 *  \file IMP/core/KinematicForest.h
 * \brief Wrapper class for a kinematic tree made of KinematicNode
           objects, interconnected by joints. This data structure
           allows for kinematic control of the tree and
           interconversion between internal and external coordinates.
 *  \authors Dina Schneidman, Barak Raveh
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */

#ifndef IMPCORE_KINEMATIC_FOREST_H
#define IMPCORE_KINEMATIC_FOREST_H

#include <IMP/Model.h>
#include <IMP/core/KinematicNode.h>
#include <IMP/core/joints.h>
#include <IMP/object.h>
#include <IMP/compatibility/set.h>
#include <IMP/exception.h>
#include <IMP/base/check_macros.h>
#include <queue>

IMPCORE_BEGIN_NAMESPACE


class KinematicForest  {
public:
    KinematicForest(Model* m);

    /**
        Builds a kinematic tree automatically from a hierarchy that
        contains rigid bodies and adds a ScoreState to the model, to
        make sure the internal and external coordinates are synced
        before model updating
       TODO: think about what foldtree scheme to use (star?),

     */
    KinematicForest(Model* m, IMP::Hierarchy hierarchy);


    /**
       Adds a kinematic edge between parent and child,
       using a TransformationJoint between them, and
       decorating them as KinematicNodes if needed.
     */
    // NOTE: must have root on first call
    // TODO: verify parent_rb is in tree
    Joint* add_edge(IMP::RigidBody parent, IMP::RigidBody child)
    {
      // create joint and associate it with parent and child
      IMP_NEW( TransformationJoint, joint, (parent, child, this) );
      add_joint( joint );
      return joint;
    }

    /**
       Adds a kinematic edge between the joint parent and child
       rigid bodies, decorating them as KinematicNodes if needed.
     */
    void add_edge(Joint* joint) {
      joint->set_owner( this );
      RigidBody parent_rb = joint->get_parent();
      RigidBody child_rb = joint->get_child();
      KinematicNode parent_kn, child_kn;
      // I. decorate parent / child as kinematic nodes if needed,
      //    and store in parent_kn / child_kn resp.
      {
        Particle* parent_p = parent_rb.get_particle();
        if(!KinematicNode::particle_is_instance( parent_p ) ) {
          parent_kn = KinematicNode::setup_particle( parent_p, nullptr );
        } else {
          parent_kn = KinematicNode( parent_p );
        }
        Particle* child_p = child_rb.get_particle();
        if(!KinematicNode::particle_is_instance( child_p ) ) {
          child_kn = KinematicNode::setup_particle( child_p, nullptr );
        } else {
          child_kn = KinematicNode( child_p );
        }
      }
      // update tree topology with parent and child
      // and update roots, joints and nodes lists
      // update parent:
      if( nodes_.find( parent_kn ) == nodes_.end() ){
        nodes_.insert( parent_kn );
        roots_.insert( parent_kn );
      }
      parent_kn.add_child_joint( joint );
      // update child:
      if( nodes_.find( child ) == nodes_.end() ){
        nodes_.insert( child_kn );
      }
      else {
        if( !child_kn.get_parent_joint() ) {
          child_kn.set_parent_joint( joint );
          roots_.erase( child_kn ); // no longer a root
        }
        else {
          IMP_THROW( "IMP currently does not support switching of "
                     + " parents in a kinematic tree",
                     IMP::ValueException );
          // TODO: do we want to allow parent switching? not for now
          //       in this case, should we remove this child from its old
          //       parent?
          //       see note above
        }
      }
      // update joints:
      joints_.push_back( joint);
    }

    /**
       adds edges between each pair of consecutive rigid bodies, using default
       transformation between rigid bodies reference frames

       @param rbs list of n consecutive rigid bodies
    */
    void add_rigid_bodies_in_chain(IMP::RigidBodies rbs){
      for(int i = 0 ; i < rbs.size() - 1; i++){
        add_edge( rbs[i], rbs[i+1] );
      }
    }

    /**
       adds edges between each pair of consecutive rigid bodies in the list
       rbs, using the corresponding joints

       @param rbs list of n consecutive rigid bodies
       @param joints list of n-1 joints to connect consecutive rigid bodies
    */
    void add_rigid_bodies_in_chain(IMP::RigidBodies rbs, Joints joints){
      for(int i = 0 ; i < rbs.size() - 1; i++){
        add_edge( rbs[i], rbs[i+1], joints[i] );
      }
    }

    // rebuild tree (same topology but change directionality)
    reset_root(IMP::Particle* new_root){
      // TODO: implement
      IMP_NOT_IMPLEMENTED;
    }

    void update_all_internal_coordinates(){
      if(is_internal_coords_updated_)
        return;
      for(int i = 0; i < joints_.size(); i++){
        joints_[i]->update_joint_from_cartesian_witnesses();
      }
      is_internal_coords_updated_ = true;
    }

    void update_all_external_coordinates(){
      if(is_external_coords_updated)
        return;
      // tree BFS traversal
      std::queue<KinematicNode> q;
      for(unsinged int i = 0 ; i < roots_.size(); i++){
        q.push( roots_[i] );
      }
      while( !q.empty() ){
        KinematicNode n = q.pop();
        JointsTemp child_joints = n.get_children_joints();
        for(unsigned int i = 0; i < child_joints.size(); i++){
          Joint* joint_i = child_joints[i];
          // TODO: add and implement to joint
          joint_i->update_child_node_reference_frame();
          q.push( KinematicNode(joint_i->get_child_node() ) );
        }
      }
      is_external_coords_updated_ = true;
    }

    void mark_internal_coordinate_changed(  ) {
      update_all_external_coordinates(); // TODO: do we really need it?
      is_external_coords_updated_ = false;
    }

    void mark_external_coordinate_changed(  ) {
      update_all_internal_coordinates(); // TODO: do we really need it?
      is_internal_coords_updated_ = false;
    }

    /**
       sets the corodinates of a particle, and makes sure that particles
       and joints in the tree will return correct external and internal
       coordinates

       @param rb a rigid body that was previously added to the tree
       @param c  new coordinates
    */
    void set_coordinates_safe(IMP::RigidBody rb, IMP::algebra::Vector3D c){
      IMP_USAGE_CHECK( is_member(rb) ,
                       "A KinematicForest can only handle particles "
                       + " that were perviously added to it" );
      rb.set_coordinates( c );
      mark_external_coordinate_changed = true;
    }

    /**
     */
    IMP::algebra::Vector3D get_cooridnates_safe( IMP::RigidBody rb ) const{
      IMP_USAGE_CHECK( is_member(rb) ,
                       "A KinematicForest can only handle particles "
                       + " that were perviously added to it" );
      const_cast<KinematicForest*>(this)->update_all_external_coordinates();
      return rb.get_coordinates();
    }

    /**
     */
    bool is_member(IMP::RigidBody rb){
      Particle* p = rb.get_particle();
      return
        KinematicNode::particle_is_instance( p ) &&
        nodes_.find( KinematicNode( p ) ) != nodes_.end() ;
    }

    // TODO: do we want to add safe access to rigid body members?

    /**
     */
    IMP::algebra::ReferenceFrame3D
      get_reference_frame_safe(IMP::RigidBody rb) const {
      IMP_USAGE_CHECK( is_member(rb) ,
                       "A KinematicForest can only handle particles "
                       + " that were perviously added to it" );
      const_cast<KinematicForest*>(this)->update_all_external_coordinates();
      return rb.get_reference_frame();
    }

    /**
       sets the reference frame of a rigid body, and makes sure that
       particles and joints in the tree will return correct external
       and internal coordinates when queried through the KinematicForest

       @param rb a rigid body that was previously added to the tree
       @param r  new reference frame
    */
    void set_reference_frame_safe
      (IMP::RigidBody rb, IMP::algebra::ReferenceFrame3D r){
      IMP_USAGE_CHECK( is_member(rb) ,
                       "A KinematicForest can only handle particles "
                       + " that were perviously added to it" );
      rb.set_reference_frame( r );
      mark_external_coordinate_changed = true;
    }

    // TODO: handle derivatives, and safe getting / setting of them

private:
    IMP::Particles get_children(IMP::Particle parent) const;

    IMP::Particle get_parent(IMP::Particle child) const;

    friend std::ostream& operator<<(std::ostream& s,
                                    const KinematicForest& kt);
private:
    Model* m_;

    bool is_internal_coords_updated_;
    bool is_external_coords_updated_;

    /** the root nodes that serves as spatial anchor to the
        kinematic trees in the forest */
    IMP::compatibility::set<KinematicNode> roots_;

    /** the set of nodes in the tree */
    IMP::compatibility::set<KinematicNode> nodes_;

    // TODO: do we really need this?
    IMP::compatibility::vector<Joint*> joints_;

};


IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_KINEMATIC_FOREST_H */
