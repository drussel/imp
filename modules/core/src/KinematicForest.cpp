/**
 *  \file IMP/core/KinematicForest.cpp
\ * \brief Wrapper class for a kinematic tree made of KinematicNode
          objects, interconnected by joints. This data structure
          allows for kinematic control of the tree and
          interconversion between internal and external coordinates.
 *  \authors Dina Schneidman, Barak Raveh
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */

#include <IMP/Model.h>
#include <IMP/core/KinematicForest.h>
#include <IMP/atom/Hierarchy.h>
#include <IMP/base/warning_macros.h>

IMPCORE_BEGIN_NAMESPACE

void
KinematicForest::add_edge(Joint* joint)
{
  joint->set_owner_kf( this );
  RigidBody parent_rb = joint->get_parent_node();
  RigidBody child_rb = joint->get_child_node();
  KinematicNode parent_kn, child_kn;
  // decorate parent and store here
  {
    Particle* parent_p = parent_rb.get_particle();
    if(!KinematicNode::particle_is_instance( parent_p ) ) {
      std::cout << "Setting up parent" << std::endl;
      parent_kn = KinematicNode::setup_particle( parent_p, this );
      nodes_.insert( parent_kn );
      roots_.insert( parent_kn );
    } else {
      parent_kn = KinematicNode( parent_p );
      if( parent_kn.get_owner() != this ) {
        IMP_THROW( "the parent rigid body " << parent_rb
                   << " in the joint " << joint
                   << " was already stored in a different kinematic forest -"
                   << " this IMP version does not support such switching",
                   IMP::ValueException );
      }
    }

    // decorare child and store here
    Particle* child_p = child_rb.get_particle();
    if(!KinematicNode::particle_is_instance( child_p ) ) {
      child_kn = KinematicNode::setup_particle( child_p, this, joint );
      nodes_.insert( child_kn );
    } else {
      child_kn = KinematicNode( child_p );
      if( child_kn.get_owner() != this ){
        IMP_THROW( "the child rigid body " << child_rb
                   << " in the joint " << joint
                   << " was already stored in a different kinematic forest -"
                   << " this IMP version does not support such switching",
                   IMP::ValueException );
      }
      if( roots_.find( child_kn) == roots_.end() ) {
        roots_.erase( child_kn ); // will no longer be a root
      }
      else {
        IMP_THROW( "IMP currently does not support switching of "
                   << " parents in a kinematic tree",
                   IMP::ValueException );
      }
    }
  }

  // store joint
  parent_kn.add_out_joint( joint );
  child_kn.set_in_joint( joint );
  joints_.push_back( joint);
}


void KinematicForest::do_show(std::ostream & os) const
{
  for(unsigned int i = 0; i < joints_.size(); i++){
    if (i >= 1) {
      os << ", ";
    }
    os << *(joints_[i]);
  }
}


KinematicForest::KinematicForest(Model* m) :
  Object("IMP_CORE_KINEMATIC_FOREST"),
  m_(m),
  is_internal_coords_updated_(true),
  is_external_coords_updated_(true)
{
}

// build an entire tree from an existing hierarchy
KinematicForest::KinematicForest(Model* m, IMP::atom::Hierarchy hierarchy) :
  Object("IMP_CORE_KINEMATIC_FOREST"),
  m_(m){
  // TODO: implement
  IMP_NOT_IMPLEMENTED;
  IMP_UNUSED(hierarchy);
}


IMPCORE_END_NAMESPACE
