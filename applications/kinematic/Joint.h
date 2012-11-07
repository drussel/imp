/**
 *  \file joints.h
 *  \brief functionality for defining a kinematic joint between rigid bodies
 *         as part of a kinematic tree
 *  \authors Dina Schneidman, Barak Raveh
 *

 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */

#ifndef IMPCORE_JOINT_H
#define IMPCORE_JOINT_H

#include "KinematicNode.h"
#include <IMP/base/Object.h>
#include <IMP/compatibility/nullptr.h>
#include <IMP/exception.h>
#include <IMP/core/internal/dihedral_helpers.h>
#include <IMP/algebra/Vector3D.h>
#include <IMP/base/check_macros.h>

IMPCORE_BEGIN_NAMESPACE

class KinematicForest;

/**
    Abstract class for joints between rigid bodies in a kinematic
    tree.
*/
class  IMPCOREEXPORT Joint
: public IMP::base::Object
{
  friend class KinematicForest;
  friend class CompositeJoint;

  IMP_OBJECT(Joint);

public:
  /**
     An abstract class for a joint between a parent and a child

     @param parent rigid body upstream of this joint
     @param child rigid body downstream of this joint
     @note we currently assume that a parent cannot be switched
   */
  Joint(RigidBody parent, RigidBody child);

  /***************** getter methods: ***************/

  KinematicForest* get_owner_kf() const{
    return owner_kf_;
  }

#ifndef SWIG
  /**
     returns the transformation of a vector from the child
     reference frame to the parent reference frame
  */
  virtual const IMP::algebra::Transformation3D&
    get_transformation_child_to_parent() const;

#endif

  RigidBody  get_parent_node() const
  { return parent_; }

  RigidBody get_child_node() const
  { return child_; }

 protected:

  /***************** setter methods: ***************/

  void set_owner_kf(KinematicForest* kf) {
    owner_kf_ = kf;
  }

  /**
     Sets the transfromation from parent to child reference frame
     (without any checks that internal coords are updated, and without
      marking the owner internal coords as changed)
  */
  void set_transformation_child_to_parent_no_checks
    (IMP::algebra::Transformation3D transformation) {
    tr_child_to_parent_ = transformation;
  }


  /**************** general methods: **************/

  /**
     Updates the reference frame of the rigid body directly downstream
     of this joint

    // TODO: validate test - must be buggy :)
  */
  virtual void
    update_child_node_reference_frame() const;

  /**
     Updates the joint transformation based on external coordinates
     of 'witness' particles.

     @note It is assumed that external coordinates are updated before
           calling this function.
     @note Witness particles do not necessarily belong to the child or
           parent rigid bodes.
   */
  virtual void update_joint_from_cartesian_witnesses() = 0;

private:
    RigidBody parent_;
    RigidBody child_;
    IMP::algebra::Transformation3D tr_child_to_parent_;
    KinematicForest* owner_kf_; // the tree that manages updates to this joint
};

IMP_OBJECTS(Joint, Joints);

IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_JOINT_H */
