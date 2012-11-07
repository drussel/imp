/**
 *  \file TransformationJoint.h
 *  \brief a kinematic joints between rigid bodies that allows any
 *         transformation
 *  \authors Dina Schneidman, Barak Raveh
 *

 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */

#ifndef IMPCORE_TRANSFORMATION_JOINT_H
#define IMPCORE_TRANSFORMATION_JOINT_H

#include "KinematicNode.h"
#include "Joint.h"
#include <IMP/base/Object.h>
#include <IMP/compatibility/nullptr.h>
#include <IMP/exception.h>
#include <IMP/core/internal/dihedral_helpers.h>
#include <IMP/algebra/Vector3D.h>
#include <IMP/base/check_macros.h>

IMPCORE_BEGIN_NAMESPACE



/********************** TransformationJoint ***************/


/* /\** A joint with a completely non-constrained transformation */
/*     between parent and child nodes reference frames */
/* *\/ */
class  IMPCOREEXPORT
TransformationJoint : public Joint{
 public:
  TransformationJoint(RigidBody parent,
                      RigidBody child);

  /**
     Sets the transfromation from parent to child reference frame,
     in a safe way - that is, after updating all intrnal coordinates
     from external if needed, and marking the owner internal coordinates
     as changed.
  */
  void set_transformation_child_to_parent
    (IMP::algebra::Transformation3D transformation);

 protected:
  /**
     sets the joint transformation to the transformation from the
     child reference frame to the parent reference frame.

     @note It is assumed that external coordinates are updated before
           calling this function.
   */
  virtual void update_joint_from_cartesian_witnesses() ;

};

IMP_OBJECTS(TransformationJoint, TransformationJoints);

IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_TRANSFORMATION_JOINT_H */
