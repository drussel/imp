/**
 *  \file core/rigid_bodies.h
 *  \brief functionality for defining rigid bodies
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 */

#ifndef IMPCORE_RIGID_BODIES_H
#define IMPCORE_RIGID_BODIES_H

#include "core_config.h"
#include "internal/rigid_bodies.h"

#include "XYZ.h"
#include "XYZR.h"
#include <IMP/SingletonContainer.h>
#include <IMP/SingletonModifier.h>
#include <IMP/Refiner.h>
#include <IMP/algebra/Vector3D.h>
#include <IMP/algebra/Rotation3D.h>
#include <IMP/algebra/ReferenceFrame3D.h>

IMPCORE_BEGIN_NAMESPACE


class RigidMember;
class RigidBody;
IMP_DECORATORS(RigidMember,RigidMembers, XYZs);
IMP_DECORATORS(RigidBody,RigidBodies, XYZs);

//! A decorator for a rigid body
/** A rigid body particle describes a set of particles, known
    as the members, which move rigidly together. Since the
    members are simply a set of particles which move together
    they don't (necessarily) define a shape. For example,
    the members of the rigid body made from a molecular hierarchy
    would include particles corresponding to intermediate levels
    of the hierarchy. As a result, methods
    that use rigid bodies also take a Refiner
    to map from the rigid body to the set of particles
    defining the geometry of interest.

    The initial orientation of the rigid body is computed from
    the coordinates, masses and radii of the particles
    passed to the constructor, based on diagonalizing the
    inertial tensor (which is not stored, currently).

    A rigid body stores the a set of local coordinates for each
    member and a algebra::ReferenceFrame3D in which those local
    coordinates are expressed.

    It is often desirable to randomize the orientation of a rigid
    body:
    \pythonexample{randomize_rigid_body}

    \usesconstraint

    \see RigidMember
    \see RigidBodyMover
    \see RigidClosePairsFinder
    \see RigidBodyDistancePairScore
 */
class IMPCOREEXPORT RigidBody: public XYZ {
  //! Return the location of a member particle given the current position
  /** This method computes the coordinates of p given its internal coordinates
      and the current position and orientation of the rigid body.
   */
  algebra::VectorD<3> get_coordinates(RigidMember p) const;

  void add_member_internal(XYZ d,
                           const algebra::ReferenceFrame3D &rf, bool cover);
  static RigidBody internal_setup_particle(Particle *p,
                                           const XYZs &members);
  void on_change();
  IMP_CONSTRAINT_DECORATOR_DECL(RigidBody);
public:

  RigidMembers get_members() const;

  IMP_DECORATOR(RigidBody, XYZ);

  /** Merge several rigid bodies into on.
   */
  static RigidBody setup_particle(Particle *p,
                                  const RigidBodies &o);


  //! Create a new rigid body from a set of particles.
  /** \param[in] p The particle to make into a rigid body
      \param[in] members The particles to use as members of the rigid body

      The initial position and orientation of p is computed, as are the
      relative positions of the member particles. The member particles
      do not already need to be RigidMember particles, only
      XYZ particles.
   */
  static RigidBody setup_particle(Particle *p,
                          const XYZs &members);

  //! Create a new rigid body from a set of particles, specifying the frame
  /** \param[in] p The particle to make into a rigid body
      \param[in] members The particles to use as members of the rigid body
      \param[in] reference_frame The reference frame to use

      The member particles
      do not already need to be RigidMember particles, only
      XYZ particles.
   */
  static RigidBody setup_particle(Particle *p,
                                  const XYZs &members,
                           const algebra::ReferenceFrame3D &reference_frame);


  //! Create a rigid body based on members of another one
  /** This function creates a rigid body that is part of another
      one. The member particles passed must be part of the other
      rigid body and the created rigid body is added
      to that one as a member. The purpose of this method
      is to, for example, define a rigid body for part of
      a large molecule that is also rigid.

      The passed members do not become members of this rigid
      body, as there would be no point.
  */
  static RigidBody setup_particle(Particle *p,
                                  RigidBody other,
                                const RigidMembers &members);

  //! Make the rigid body no longer rigid.
  static void teardown_particle(RigidBody rb);

  ~RigidBody();

  //!Return true of the particle is a rigid body
  static bool particle_is_instance(Particle *p) {
    return internal::get_has_required_attributes_for_body(p);
  }

  // swig doesn't support using, so the method is wrapped
  //! Get the coordinates of the particle
  algebra::VectorD<3> get_coordinates() const {
    return XYZ::get_coordinates();
  }

  //! Get the reference frame for the local coordinates
  IMP::algebra::ReferenceFrame3D get_reference_frame() const;

  //! Set the current reference frame
  /** All members of the rigid body will have their coordinates updated
      immediately.
      \see IMP::core::transform(RigidBody,const algebra::Transformation3D&)
      \see lazy_set_reference_frame()
   */
  void set_reference_frame(const IMP::algebra::ReferenceFrame3D &tr);

  //! Change the reference, delay updating the members until evaluate
  /** See set_transformation()
   */
  void lazy_set_reference_frame(const IMP::algebra::ReferenceFrame3D &tr);

#ifndef IMP_DOXYGEN
  void set_transformation(const algebra::Transformation3D &tr) {
    set_reference_frame(algebra::ReferenceFrame3D(tr));
  }
  void lazy_set_transformation(const algebra::Transformation3D &tr) {
    lazy_set_reference_frame(algebra::ReferenceFrame3D(tr));
  }
  algebra::Transformation3D get_transformation() {
    return get_reference_frame().get_transformation_to();
  }

  /** This takes a cartesian derivative, and a location in internal coordinates.

      It is currently hidden since the function signature is highly ambiguous.
   */
  void add_to_derivatives(const algebra::Vector3D &derivative,
                          const algebra::Vector3D &local_location,
                          DerivativeAccumulator &da);

  algebra::Vector3D get_torque() const;
#endif

  bool get_coordinates_are_optimized() const;

  //! Set whether the rigid body coordinates are optimized
  void set_coordinates_are_optimized(bool tf);

  //! Normalized the quaternion
  void normalize_rotation();

  //! Update the coordinates of the members
  void update_members();

  //! Get the derivatives of the quaternion
  algebra::VectorD<4> get_rotational_derivatives() const;

  unsigned int get_number_of_members() const;

  RigidMember get_member(unsigned int i) const;

  //! Add a particle as a member
  void add_member(XYZ d);

  //! Add another rigid body
  /** Rigid bodies can be tied together so that one updates
      the other.
  */
  void add_member(RigidBody o);
};

IMP_OUTPUT_OPERATOR(RigidBody);


/** It is often useful to store precalculated properties of the rigid body
    for later use. These need to be cleared out when the rigid body changes.
    To make sure this happens, register the key here.
*/
void IMPCOREEXPORT add_rigid_body_cache_key(ObjectKey k);


//! A decorator for a particle that is part of a rigid body
/**
   \see RigidBody
 */
class IMPCOREEXPORT RigidMember: public XYZ {
 public:
  IMP_DECORATOR(RigidMember, XYZ);

  RigidBody get_rigid_body() const;

  //! Return the current orientation of the body
  algebra::VectorD<3> get_internal_coordinates() const {
    return algebra::VectorD<3>(get_particle()
                    ->get_value(internal::rigid_body_data().child_keys_[0]),
                    get_particle()
                    ->get_value(internal::rigid_body_data().child_keys_[1]),
                    get_particle()
                    ->get_value(internal::rigid_body_data().child_keys_[2]));
  }

  //! set the internal coordinates for this member
  void set_internal_coordinates(const algebra::VectorD<3> &v) const {
    get_particle()->set_value(internal::rigid_body_data().child_keys_[0],
                              v[0]);
    get_particle()->set_value(internal::rigid_body_data().child_keys_[1],
                              v[1]);
    get_particle()->set_value(internal::rigid_body_data().child_keys_[2],
                              v[2]);
  }
  //! Member must be a rigid body
  void set_internal_transformation(const  algebra::Transformation3D& v) {
    IMP_USAGE_CHECK(
 get_particle()->has_attribute(internal::rigid_body_data().lquaternion_[0]),
         "Can only set the internal transformation if member is"
         << " a rigid body itself.");
    get_particle()->set_value(internal::rigid_body_data().child_keys_[0],
                              v.get_translation()[0]);
    get_particle()->set_value(internal::rigid_body_data().child_keys_[1],
                              v.get_translation()[1]);
    get_particle()->set_value(internal::rigid_body_data().child_keys_[2],
                              v.get_translation()[2]);

    get_particle()->set_value(internal::rigid_body_data().lquaternion_[0],
                              v.get_rotation().get_quaternion()[0]);
    get_particle()->set_value(internal::rigid_body_data().lquaternion_[1],
                              v.get_rotation().get_quaternion()[1]);
    get_particle()->set_value(internal::rigid_body_data().lquaternion_[2],
                              v.get_rotation().get_quaternion()[2]);
    get_particle()->set_value(internal::rigid_body_data().lquaternion_[3],
                              v.get_rotation().get_quaternion()[3]);
  }

  algebra::Transformation3D get_internal_transformation() const {
    IMP_USAGE_CHECK(
     get_particle()->has_attribute(internal::rigid_body_data().lquaternion_[0]),
     "Can only set the internal transformation if member is a "
     << "rigid body itself.");
    algebra::VectorD<3>
      tr(get_particle()->get_value(internal::rigid_body_data().child_keys_[0]),
         get_particle()->get_value(internal::rigid_body_data().child_keys_[1]),
         get_particle()->get_value(internal::rigid_body_data().child_keys_[2]));
    algebra::Rotation3D
      rot(get_particle()->get_value(internal::rigid_body_data()
                                    .lquaternion_[0]),
          get_particle()->get_value(internal::rigid_body_data()
                                    .lquaternion_[1]),
          get_particle()->get_value(internal::rigid_body_data()
                                    .lquaternion_[2]),
          get_particle()->get_value(internal::rigid_body_data()
                                    .lquaternion_[3]));
    return algebra::Transformation3D(rot, tr);
  }

  //! XYZ::set_coordiantes()
  // this is here since swig does like using statements
  void set_coordinates(const algebra::VectorD<3> &center) {
    XYZ::set_coordinates(center);
  }
#ifndef IMP_DOXYGEN
  //! Set the coordinates from the internal coordinates
  void set_coordinates(const algebra::Transformation3D &tr) {
    set_coordinates(tr.get_transformed(get_internal_coordinates()));
  }
#endif
  ~RigidMember();

  //! return true if it is a rigid member
  static bool particle_is_instance(Particle *p) {
    return internal::get_has_required_attributes_for_member(p);
  }
};

IMP_OUTPUT_OPERATOR(RigidMember);

#ifndef IMP_DOXYGEN

class IMPCOREEXPORT RigidMembersRefiner: public Refiner {
 public:
  RigidMembersRefiner():Refiner("RigidMembersRefiner%d"){}
  IMP_SIMPLE_REFINER(RigidMembersRefiner);
};

namespace internal {
  IMPCOREEXPORT RigidMembersRefiner* get_rigid_members_refiner();
}
#endif

//! Transform a rigid body
/** The transformation is applied current conformation of the rigid
    body, as opposed to replacing the current conformation, as in
    RigidBody::set_transformation().

    \relatesalso RigidBody
     algebra::Transformation3D
*/
inline void transform(RigidBody a, const algebra::Transformation3D&tr) {
  a.set_reference_frame(get_transformed(a.get_reference_frame(), tr));
}

IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_RIGID_BODIES_H */
