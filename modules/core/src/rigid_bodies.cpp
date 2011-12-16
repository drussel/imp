/**
 *  \file rigid_bodies.cpp
 *  \brief Support for rigid bodies.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 *
 */

#include "IMP/core/rigid_bodies.h"
#include "IMP/core/SingletonConstraint.h"
#include <IMP/algebra/Vector3D.h>
#include <IMP/algebra/internal/tnt_array2d.h>
#include <IMP/algebra/internal/tnt_array2d_utils.h>
#include <IMP/algebra/internal/jama_eig.h>
#include <IMP/algebra/geometric_alignment.h>
#include "IMP/generic.h"
#include <IMP/SingletonContainer.h>
#include <IMP/core/FixedRefiner.h>
#include <IMP/core/internal/rigid_body_tree.h>

IMPCORE_BEGIN_INTERNAL_NAMESPACE
const RigidBodyData &rigid_body_data() {
  static RigidBodyData rbd;
  return rbd;
}
IMPCORE_END_INTERNAL_NAMESPACE
IMPCORE_BEGIN_NAMESPACE

namespace {
  ObjectKeys cache_keys;
}
void add_rigid_body_cache_key(ObjectKey k) {
  if (!std::binary_search(cache_keys.begin(), cache_keys.end(), k)) {
    cache_keys.push_back(k);
    std::sort(cache_keys.begin(), cache_keys.end());
  }
}

namespace {



  /*ParticlesTemp get_rigid_body_used_particles(Particle *p) {
    RigidBody b(p);
    unsigned int n=b.get_number_of_members();
    ParticlesTemp ret(1+n);
    ret[0]=p;
    for (unsigned int i=0; i< n; ++i) {
      ret[i+1]= b.get_member(i);
    }
    return ret;
    }*/

  //! Accumulate the derivatives from the refined particles in the rigid body
  /** You can
      use the setup_rigid_bodies and setup_rigid_body methods instead of
      creating these objects yourself.
      \see setup_rigid_bodies
      \see setup_rigid_body
      \see RigidBody
      \verbinclude rigid_bodies.py
      \see UpdateRigidBodyMembers
  */
  class AccumulateRigidBodyDerivatives:
    public SingletonDerivativeModifier {
  public:
    AccumulateRigidBodyDerivatives(std::string name
                                   ="AccumulateRigidBodyDerivatives%1%"):
        SingletonDerivativeModifier(name){}
    IMP_SINGLETON_DERIVATIVE_MODIFIER(AccumulateRigidBodyDerivatives);
  };


  /** \brief Compute the coordinates of the RigidMember objects bases
      on the orientation.

      This should be applied after evaluate to keep the bodies
      rigid. You can use the setup_rigid_bodies and setup_rigid_body
      methods instead of creating these objects yourself.

      \see setup_rigid_bodies
      \see setup_rigid_body
      \see RigidBody
      \see AccumulateRigidBodyDerivatives */
  class UpdateRigidBodyMembers: public SingletonModifier {
  public:
    UpdateRigidBodyMembers(std::string name
                           = "UpdateRigidBodyMembers%1%"):
        SingletonModifier(name) {}
    IMP_SINGLETON_MODIFIER(UpdateRigidBodyMembers);
  };


 /** \brief Fix the normalization of the rotation term. */
  class NormalizeRotation: public SingletonModifier {
  public:
    NormalizeRotation(std::string name
                      = "NormalizeRotation%1%"):
        SingletonModifier(name){}
    IMP_SINGLETON_MODIFIER(NormalizeRotation);
  };


 /** \brief Fix the normalization of the rotation term. */
  class NullSDM: public SingletonDerivativeModifier {
  public:
    NullSDM(std::string name="NullModifier%1%"):
        SingletonDerivativeModifier(name){}
    IMP_SINGLETON_DERIVATIVE_MODIFIER(NullSDM);
  };



  void AccumulateRigidBodyDerivatives::apply(Particle *p,
                                             DerivativeAccumulator &da) const {
    IMP_OBJECT_LOG;
    RigidBody rb(p);
#if IMP_BUILD < IMP_FAST
    algebra::Vector4D oldderiv;
    algebra::Vector3D oldcartesian= rb.get_derivatives();
    for (unsigned int j=0; j< 4; ++j) {
      oldderiv[j]=rb.get_particle()->get_derivative(internal::rigid_body_data()
                                                    .quaternion_[j]);
    }
#endif
    algebra::Rotation3D rot= rb.get_reference_frame().get_transformation_from()\
      .get_rotation();
    const ParticleIndexes &rbis= rb.get_member_particle_indexes();
    for (unsigned int i=0; i< rbis.size(); ++i) {
      RigidMember d(rb.get_model(), rbis[i]);
      algebra::Vector3D dv= rot*d.get_derivatives();
      rb.add_to_derivatives(dv, d.get_internal_coordinates(), da);
    }
    const ParticleIndexes &rbbis= rb.get_body_member_particle_indexes();
    for (unsigned int i=0; i< rbbis.size(); ++i) {
      RigidMember d(rb.get_model(), rbbis[i]);
      algebra::Vector3D dv= rot*d.get_derivatives();
      rb.add_to_derivatives(dv, d.get_internal_coordinates(), da);
    }
    IMP_LOG(TERSE, "Rigid body derivative is "
            << p->get_derivative(internal::rigid_body_data().quaternion_[0])
            << " "
            << p->get_derivative(internal::rigid_body_data().quaternion_[1])
            << " "
            << p->get_derivative(internal::rigid_body_data().quaternion_[2])
            << " "
            << p->get_derivative(internal::rigid_body_data().quaternion_[3])
            << " and ");

    IMP_LOG(TERSE, "Translation deriv is "
            << static_cast<XYZ>(rb).get_derivatives()
            << "" << std::endl);
    IMP_IF_CHECK(USAGE_AND_INTERNAL) {
      algebra::Rotation3D rot= rb.get_reference_frame()
        .get_transformation_to().get_rotation();
      //IMP_LOG(TERSE, "Accumulating rigid body derivatives" << std::endl);
      algebra::Vector3D v(0,0,0);
      algebra::VectorD<4> q(0,0,0,0);
      for (unsigned int i=0; i< rb.get_number_of_members(); ++i) {
        RigidMember d= rb.get_member(i);
        algebra::Vector3D dv= d.get_derivatives();
        v+=dv;
        //IMP_LOG(TERSE, "Adding " << dv << " to derivative" << std::endl);
        for (unsigned int j=0; j< 4; ++j) {
          algebra::Vector3D v
            = rot.get_derivative(d.get_internal_coordinates(),
                                 j);
          /*IMP_LOG(VERBOSE, "Adding " << dv*v << " to quaternion deriv " << j
            << std::endl);*/
          q[j]+= dv*v;
        }
      }
      for (unsigned int j=0; j< 4; ++j) {
#if IMP_BUILD < IMP_FAST
        double d= rb.get_particle()->get_derivative(internal::rigid_body_data()
                                                    .quaternion_[j])
          - oldderiv[j];
#endif
        IMP_INTERNAL_CHECK(std::abs(d-q[j])< .05*std::abs(d+q[j])+.05,
                           "Derivatives do not match "
                           << oldderiv << ": "
                           << rb.get_particle()
                           ->get_derivative(internal::rigid_body_data()
                                            .quaternion_[0])
                           << " " << rb.get_particle()
                           ->get_derivative(internal::rigid_body_data()
                                            .quaternion_[1])
                           << " " << rb.get_particle()
                           ->get_derivative(internal::rigid_body_data()
                                            .quaternion_[1])
                           << " " << rb.get_particle()
                           ->get_derivative(internal::rigid_body_data()
                                            .quaternion_[2])
                           << ": " << q);
      }
#if IMP_BUILD < IMP_FAST
      algebra::Vector3D deltacartesian= rb.get_derivatives()-oldcartesian;
#endif
      IMP_INTERNAL_CHECK((deltacartesian-v).get_magnitude()
                         < .01*(v+deltacartesian).get_magnitude()+.1,
                         "Cartesian derivatives don't match : "
                         << deltacartesian << " vs " << v);
    }
  }


  IMP_SINGLETON_MODIFIER_FROM_REFINED(AccumulateRigidBodyDerivatives,
                                      internal::get_rigid_members_refiner());


  void UpdateRigidBodyMembers::apply(Particle *p) const {
    RigidBody rb(p);
    rb.update_members();
  }
  ParticlesTemp UpdateRigidBodyMembers::get_input_particles(Particle *p) const {
    return ParticlesTemp(1, p);
  }
  ParticlesTemp
  UpdateRigidBodyMembers::get_output_particles(Particle *p) const {
    IMP_OBJECT_LOG;
    RigidBody rb(p);
    ParticlesTemp ret
      = IMP::internal::get_particle(p->get_model(),
                                    rb.get_member_particle_indexes());
    ParticlesTemp ret2
      = IMP::internal::get_particle(p->get_model(),
                                    rb.get_body_member_particle_indexes());
    ret.insert(ret.end(), ret2.begin(), ret2.end());
    return ret;
  }
  ContainersTemp
  UpdateRigidBodyMembers::get_input_containers(Particle *p) const {
    return ContainersTemp(1,p);
  }
  ContainersTemp
  UpdateRigidBodyMembers::get_output_containers(Particle *) const {
    return ContainersTemp();
  }
  void UpdateRigidBodyMembers::do_show(std::ostream &) const {
  }


  void NormalizeRotation::apply(Particle *p) const {
    RigidBody rb(p);
    rb.normalize_rotation();
  }
  ParticlesTemp NormalizeRotation::get_input_particles(Particle *p) const {
    return ParticlesTemp(1, p);
  }
  ParticlesTemp
  NormalizeRotation::get_output_particles(Particle *p) const {
    return ParticlesTemp(1,p);
  }
  ContainersTemp
  NormalizeRotation::get_input_containers(Particle *) const {
    return ContainersTemp();
  }
  ContainersTemp
  NormalizeRotation::get_output_containers(Particle *) const {
    return ContainersTemp();
  }
  void NormalizeRotation::do_show(std::ostream &) const {
  }

  void NullSDM::apply(Particle *, DerivativeAccumulator &) const {
  }
  ParticlesTemp NullSDM::get_input_particles(Particle *p) const {
    return ParticlesTemp(1,p);
  }
  ParticlesTemp
  NullSDM::get_output_particles(Particle *p) const {
    return ParticlesTemp(1,p);
  }
  ContainersTemp
  NullSDM::get_input_containers(Particle *) const {
    return ContainersTemp();
  }
  ContainersTemp
  NullSDM::get_output_containers(Particle *) const {
    return ContainersTemp();
  }
  void NullSDM::do_show(std::ostream &) const {
  }
}

typedef IMP::algebra::internal::TNT::Array2D<double> Matrix;

namespace {
Matrix compute_I(const XYZs &ds,
                 const algebra::Vector3D &center,
                 const IMP::algebra::Rotation3D &rot) {
  Matrix I(3,3, 0.0);
  for (unsigned int i=0; i< ds.size(); ++i) {
    XYZ cm= ds[i];
    double m=1;
    double r=0;
    algebra::Vector3D cv=rot.get_rotated(cm.get_coordinates()-center);

    Matrix Is(3,3, 0.0);
    for (unsigned int i=0; i<3; ++i) {
      for (unsigned int j=0; j<3; ++j) {
        Is[i][j]= -m*cv[i]*cv[j];
        if (i==j) {
          Is[i][j]+= m*cv.get_squared_magnitude() + .4*m*square(r);
        }
      }
    }
    I+= Is;
  }
  return I;
}
}

void RigidBody::on_change() {
   double md=0;
   {
     const ParticleIndexes& members= get_member_particle_indexes();
     for (unsigned int i=0; i< members.size(); ++i) {
       double cd= (get_coordinates()
                   -XYZ(get_model(),
                        members[i]).get_coordinates()).get_magnitude();
       if (get_model()->get_has_attribute(XYZR::get_radius_key(),
                                          members[i])) {
         cd+= get_model()->get_attribute(XYZR::get_radius_key(),
                                         members[i]);
       }
       md=std::max(cd, md);
     }
   }
   {
     const ParticleIndexes& members= get_body_member_particle_indexes();
     for (unsigned int i=0; i< members.size(); ++i) {
       double cd= (get_coordinates()
                   -XYZ(get_model(),
                        members[i]).get_coordinates()).get_magnitude();
       if (get_model()->get_has_attribute(XYZR::get_radius_key(), members[i])) {
         cd+= get_model()->get_attribute(XYZR::get_radius_key(), members[i]);
       }
       md=std::max(cd, md);
     }
   }
   if (get_particle()->has_attribute(XYZR::get_radius_key())) {
     get_particle()->set_value(XYZR::get_radius_key(), md);
   } else {
     get_particle()->add_attribute(XYZR::get_radius_key(), md);
   }
   for (unsigned int i=0; i< cache_keys.size(); ++i) {
     if (get_particle()
         ->has_attribute(cache_keys[i])) {
       get_particle()->remove_attribute(cache_keys[i]);
     }
   }
   get_particle()->get_model()->reset_dependencies();
}

RigidBody RigidBody::internal_setup_particle(Particle *p,
                                             const XYZs &members) {
  IMP_FUNCTION_LOG;
  IMP_USAGE_CHECK(!internal::
                  get_has_required_attributes_for_body(p->get_model(),
                                                       p->get_index()),
                  "The RigidBody is already set up.");

  XYZs ds;
  IMP_USAGE_CHECK(!members.empty(),
                  "There must be particles to make a rigid body");
  for (unsigned int i=0; i< members.size(); ++i) {
    Particle *mp= members[i];
    IMP_USAGE_CHECK(mp != p, "A rigid body cannot have itself as a member "
                    << p->get_name());
    IMP_USAGE_CHECK(!internal
                    ::get_has_required_attributes_for_member(p->get_model(),
                                                             p->get_index()),
                    "Particle " << p->get_name() << " is already part of "
                    << "a conflicting rigid body");
    ds.push_back(XYZ(mp));
  }

  // compute center of mass
  algebra::Vector3D v(0,0,0);
  Float mass=0;
  for (unsigned int i=0; i< ds.size(); ++i) {
    XYZ cm= ds[i];

    v+= cm.get_coordinates()*1.0 /*cm.get_mass()*/;
    mass+= 1.0 /*cm.get_mass()*/;
  }
  v/= mass;
  IMP_LOG(VERBOSE, "Center of mass is " << v << std::endl);
  // for a sphere 2/5 m r^2 (diagopnal)
  // parallel axis theorem
  // I'ij= Iij+M(v^2delta_ij-vi*vj)
  // compute I
  Matrix I = compute_I(ds, v, IMP::algebra::get_identity_rotation_3d());
  //IMP_LOG(VERBOSE, "Initial I is " << I << std::endl);
  // diagonalize it
  IMP::algebra::internal::JAMA::Eigenvalue<double> eig(I);
  Matrix rm;
  eig.getV(rm);
  if (IMP::algebra::internal::JAMA::determinant(rm) <0) {
    for (unsigned int i=0; i< 3; ++i) {
      for (unsigned int j=0; j< 3; ++j) {
        rm[i][j]= -rm[i][j];
      }
    }
  }
  // use the R as the initial orientation
  IMP::algebra::Rotation3D rot
    = IMP::algebra::get_rotation_from_matrix(rm[0][0], rm[0][1], rm[0][2],
                                         rm[1][0], rm[1][1], rm[1][2],
                                         rm[2][0], rm[2][1], rm[2][2]);
  IMP_LOG(VERBOSE, "Initial rotation is " << rot << std::endl);
  IMP::algebra::Rotation3D roti= rot.get_inverse();

  Matrix I2= compute_I(ds, v, roti);
  //IMP_LOG(VERBOSE, I << std::endl);
  //IMP_LOG(VERBOSE, I2 << std::endl);
  internal::add_required_attributes_for_body(p);
  RigidBody d(p);
  d.set_reference_frame(algebra::ReferenceFrame3D(
                                algebra::Transformation3D(rot, v)));
  //IMP_LOG(VERBOSE, "Particle is " << d << std::endl);
  return d;
}

ObjectKey RigidBody::get_constraint_key_0() {
  static ObjectKey ret("rigid body derivatives");
  return ret;
}
ObjectKey RigidBody::get_constraint_key_1() {
  static ObjectKey ret("rigid body angle");
  return ret;
}

void RigidBody::setup_constraints(Particle *p) {
  Pointer<Constraint> c0
    = create_constraint(new UpdateRigidBodyMembers(p->get_name()
                                    +" UpdateMembers %1%"),
                        new AccumulateRigidBodyDerivatives(p->get_name()
                                    +" AccumulateDerivatives %1%"), p,
                         p->get_name()+" positions %1%");
  p->get_model()->add_score_state(c0);
  p->get_model()->add_attribute(get_constraint_key_0(), p->get_index(),
                             c0);
  IMP_INTERNAL_CHECK(c0->get_ref_count()==3,
                     "Wrong number of ref counts after constraint creation: "
                     << c0->get_ref_count());
  Pointer<Constraint> c1
      = create_constraint(new NormalizeRotation(p->get_name()
                                                +" NormalizeRotation %1%"),
                        new NullSDM(p->get_name()
                                    +" Null %1%"),
                        p, p->get_name()+" normalize %1%");
  p->get_model()->add_score_state(c1);
  p->get_model()->add_attribute(get_constraint_key_1(), p->get_index(),
                             c1);
}
void RigidBody::teardown_constraints(Particle *p) {
  Object *oc0= p->get_model()->get_attribute(get_constraint_key_0(),
                                             p->get_index());
  Pointer<Constraint> c0= dynamic_cast<Constraint*>(oc0);
  p->get_model()->remove_score_state(c0);
  p->get_model()->remove_attribute(get_constraint_key_0(), p->get_index());
  Object *oc1= p->get_model()->get_attribute(get_constraint_key_1(),
                                             p->get_index());
  Pointer<Constraint> c1= dynamic_cast<Constraint*>(oc1);
  p->get_model()->remove_score_state(c1);
  p->get_model()->remove_attribute(get_constraint_key_1(), p->get_index());
}

RigidBody RigidBody::setup_particle(Particle *p,
                                    RigidBody,
                                    const RigidMembers &rms) {
  RigidBody ret=internal_setup_particle(p, rms);
  rms[0].get_rigid_body().add_member(ret);
  setup_constraints(p);
  ret.on_change();
  return ret;
}


RigidBody RigidBody::setup_particle(Particle *p,
                                    const XYZs &members){
  RigidBody d=internal_setup_particle(p, members);
  for (unsigned int i=0; i< members.size(); ++i) {
    d.add_member_internal(members[i], d.get_reference_frame());
    //IMP_LOG(VERBOSE, " " << cm << " | " << std::endl);
  }
  d.on_change();
  IMP_IF_CHECK(USAGE_AND_INTERNAL) {
    RigidMembers ds(members);
    for (unsigned int i=0; i< ds.size(); ++i) {
      RigidMember cm= RigidMember(ds[i]);
      algebra::Vector3D v= cm.get_coordinates();
      algebra::Vector3D nv= d.get_coordinates(cm);
      IMP_INTERNAL_CHECK((v-nv).get_squared_magnitude() < .1,
                         "Bad initial orientation "
                         << d.get_reference_frame() << std::endl
                         << v << std::endl
                         << nv);
    }
  }
  setup_constraints(p);
  return d;
}


RigidBody RigidBody::setup_particle(Particle *p,
                                    const XYZs &members,
                                    const algebra::ReferenceFrame3D &rf){
  internal::add_required_attributes_for_body(p);
  RigidBody d(p);
  d.set_reference_frame(rf);
  for (unsigned int i=0; i< members.size(); ++i) {
    d.add_member_internal(members[i], d.get_reference_frame());
    //IMP_LOG(VERBOSE, " " << cm << " | " << std::endl);
  }
  d.on_change();
  IMP_IF_CHECK(USAGE_AND_INTERNAL) {
    RigidMembers ds(members);
    for (unsigned int i=0; i< ds.size(); ++i) {
      RigidMember cm= RigidMember(ds[i]);
      algebra::Vector3D v= cm.get_coordinates();
      algebra::Vector3D nv= d.get_coordinates(cm);
      IMP_INTERNAL_CHECK((v-nv).get_squared_magnitude() < .1,
                         "Bad initial orientation "
                         << d.get_reference_frame() << std::endl
                         << v << std::endl
                         << nv);
    }
  }
  setup_constraints(p);
  return d;
}


RigidBody RigidBody::setup_particle(Particle *p,
                                    const RigidBodies &members){
  IMP_FUNCTION_LOG;
  IMP_LOG(VERBOSE, "Creating rigid body from other rigid bodies"<<std::endl);
  IMP_USAGE_CHECK(members.size() > 0, "Must provide members");
  XYZs xmember;
  for (unsigned int i=0; i< members.size(); ++i) {
    RigidMembers m= members[i].get_members();
    for (unsigned int j=0; j< m.size(); ++j) {
      xmember.push_back(m[j]);
    }
  }
  RigidBody d=internal_setup_particle(p, xmember);
  for (unsigned int i=0; i< members.size(); ++i) {
    d.add_member(members[i]);
    //IMP_LOG(VERBOSE, " " << cm << " | " << std::endl);
  }
  d.on_change();
  setup_constraints(p);
  return d;
}

RigidBody RigidBody::setup_particle(Particle *p,
                                    const algebra::ReferenceFrame3D &rf) {
  internal::add_required_attributes_for_body(p);
  RigidBody d(p);
  d.set_reference_frame(rf);
  setup_constraints(p);
  return d;
}


void RigidBody::teardown_particle(RigidBody rb) {
  IMP_FUNCTION_LOG;
  // clear caches
  rb.on_change();
  {
    const ParticleIndexes& members= rb.get_member_particle_indexes();
    for (unsigned int i=0; i < members.size(); ++i) {
      RigidMember rm(rb.get_model(), members[i]);
      internal::remove_required_attributes_for_member(rm);
    }
  }
  {
    const ParticleIndexes& members= rb.get_body_member_particle_indexes();
    for (unsigned int i=0; i < members.size(); ++i) {
      RigidMember rm(rb.get_model(), members[i]);
      internal::remove_required_attributes_for_body_member(rm);
    }
  }
  teardown_constraints(rb.get_particle());
  internal::remove_required_attributes_for_body(rb.get_particle());
}

void
RigidBody::normalize_rotation() {
  algebra::VectorD<4>
    v(get_particle()->get_value(internal::rigid_body_data().quaternion_[0]),
      get_particle()->get_value(internal::rigid_body_data().quaternion_[1]),
      get_particle()->get_value(internal::rigid_body_data().quaternion_[2]),
      get_particle()->get_value(internal::rigid_body_data().quaternion_[3]));
  //IMP_LOG(TERSE, "Rotation was " << v << std::endl);
  if (v.get_squared_magnitude() >0){
    v= v.get_unit_vector();
  } else {
    v= algebra::VectorD<4>(1,0,0,0);
  }
  //IMP_LOG(TERSE, "Rotation is " << v << std::endl);
  get_particle()->set_value(internal::rigid_body_data().quaternion_[0], v[0]);
  get_particle()->set_value(internal::rigid_body_data().quaternion_[1], v[1]);
  get_particle()->set_value(internal::rigid_body_data().quaternion_[2], v[2]);
  get_particle()->set_value(internal::rigid_body_data().quaternion_[3], v[3]);

  // evil hack
  for (unsigned int i=0; i< 3; ++i) {
    get_model()->set_attribute(internal::rigid_body_data().torque_[0],
                               get_particle_index(), 0);
    get_model()->set_attribute(internal::rigid_body_data().torque_[1],
                               get_particle_index(), 0);
    get_model()->set_attribute(internal::rigid_body_data().torque_[2],
                               get_particle_index(), 0);
  }
}


void RigidBody::update_members() {
  algebra::Transformation3D tr= get_reference_frame().get_transformation_to();
  {
    const ParticleIndexes&members= get_member_particle_indexes();
    for (unsigned int i=0; i< members.size(); ++i) {
      RigidMember rm(get_model(), members[i]);
      rm.set_coordinates(tr.get_transformed(rm.get_internal_coordinates()));
    }
  }
  {
    const ParticleIndexes&members= get_body_member_particle_indexes();
    for (unsigned int i=0; i< members.size(); ++i) {
      RigidBody rb(get_model(), members[i]);
      rb.set_reference_frame_lazy(algebra::ReferenceFrame3D(tr
                                                *RigidMember(get_model(),
                             members[i]).get_internal_transformation()));
    }
  }
}


IMP::algebra::ReferenceFrame3D
RigidBody::get_reference_frame() const {
  algebra::VectorD<4>
      v(get_particle()->get_value(internal::rigid_body_data().quaternion_[0]),
        get_particle()->get_value(internal::rigid_body_data().quaternion_[1]),
        get_particle()->get_value(internal::rigid_body_data().quaternion_[2]),
        get_particle()->get_value(internal::rigid_body_data().quaternion_[3]));
  if (v.get_squared_magnitude() > 0){
      v = v.get_unit_vector();
  } else {
      v = algebra::VectorD<4>(1,0,0,0);
  }
  IMP::algebra::Rotation3D rot(v[0], v[1], v[2], v[3]);
  return algebra::ReferenceFrame3D(algebra::Transformation3D(rot,
                                                           get_coordinates()));
}

RigidMembers
RigidBody::get_members() const {
  RigidMembers ret;
  {
    ParticleIndexes pis= get_member_particle_indexes();
    for (unsigned int i=0; i< pis.size(); ++i) {
      ret.push_back(RigidMember(get_model(), pis[i]));
    }
  }
  {
   ParticleIndexes pis= get_body_member_particle_indexes();
   for (unsigned int i=0; i< pis.size(); ++i) {
     ret.push_back(RigidMember(get_model(), pis[i]));
    }
  }
  return ret;
}




void RigidBody::add_member(XYZ d) {
  add_member_internal(d, get_reference_frame());
  on_change();
}

void RigidBody::add_member_internal(XYZ d,
                                    const algebra::ReferenceFrame3D &ref) {
  internal::add_required_attributes_for_member(d, get_particle());
  RigidMember cm(d);
  if (get_model()->get_has_attribute(internal::rigid_body_data().members_,
                                     get_particle_index())) {
    ParticleIndexes members
      =get_model()->get_attribute(internal::rigid_body_data().members_,
                                  get_particle_index());
    members.push_back(d.get_particle_index());
    get_model()->set_attribute(internal::rigid_body_data().members_,
                               get_particle_index(), members);
  } else {
    get_model()->add_attribute(internal::rigid_body_data().members_,
                               get_particle_index(),
                               ParticleIndexes(1, d.get_particle_index()));
  }
  algebra::Vector3D lc=ref.get_local_coordinates(d.get_coordinates());
  cm.set_internal_coordinates(lc);
}

void RigidBody::add_member(RigidBody d) {
  algebra::ReferenceFrame3D r= get_reference_frame();
  internal::add_required_attributes_for_body_member(d, get_particle());
  RigidMember cm(d);
  if (get_model()->get_has_attribute(internal::rigid_body_data().body_members_,
                                     get_particle_index())) {
    ParticleIndexes members
      =get_model()->get_attribute(internal::rigid_body_data().body_members_,
                                                       get_particle_index());
    members.push_back(d.get_particle_index());
    get_model()->set_attribute(internal::rigid_body_data().body_members_,
                               get_particle_index(), members);
  } else {
    get_model()->add_attribute(internal::rigid_body_data().body_members_,
                               get_particle_index(),
                               ParticleIndexes(1, d.get_particle_index()));
  }
  // want tr*ltr= btr, so ltr= tr-1*btr
  algebra::Transformation3D tr
    =r.get_transformation_from()
    *d.get_reference_frame().get_transformation_to();
  cm.set_internal_transformation(tr);
  on_change();
}

void RigidBody::set_log_level(LogLevel l) {
  Particle *p= get_particle();
  p->set_log_level(l);
  Object *oc0= p->get_model()->get_attribute(get_constraint_key_0(),
                                             p->get_index());
  oc0->set_log_level(l);
  Object *oc1= p->get_model()->get_attribute(get_constraint_key_1(),
                                             p->get_index());
  oc1->set_log_level(l);
}

algebra::VectorD<4> RigidBody::get_rotational_derivatives() const {
  algebra::VectorD<4>
    v(get_particle()
      ->get_derivative(internal::rigid_body_data().quaternion_[0]),
      get_particle()
      ->get_derivative(internal::rigid_body_data().quaternion_[1]),
      get_particle()
      ->get_derivative(internal::rigid_body_data().quaternion_[2]),
      get_particle()
      ->get_derivative(internal::rigid_body_data().quaternion_[3]));
  return v;
}

bool RigidBody::get_coordinates_are_optimized() const {
  for (unsigned int i=0; i< 4; ++i) {
    if(!get_particle()
       ->get_is_optimized(internal::rigid_body_data().quaternion_[i]))
      return false;
  }
  return XYZ::get_coordinates_are_optimized();
}

void RigidBody::set_coordinates_are_optimized(bool tf) {
  const bool body=tf;
  const bool member=false;
  for (unsigned int i=0; i< 4; ++i) {
    get_particle()->set_is_optimized(internal::rigid_body_data().quaternion_[i],
                                     body);
  }
  XYZ::set_coordinates_are_optimized(body);
  for (unsigned int i=0; i< get_number_of_members(); ++i) {
    get_member(i).set_coordinates_are_optimized(member);
  }
}

RigidMember RigidBody::get_member(unsigned int i) const {
  IMP_USAGE_CHECK(i < get_number_of_members(),
                  "Out of range member requested: " << i
                  << " of " << get_number_of_members());
  unsigned int sz=get_member_particle_indexes().size();
  if (i < sz) {
    return RigidMember(get_model(), get_member_particle_indexes()[i]);
  } else {
    return RigidMember(get_model(),
                       get_body_member_particle_indexes()[i-sz]);
  }
}

algebra::Vector3D RigidBody::get_coordinates(RigidMember p)
  const {
  algebra::Vector3D lp= p.get_internal_coordinates();
  return get_reference_frame().get_global_coordinates(lp);
}

void RigidBody
::set_reference_frame_lazy(const IMP::algebra::ReferenceFrame3D &tr) {
  algebra::VectorD<4> v
    = tr.get_transformation_to().get_rotation().get_quaternion();
  get_particle()->set_value(internal::rigid_body_data().quaternion_[0], v[0]);
  get_particle()->set_value(internal::rigid_body_data().quaternion_[1], v[1]);
  get_particle()->set_value(internal::rigid_body_data().quaternion_[2], v[2]);
  get_particle()->set_value(internal::rigid_body_data().quaternion_[3], v[3]);
  set_coordinates(tr.get_transformation_to().get_translation());
}

void RigidBody
  ::set_reference_frame(const IMP::algebra::ReferenceFrame3D &tr) {
  set_reference_frame_lazy(tr);
  {
    const ParticleIndexes &members= get_member_particle_indexes();
    for (unsigned int i=0; i< members.size(); ++i) {
      XYZ(get_model(), members[i])
        .set_coordinates(tr.get_global_coordinates(RigidMember(get_model(),
                                                               members[i])
                                             .get_internal_coordinates()));
    }
  }
  {
    const ParticleIndexes &members= get_body_member_particle_indexes();
    for (unsigned int i=0; i< members.size(); ++i) {
      XYZ(get_model(), members[i])
        .set_coordinates(tr.get_global_coordinates(RigidMember(get_model(),
                                                               members[i])
                                             .get_internal_coordinates()));
    }
  }
}

algebra::Vector3D RigidBody::get_torque() const {
  algebra::Vector3D ret;
  for (unsigned int i=0; i< 3; ++i) {
    ret[i]
      =get_particle()->get_derivative(internal::rigid_body_data().torque_[i]);
  }
  return ret;
}

void RigidBody::add_to_derivatives(const algebra::Vector3D &deriv_local,
                                   const algebra::Vector3D &local,
                                   DerivativeAccumulator &da) {
  algebra::Rotation3D rot= get_reference_frame()
    .get_transformation_to().get_rotation();
  const algebra::Vector3D deriv_global= rot*deriv_local;
  //IMP_LOG(TERSE, "Accumulating rigid body derivatives" << std::endl);
  algebra::VectorD<4> q(0,0,0,0);
  for (unsigned int j=0; j< 4; ++j) {
    algebra::Vector3D v= rot.get_derivative(local, j);
    q[j]= deriv_global*v;
  }
  XYZ::add_to_derivatives(deriv_global, da);
  for (unsigned int j=0; j< 4; ++j) {
    get_model()->add_to_derivative(internal::rigid_body_data()
                                   .quaternion_[j], get_particle_index(),
                                   q[j],da);
  }
  algebra::Vector3D torque= algebra::get_vector_product(local, deriv_local);
  for (unsigned int i=0; i< 3; ++i) {
    get_model()->add_to_derivative(internal::rigid_body_data().torque_[i],
                                   get_particle_index(),
                                   torque[i], da);
  }
}


RigidBody::~RigidBody(){}
RigidMember::~RigidMember(){}






void RigidBody::show(std::ostream &out) const {
  out << "Rigid body " << get_reference_frame()
      << "("
      << get_particle()->get_derivative(internal::rigid_body_data()
                                        .quaternion_[0])
      << " "
      << get_particle()->get_derivative(internal::rigid_body_data()
                                        .quaternion_[1])
      << " "
      << get_particle()->get_derivative(internal::rigid_body_data()
                                        .quaternion_[2])
      << " "
      << get_particle()->get_derivative(internal::rigid_body_data()
                                        .quaternion_[3])
      << ")";
}

void RigidMember::show(std::ostream &out) const {
  out << "Member at " << get_internal_coordinates();
}


RigidBody RigidMember::get_rigid_body() const {
  return RigidBody(get_particle()
                   ->get_value(internal::rigid_body_data().body_));
}





bool RigidMembersRefiner::get_can_refine(Particle *p) const {
  return RigidBody::particle_is_instance(p);
}
unsigned int RigidMembersRefiner::get_number_of_refined(Particle *p) const {
  return RigidBody(p).get_number_of_members();
}
Particle* RigidMembersRefiner::get_refined(Particle *p, unsigned int i) const {
  return RigidBody(p).get_member(i);
}

ParticlesTemp RigidMembersRefiner::get_input_particles(Particle *) const {
  return ParticlesTemp();
}

ContainersTemp RigidMembersRefiner::get_input_containers(Particle *p) const {
  return ContainersTemp(1, p);
}

void RigidMembersRefiner::do_show(std::ostream &) const {
}


namespace internal {
  IMPCOREEXPORT RigidMembersRefiner* get_rigid_members_refiner() {
    static IMP::OwnerPointer<RigidMembersRefiner> pt
      = new RigidMembersRefiner("The rigid members refiner");
    return pt;
  }
}


namespace {
bool check_rigid_body(Particle*p) {
  algebra::Vector4D
    v(p->get_value(internal::rigid_body_data().quaternion_[0]),
      p->get_value(internal::rigid_body_data().quaternion_[1]),
      p->get_value(internal::rigid_body_data().quaternion_[2]),
      p->get_value(internal::rigid_body_data().quaternion_[3])
      );
  if (std::abs(v.get_magnitude()-1) >.1) {
    IMP_THROW("Bad quaternion in rigid body: " << v, ValueException);
  }
  return true;
}
}

IMP_CHECK_DECORATOR(RigidBody, check_rigid_body);

IMPCORE_END_NAMESPACE
