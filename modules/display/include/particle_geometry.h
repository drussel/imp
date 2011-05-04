/**
 *  \file particle_geometry.h
 *  \brief Represent an XYZR particle with a sphere
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 */

#ifndef IMPDISPLAY_PARTICLE_GEOMETRY_H
#define IMPDISPLAY_PARTICLE_GEOMETRY_H

#include "display_config.h"
#include "display_macros.h"
#include "Colored.h"
#include <IMP/SingletonContainer.h>
#include <IMP/PairContainer.h>
#include <IMP/core/XYZR.h>
#include <IMP/atom/bond_decorators.h>
#include <IMP/atom/Hierarchy.h>
#include <IMP/display/geometry.h>
#include <IMP/core/rigid_bodies.h>
#include <IMP/SingletonScore.h>
#include <IMP/atom/Selection.h>

IMPDISPLAY_BEGIN_NAMESPACE

//! A base class for geometry contained in particles
/** */
class IMPDISPLAYEXPORT SingletonGeometry: public Geometry
{
  IMP::Pointer<Particle> p_;
public:
  SingletonGeometry(Particle *p);

  bool get_has_color() const {
    return Geometry::get_has_color()
      || Colored::particle_is_instance(p_);
  }

  Color get_color() const {
    if (Geometry::get_has_color()) {
      return Geometry::get_color();
    } else {
      return Colored(p_).get_color();
    }
  }

  Particle *get_particle() const {
    return p_;
  }

  virtual ~SingletonGeometry(){}
};



//! A base class for geometry from a set of particles
/**
 */
class IMPDISPLAYEXPORT SingletonsGeometry: public Geometry
{
  IMP::internal::OwnerPointer<SingletonContainer> sc_;
public:
  SingletonsGeometry(SingletonContainer *pc, Color c);
  SingletonsGeometry(SingletonContainer *pc);

  SingletonContainer *get_container() const {
    return sc_;
  }

  virtual ~SingletonsGeometry(){}
};


//! A base class for geometry contained in particles
/** */
class IMPDISPLAYEXPORT PairGeometry: public Geometry
{
  IMP::Pointer<Particle> p0_, p1_;
public:
  PairGeometry(const ParticlePair &pp);

  bool get_has_color() const {
    return Geometry::get_has_color()
      || Colored::particle_is_instance(p0_)
      || Colored::particle_is_instance(p1_);
  }

  Color get_color() const {
    if (Geometry::get_has_color()) {
      return Geometry::get_color();
    } else if (Colored::particle_is_instance(p0_))
      return Colored(p0_).get_color();
    else return Colored(p1_).get_color();
  }

  ParticlePair get_particle_pair() const {
    return ParticlePair(p0_, p1_);
  }

  virtual ~PairGeometry(){}
};


//! A base class for geometry from a set of particles
/**
 */
class IMPDISPLAYEXPORT PairsGeometry: public Geometry
{
  IMP::internal::OwnerPointer<PairContainer> sc_;
public:
  PairsGeometry(PairContainer *pc, Color c);
  PairsGeometry(PairContainer *pc);

  PairContainer *get_container() const {
    return sc_;
  }

  virtual ~PairsGeometry(){}
};


/** \class XYZRGeometry
    \brief Display an IMP::core::XYZR particle as a ball.

    \class XYZRsGeometry
    \brief Display an IMP::SingletonContainer of IMP::core::XYZR particles
    as balls.
*/
IMP_PARTICLE_GEOMETRY(XYZR, core::XYZR,
 {
   SphereGeometry *g= new SphereGeometry(d.get_sphere());
   if (Colored::particle_is_instance(d)) {
     g->set_color(Colored(d).get_color());
   }
   ret.push_back(g);
  });


/** \class BondGeometry
    \brief Display an IMP::atom::Bond particle as a segment.

    \class BondsGeometry
    \brief Display an IMP::SingletonContainer of IMP::atom::Bond particles
    as segments.
*/
IMP_PARTICLE_GEOMETRY(Bond, atom::Bond,{
    atom::Bonded ep0=  d.get_bonded(0);
    core::XYZ epi0(ep0.get_particle());
    atom::Bonded ep1=  d.get_bonded(1);
    core::XYZ epi1(ep1.get_particle());
    algebra::Segment3D s(epi0.get_coordinates(),
                         epi1.get_coordinates());
    Geometry *g= new SegmentGeometry(s);
    ret.push_back(g);
  });


IMP_PARTICLE_GEOMETRY(XYZDerivative, core::XYZ, {
    algebra::Segment3D s(d.get_coordinates(),
                         d.get_coordinates()+d.get_derivatives());
    Geometry *g= new SegmentGeometry(s);
    ret.push_back(g);
  });

IMP_PARTICLE_GEOMETRY(RigidBodyDerivative, core::RigidBody, {
    Particles ms=d.get_members();
    algebra::Transformation3D otr
      = d.get_reference_frame().get_transformation_to();
    algebra::VectorD<4> rderiv= d.get_rotational_derivatives();
    algebra::VectorD<3> tderiv= d.get_derivatives();
    algebra::VectorD<4> rot = otr.get_rotation().get_quaternion();
    IMP_LOG(TERSE, "Old rotation was " << rot << std::endl);
    Float scale=.1;
    algebra::VectorD<4> dv=rderiv;
    if (dv.get_squared_magnitude() > 0.00001) {
      dv= scale*dv.get_unit_vector();
    }
    rot+= dv;
    rot= rot.get_unit_vector();
    algebra::Rotation3D r(rot[0], rot[1], rot[2], rot[3]);
    IMP_LOG(TERSE, "Derivative was " << tderiv << std::endl);
    IMP_LOG(TERSE, "New rotation is " << rot << std::endl);
    FloatRange xr= d.get_particle()->get_model()
      ->get_range(core::XYZ::get_xyz_keys()[0]);
    Float wid= xr.second-xr.first;
    algebra::VectorD<3> stderiv= scale*tderiv*wid;
    algebra::Transformation3D ntr(algebra::Rotation3D(rot[0], rot[1],
                                                      rot[2], rot[3]),
                                  stderiv+otr.get_translation());
    for (unsigned int i=0; i< ms.size(); ++i) {
      core::RigidMember dm(ms[i]);
      SegmentGeometry *tr
        = new SegmentGeometry(algebra::Segment3D(dm.get_coordinates(),
                                                 dm.get_coordinates()+tderiv),
                              /*xyzcolor_*/
                              Color(1,0,0));
      ret.push_back(tr);
      algebra::VectorD<3> ic= r.get_rotated(dm.get_internal_coordinates())
      + d.get_coordinates();
      SegmentGeometry *rtr
        = new SegmentGeometry(algebra::Segment3D(dm.get_coordinates(),
                                                 ic),
                              Color(0,1,0));
      ret.push_back(rtr);
      SegmentGeometry *nrtr
        = new SegmentGeometry(algebra::Segment3D(dm.get_coordinates(),
                      ntr.get_transformed(dm.get_internal_coordinates())),
                              Color(0,0,1));
      ret.push_back(nrtr);
    }
  });


/** \class BondGeometry
    \brief Display an IMP::atom::Bond particle as a segment.

    \class BondsGeometry
    \brief Display an IMP::SingletonContainer of IMP::atom::Bond particles
    as segments.
*/
IMP_PARTICLE_PAIR_GEOMETRY(EdgePair, core::XYZ, {
    ret.push_back(
    new SegmentGeometry(algebra::Segment3D(d0.get_coordinates(),
                                           d1.get_coordinates())));
  });


/** \class HierarchyGeometry
    \brief Display an IMP::atom::Hierarchy particle as balls.

    \class HierarchiesGeometry
    \brief Display an IMP::SingletonContainer of IMP::atom::Hierarchy particles
    as balls.
*/
class HierarchyGeometry: public SingletonGeometry {
  double res_;
public:
  HierarchyGeometry(core::Hierarchy d, double resolution=-1):
    SingletonGeometry(d), res_(resolution){}
  Geometries get_components() const {
    Geometries ret;
    atom::Hierarchy d(get_particle());
    atom::Selection sel(d);
    sel.set_target_radius(res_);
    ParticlesTemp ps= sel.get_selected_particles();
    for (unsigned int i=0; i< ps.size(); ++i) {
      IMP_NEW(XYZRGeometry, g, (core::XYZR(ps[i])));
      ret.push_back(g);
    }
    return ret;
  }
  IMP_OBJECT_INLINE(HierarchyGeometry,
                    out <<  atom::Hierarchy(get_particle())<< std::endl;,{});
};
class HierarchiesGeometry: public SingletonsGeometry {
  double res_;
  public:
  HierarchiesGeometry(SingletonContainer* sc, double resolution=-1):
    SingletonsGeometry(sc), res_(resolution){}
  Geometries get_components() const {
    Geometries ret;
    for (unsigned int i=0;
         i< get_container()->get_number_of_particles();
         ++i) {
      IMP_NEW(HierarchyGeometry, g, (get_container()->get_particle(i), res_));
      ret.push_back(g);
    }
    return ret;
  }
  IMP_OBJECT_INLINE(HierarchiesGeometry,
                    out <<  get_container() << std::endl;,{});
};



/** \class SelectionGeometry
    \brief Display an IMP::atom::Selection.
*/
class SelectionGeometry: public Geometry {
  atom::Selection res_;
public:
  SelectionGeometry(atom::Selection d,
                    std::string name="Selection"):
    Geometry(name), res_(d) {}
  Geometries get_components() const {
    Geometries ret;
    ParticlesTemp ps= res_.get_selected_particles();
    for (unsigned int i=0; i< ps.size(); ++i) {
      IMP_NEW(HierarchyGeometry, g, (atom::Hierarchy(ps[i])));
      ret.push_back(g);
      ret.back()->set_name(get_name());
      if (get_has_color()) {
        ret.back()->set_color(get_color());
      }
    }
    return ret;
  }
  IMP_OBJECT_INLINE(SelectionGeometry,IMP_UNUSED(out);,);
};

IMPDISPLAY_END_NAMESPACE

#endif  /* IMPDISPLAY_PARTICLE_GEOMETRY_H */
