/**
 *  \file atom/bond_decorators.h     \brief Contains decorators for a bond
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPATOM_BOND_DECORATORS_H
#define IMPATOM_BOND_DECORATORS_H

#include "atom_config.h"
#include <IMP/core/internal/graph_base.h>
#include "internal/bond_helpers.h"
#include <IMP/display/particle_geometry.h>
#include <IMP/display/primitive_geometries.h>
#include <IMP/Particle.h>
#include <IMP/Model.h>
#include <IMP/Decorator.h>
#include <IMP/core/XYZ.h>

#include <IMP/internal/IndexingIterator.h>
IMPATOM_BEGIN_NAMESPACE

class Bond;
class Bonded;

/** \defgroup bond Creating and restraining bonds
    A set of classes and functions for manipulating bonds.
 */

//! A decorator for wrapping a particle representing a molecular bond
/**
   As with Atom, the types of bonds will eventually be run-time
   expandible.

   \ingroup bond
   \see Bonded
   \see IMP::atom::get_internal_bonds()
 */
class IMPATOMEXPORT Bond: public Decorator
{
  friend class Bonded;
public:
  IMP_DECORATOR(Bond, Decorator);

  //! Return true if the particle is a bond.
  static bool particle_is_instance(Particle *p) {
    return IMP::core::internal::graph_is_edge(p,
                                  internal::get_bond_data().graph_);
  }

  //! The types a bond can have right now
  enum Type {UNKNOWN=-1,
             NONBIOLOGICAL, SINGLE=1, DOUBLE=2, TRIPLE=3, HYDROGEN,
             SALT, PEPTIDE,
             AMIDE, AROMATIC
            };

  //! Get the atom i of the bond
  /** \param[in] i 0 or 1
      \return Bonded for the atom in question
  */
  Bonded get_bonded(unsigned int i) const ;

  IMP_DECORATOR_GET_SET_OPT(type,
                            internal::get_bond_data().type_, Int, Int,
                            UNKNOWN);

  IMP_DECORATOR_GET_SET_OPT(order,
                            internal::get_bond_data().order_,
                            Int, Int, 1);

  IMP_DECORATOR_GET_SET_OPT(length,
                            internal::get_bond_data().length_, Float,
                            Float, -1);
  IMP_DECORATOR_GET_SET_OPT(stiffness,
                            internal::get_bond_data().stiffness_,
                            Float,
                            Float, -1);

  static FloatKey get_length_key() {
    return internal::get_bond_data().length_;
  }
};



//! A decorator for a particle which has bonds.
/** \ingroup bond
    \see Bond
 */
class IMPATOMEXPORT Bonded: public Decorator
{
  struct GetBond {
    typedef Bond result_type;
    Particle* d_;
    GetBond():d_(nullptr){}
    GetBond(Particle* d): d_(d){}
    Bond operator()(unsigned int i) const;
    bool operator==(const GetBond &o) const {
      return d_== o.d_;
    }
  };
  struct GetBonded {
    typedef Bonded result_type;
    Particle* d_;
    GetBonded():d_(nullptr){}
    GetBonded(Particle* d): d_(d){}
    Bonded operator()(unsigned int i) const;
    bool operator==(const GetBonded &o) const {
      return d_== o.d_;
    }
  };
public:
  IMP_DECORATOR(Bonded, Decorator);
  //! return true if it is a bonded particle
  static bool particle_is_instance(Particle *p) {
    return IMP::core::internal::graph_is_node(p,
                             internal::get_bond_data().graph_);
  }

  static Bonded setup_particle(Particle *p) {
    graph_initialize_node(p, internal::get_bond_data().graph_);
  return Bonded(p);
  }

  /** */
  unsigned int get_number_of_bonds() const {
    return graph_get_number_of_edges(get_particle(),
                                     internal::get_bond_data().graph_);
  }


  //! Get a Bond of the ith bond
  /** \return decorator of the ith child, or throw an exception if there
              is no such bond
  */
  Bond get_bond(unsigned int i) const {
    Particle *p= graph_get_edge(get_particle(), i,
                                     internal::get_bond_data().graph_);
    return Bond(p);
  }

  //! Get a Bonded of the ith bonded particle
  /** \return decorator of the ith child, or throw an exception if there
              is no such bond

      \note I don't entirely like having this here as it duplicates
      functionality available otherwise, however it is such a fundamental
      operation and kind of a pain to write. It also means that we
      could later pull the edge endpoints into the vertex if
      desired.
  */
  Bonded get_bonded(unsigned int i) const {
    Particle *p= graph_get_edge(get_particle(), i,
                                internal::get_bond_data().graph_);
    Bond bd(p);
    if (bd.get_bonded(0) == *this) return bd.get_bonded(1);
    else return bd.get_bonded(0);
  }

  /** @name Iterate through the bonds
      @{
  */
#ifdef IMP_DOXYGEN
  class BondIterator;
#else
  typedef IMP::internal::IndexingIterator<GetBond> BondIterator;
#endif
#ifndef SWIG
  BondIterator bonds_begin() const {
    return BondIterator(GetBond(get_particle()), 0);
  }
  BondIterator bonds_end() const {
    return BondIterator(GetBond(get_particle()), get_number_of_bonds());
  }
#endif
  /** @} */
  /** @name Iterate through the bondeds
      @{
  */
#ifdef IMP_DOXYGEN
  class BondedIterator;
#else
  typedef IMP::internal::IndexingIterator<GetBonded> BondedIterator;
#endif
#ifndef SWIG
  BondedIterator bondeds_begin() const {
    return BondedIterator(GetBonded(get_particle()), 0);
  }
  BondedIterator bondeds_end() const {
    return BondedIterator(GetBonded(get_particle()), get_number_of_bonds());
  }
#endif
  /** @} */
};

IMP_DECORATORS(Bonded,Bondeds, ParticlesTemp);
IMP_DECORATORS(Bond,Bonds, ParticlesTemp);


inline Bonded Bond::get_bonded(unsigned int i) const
{
  Particle *p= graph_get_node(get_particle(), i,
                              internal::get_bond_data().graph_);
  return Bonded(p);
}

#ifndef IMP_DOXYGEN
inline Bond Bonded::GetBond::operator()(unsigned int i)
  const {
  return Bonded(d_).get_bond(i);
}
inline Bonded Bonded::GetBonded::operator()(unsigned int i)
  const {
  return Bonded(d_).get_bonded(i);
}
#endif


//! Connect the two wrapped particles by a bond.
/** \param[in] a The first Particle as a Bonded
    \param[in] b The second Particle as a Bonded
    \param[in] t The type to use for the bond
    \return Bond of the bond Particle.

    \ingroup bond
    \relatesalso Bond
    \relatesalso Bonded
 */
IMPATOMEXPORT
Bond create_bond(Bonded a, Bonded b, Int t);


//! Connect the two wrapped particles by a custom bond.
/** \param[in] a The first Particle as a Bonded
    \param[in] b The second Particle as a Bonded
    \param[in] length The length of the bond.
    \param[in] stiffness The stiffness of the bond.
    \return Bond of the bond Particle.

    \ingroup bond
    \relatesalso Bond
    \relatesalso Bonded
 */
IMPATOMEXPORT
inline Bond create_custom_bond(Bonded a, Bonded b,
                          Float length, Float stiffness=-1) {
  IMP_INTERNAL_CHECK(length>=0, "Length must be positive");
  Bond bd=create_bond(a,b, Bond::NONBIOLOGICAL);
  bd.set_length(length);
  bd.get_particle()->set_name(std::string("bond ")+
                              a.get_particle()->get_name()
                              + " and " + b.get_particle()->get_name());
  if (stiffness >=0) bd.set_stiffness(stiffness);
  return bd;
}


//! Connect the two wrapped particles by a custom bond.
/** Create a bond by copying the information from the othr bond

    \ingroup bond
    \relatesalso Bond
    \relatesalso Bonded
 */
IMPATOMEXPORT
inline Bond create_bond(Bonded a, Bonded b,
                                 Bond o) {
  Bond bd=create_bond(a,b, o.get_type());
  if (o.get_length() > 0) bd.set_length(o.get_length());
  bd.get_particle()->set_name(std::string("bond ")+
                              a.get_particle()->get_name()
                              + " and " + b.get_particle()->get_name());
  if (o.get_stiffness() >=0) bd.set_stiffness(o.get_stiffness());
  return bd;
}

//! Destroy the bond connecting to particles.
/** \param[in] b The bond.
    \ingroup bond
    \relatesalso Bond
    \relatesalso Bonded
 */
IMPATOMEXPORT
void destroy_bond(Bond b);

//! Get the bond between two particles.
/** Bond() is returned if the particles are not bonded.
    \ingroup bond
    \relatesalso Bond
    \relatesalso Bonded
 */
IMPATOMEXPORT
Bond get_bond(Bonded a, Bonded b);




/** \class BondGeometry
    \brief Display an Bond particle as a segment.

    \class BondsGeometry
    \brief Display an IMP::SingletonContainer of Bond particles
    as segments.
*/
IMP_PARTICLE_GEOMETRY(Bond, Bond,{
    atom::Bonded ep0=  d.get_bonded(0);
    core::XYZ epi0(ep0.get_particle());
    atom::Bonded ep1=  d.get_bonded(1);
    core::XYZ epi1(ep1.get_particle());
    algebra::Segment3D s(epi0.get_coordinates(),
                         epi1.get_coordinates());
    display::Geometry *g= new display::SegmentGeometry(s);
    ret.push_back(g);
  });

IMPATOM_END_NAMESPACE

#endif  /* IMPATOM_BOND_DECORATORS_H */
