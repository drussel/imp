/**
 *  \file QuadModifier.h    \brief A Modifier on ParticleQuadsTemp
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 */

#ifndef IMP_QUAD_MODIFIER_H
#define IMP_QUAD_MODIFIER_H

#include "kernel_config.h"
#include "DerivativeAccumulator.h"
#include "base_types.h"
#include "ParticleTuple.h"
#include "internal/container_helpers.h"

IMP_BEGIN_NAMESPACE
// to keep swig happy
class Particle;

//! A base class for modifiers of ParticleQuadsTemp
/** The primary function of such a class is to change
    the passed particles.

    \see IMP::QuadFunctor

    Implementors should see IMP_QUAD_MODIFIER(). Also see
    QuadDerivativeModifier.
 */
class IMPEXPORT QuadModifier : public Object
{
public:
  typedef ParticleQuad Argument;
  QuadModifier(std::string name="QuadModifier %1%");

  /** Apply the function to a single value*/
  virtual void apply(const ParticleQuad&) const =0;

 /** Apply the function to a single value*/
  virtual void apply_index(Model *m, const ParticleIndexQuad& v) const {
    apply(internal::get_particle(m, v));
  }

  /** Apply the function to a collection of ParticleQuadsTemp */
  virtual void apply_indexes(Model *m, const ParticleIndexQuads &o) const {
    for (unsigned int i=0; i < o.size(); ++i) {
      apply_index(m, o[i]);
    }
  }

  /** Get the set of particles read when applied to the arguments.*/
  virtual ParticlesTemp
    get_input_particles(Particle* p) const =0;
  /** Get the set of particles modifier when applied to the arguments.*/
  virtual ParticlesTemp
    get_output_particles(Particle *p) const =0;
  /** Get the set of input containers when this modifier is applied to
      the arguments. */
  virtual ContainersTemp
    get_input_containers(Particle *p) const =0;
  /** Get the set of output containers when this modifier is applied to
      the arguments. */
  virtual ContainersTemp
    get_output_containers(Particle *p) const =0;
};

IMP_OUTPUT_OPERATOR(QuadModifier);

#ifdef IMP_DOXYGEN
/** Create a modifier from a functor. C++ only. The function should take
    a Quad as an argument.
    This is intended to be used as a temporary object and not stored.
    A reference to the functor is saved.
 */
template <class Functor>
QuadModifier *create_particle_quad_modifier(const Functor& f);
#elif !defined(SWIG)
template <class Functor>
class FunctorQuadModifier: public QuadModifier {
  const Functor &f_;
public:
  FunctorQuadModifier(const Functor& f):
    QuadModifier("FunctorModifier %1%"),
    f_(f){}
  IMP_QUAD_MODIFIER(FunctorQuadModifier);
};

template <class Functor>
void FunctorQuadModifier<Functor>::apply(const ParticleQuad& v) const {
  f_(v);
}

template <class Functor>
 ParticlesTemp
FunctorQuadModifier<Functor>::get_input_particles(Particle* p) const {
  return ParticlesTemp(1,p);
}
template <class Functor>
ParticlesTemp
FunctorQuadModifier<Functor>::get_output_particles(Particle *p) const {
  return ParticlesTemp(1,p);
}
template <class Functor>
ContainersTemp
FunctorQuadModifier<Functor>::get_input_containers(Particle *) const {
  return ContainersTemp();
}
template <class Functor>
ContainersTemp
FunctorQuadModifier<Functor>::get_output_containers(Particle *) const {
  return ContainersTemp();
}

/** Create a modifier from a functor. C++ only.*/
template <class Functor>
inline FunctorQuadModifier<Functor> *
create_particle_quad_modifier(const Functor& f) {
  return new FunctorQuadModifier<Functor>(f);
}

#endif

IMP_OBJECTS(QuadModifier,QuadModifiers);

//! Create a functor which can be used with build in C++ and python commands
/** For example, you can do
    \code
    std::for_each(particles.begin(), particles.end(),
                  IMP::SingletonFunctor(new IMP::core::Transform(tr)));
    IMP::for_each(particles,
                  IMP::SingletonFunctor(new IMP::core::Transform(tr)));
    \endcode
    in C++ (the second can be used with when \c particles is a temporary
    value) or
    \verbatim
    map(SingletonFunctor(Transform(tr)), particles)
    \endverbatim
    in python.

    \see IMP::QuadModifier
 */
class QuadFunctor {
  Pointer<const QuadModifier> f_;
public:
  //! Store the QuadModifier and the optional DerivativeAccumulator
  QuadFunctor(const QuadModifier *f): f_(f){}
  void operator()( ParticleQuad p) const {
    f_->apply(p);
  }
};



IMP_END_NAMESPACE

#endif  /* IMP_QUAD_MODIFIER_H */
