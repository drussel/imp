/**
 *  \file SingletonContainer.h    \brief A container for Singletons.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 */

#ifndef IMPKERNEL_SINGLETON_CONTAINER_H
#define IMPKERNEL_SINGLETON_CONTAINER_H

#include "kernel_config.h"
#include "internal/IndexingIterator.h"
#include "Particle.h"
#include "container_base.h"
#include "VersionInfo.h"
#include "DerivativeAccumulator.h"
#include "internal/OwnerPointer.h"
#include "ParticleTuple.h"
#include "SingletonScore.h"
#include "SingletonModifier.h"
#include "SingletonDerivativeModifier.h"
#include "macros.h"

IMP_BEGIN_NAMESPACE

// for swig
class SingletonScore;
class SingletonModifier;


//! A shared container for Singletons
/** Stores a searchable shared collection of Singletons.
    \ingroup restraints

    \implementationwithoutexample{SingletonContainer, IMP_SINGLETON_CONTAINER}
 */
class IMPEXPORT SingletonContainer : public Container
{
 protected:
  SingletonContainer(){}
  SingletonContainer(Model *m,
                     std::string name="SingletonContainer %1%");
#ifndef IMP_DOXYGEN
  template <class S>
    double call_evaluate_index(const S *s,
                         ParticleIndex a,
                         DerivativeAccumulator *da) const {
    return s->S::evaluate_index(get_model(), a, da);
  }
  double call_evaluate_index(const SingletonScore *s,
                              ParticleIndex a,
                              DerivativeAccumulator *da) const {
    return s->evaluate_index(get_model(), a, da);
  }
  template <class S>
    double call_evaluate_if_good_index(const S *s,
                                 ParticleIndex a,
                                 DerivativeAccumulator *da,
                                 double max) const {
    return s->S::evaluate_if_good_index(get_model(), a, da, max);
  }
  double call_evaluate_if_good_index(const SingletonScore *s,
                                      ParticleIndex a,
                                      DerivativeAccumulator *da,
                                      double max) const {
    return s->evaluate_if_good_index(get_model(), a, da, max);
  }
  template <class S>
    void call_apply_index(const S *s,
                    ParticleIndex a) const {
    s->S::apply_index(get_model(), a);
  }
  void call_apply(const SingletonModifier *s,
                         ParticleIndex a) const {
    s->apply_index(get_model(), a);
  }
  template <class S>
    void call_apply_index(const S *s,
                           ParticleIndex a,
                           DerivativeAccumulator *&da) const {
    s->S::apply_index(get_model(), a, da);
  }
  void call_apply_index(const SingletonDerivativeModifier *s,
                  ParticleIndex a,
                  DerivativeAccumulator &da) const {
    s->apply_index(get_model(), a, da);
  }
#endif
public:
  typedef Particle* ContainedType;
  typedef ParticlesTemp ContainedTypes;
  /** \note This function may be linear. Be aware of the complexity
      bounds of your particular container.
   */
  virtual bool get_contains_particle(Particle* v) const =0;

  ParticlesTemp get_particles() const {
    return IMP::internal::get_particle(get_model(),
                                       get_indexes());
  }
#ifndef IMP_DOXGEN
  //! return the number of Singletons in the container
  /** \note this isn't always constant time
   */
  virtual unsigned int get_number_of_particles() const {
    return get_number();
  }

  virtual Particle* get_particle(unsigned int i) const {
    return get(i);
  }

#endif

  //! Apply a SingletonModifier to the contents
  virtual void apply(const SingletonModifier *sm) const=0;
  //! Apply a SingletonModifier to the contents
  virtual void apply(const SingletonDerivativeModifier *sm,
                     DerivativeAccumulator &da) const=0;

  //! Evaluate a score on the contents
  virtual double evaluate(const SingletonScore *s,
                          DerivativeAccumulator *da) const=0;

  //! Evaluate a score on the contents
  virtual double evaluate_if_good(const SingletonScore *s,
                                  DerivativeAccumulator *da,
                                  double max) const=0;

  /** Return true if the contents of the container changed since the last
      evaluate.
  */
  virtual bool get_contents_changed() const=0;

#ifndef IMP_DOXYGEN
  typedef Particle* value_type;
  Particle* get(unsigned int i) const {
    return IMP::internal::get_particle(get_model(),
                                       get_indexes()[i]);
  }
  ParticlesTemp get() const {
    return IMP::internal::get_particle(get_model(), get_indexes());
  }
  bool get_contains(Particle* v) const {
    return get_contains_particle(v);
  }
  unsigned int get_number() const {return get_indexes().size();}
  virtual ParticleIndexes get_indexes() const=0;
  virtual ParticleIndexes get_all_possible_indexes() const=0;
  virtual Restraints create_decomposition(SingletonScore *s) const=0;
#ifndef SWIG
  virtual bool get_provides_access() const {return false;}
  virtual const ParticleIndexes& get_access() const {
    IMP_THROW("Object not implemented properly.", IndexException);
  }
#endif
#endif

  IMP_REF_COUNTED_NONTRIVIAL_DESTRUCTOR(SingletonContainer);
};

IMP_OBJECTS(SingletonContainer,SingletonContainers);

IMP_END_NAMESPACE

#endif  /* IMPKERNEL_SINGLETON_CONTAINER_H */
