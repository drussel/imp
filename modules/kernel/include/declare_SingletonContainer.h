/**
 *  \file IMP/declare_SingletonContainer.h
 *  \brief A container for Singletons.
 *
 *  WARNING This file was generated from declare_NAMEContainer.hpp
 *  in tools/maintenance/container_templates/kernel
 *  by tools/maintenance/make-container.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */

#ifndef IMPKERNEL_DECLARE_SINGLETON_CONTAINER_H
#define IMPKERNEL_DECLARE_SINGLETON_CONTAINER_H

#include "kernel_config.h"
#include "internal/IndexingIterator.h"
#include "declare_Particle.h"
#include "container_base.h"
#include "internal/container_helpers.h"
#include "DerivativeAccumulator.h"
#include "internal/OwnerPointer.h"
#include "ParticleTuple.h"
#include <IMP/base/ref_counted_macros.h>
#include <IMP/base/check_macros.h>
#include <IMP/base/Pointer.h>
#include <IMP/base/InputAdaptor.h>
#include <IMP/base/utility_macros.h>


IMP_BEGIN_NAMESPACE
class SingletonModifier;
class SingletonDerivativeModifier;
class SingletonScore;

//! A shared container for Singletons
/** Stores a searchable shared collection of Singletons.
    \headerfile SingletonContainer.h "IMP/SingletonContainer.h"
    \implementationwithoutexample{SingletonContainer, IMP_SINGLETON_CONTAINER}
 */
class IMPEXPORT SingletonContainer : public Container
{
  IMP_PROTECTED_CONSTRUCTOR(SingletonContainer, (Model *m,
                           std::string name="SingletonContainer %1%"), );
public:
  typedef Particle* ContainedType;
  typedef ParticlesTemp ContainedTypes;
  typedef ParticleIndexes ContainedIndexTypes;
  typedef ParticleIndex ContainedIndexType;
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
  /** Return the ith Particle* of the container.*/
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

#ifndef IMP_DOXYGEN
  Particle* get(unsigned int i) const {
    return IMP::internal::get_particle(get_model(),
                                       get_indexes()[i]);
  }
  ParticlesTemp get() const {
    return IMP::internal::get_particle(get_model(),
                                       get_indexes());
  }
  /** Return true if the container contains the passed Particle*.*/
  bool get_contains(Particle* v) const {
    return get_contains_particle(v);
  }
  /** Return true if the container contains the passed Particle*.*/
  virtual bool get_contains_index(ParticleIndex v) const {
    return get_contains_particle(IMP::internal
                                     ::get_particle(get_model(),
                                                    v));
  }
  unsigned int get_number() const {return get_indexes().size();}
  virtual ParticleIndexes get_indexes() const=0;
  virtual ParticleIndexes get_all_possible_indexes() const=0;
#ifndef SWIG
  virtual bool get_provides_access() const {return false;}
  virtual const ParticleIndexes& get_access() const {
    IMP_THROW("Object not implemented properly.", base::IndexException);
  }


  template <class Modifier>
  void template_apply(const Modifier *sm,
                      DerivativeAccumulator &da) const {
    apply(sm, da);
  }
  template <class Modifier>
      void template_apply(const Modifier *sm) const {
    apply(sm);
  }
  template <class Score>
      double template_evaluate(const Score *s,
                               DerivativeAccumulator *da) const {
    return evaluate(s, da);
  }
  template <class Score>
      double template_evaluate_if_good(const Score *s,
                                       DerivativeAccumulator *da,
                                       double max) const {
    return evaluate_if_good(s, da, max);
  }


#endif
#endif

  IMP_REF_COUNTED_NONTRIVIAL_DESTRUCTOR(SingletonContainer);
};


/** This class allows either a list or a container to be
    accepted as input.
*/
class IMPEXPORT SingletonContainerAdaptor:
#if !defined(SWIG) && !defined(IMP_DOXYGEN)
public base::Pointer<SingletonContainer>
#else
public base::InputAdaptor
#endif
{
  typedef base::Pointer<SingletonContainer> P;
 public:
  SingletonContainerAdaptor(){}
  SingletonContainerAdaptor(SingletonContainer *c);
  template <class C>
  SingletonContainerAdaptor(base::internal::PointerBase<C> c): P(c){}
  SingletonContainerAdaptor(const ParticlesTemp &t,
                          std::string name="SingletonContainerAdaptor%1%");
};


IMP_END_NAMESPACE

#endif  /* IMPKERNEL_DECLARE_SINGLETON_CONTAINER_H */
