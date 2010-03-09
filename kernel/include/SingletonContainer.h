/**
 *  \file SingletonContainer.h    \brief A container for particles.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 */

#ifndef IMP_SINGLETON_CONTAINER_H
#define IMP_SINGLETON_CONTAINER_H

#include "kernel_config.h"
#include "internal/IndexingIterator.h"
#include "Particle.h"
#include "container_base.h"
#include "utility.h"
#include "VersionInfo.h"
#include "base_types.h"
#include "Pointer.h"
#include "VectorOfRefCounted.h"
#include "VersionInfo.h"
#include "DerivativeAccumulator.h"
#include "internal/OwnerPointer.h"
#include "macros.h"

IMP_BEGIN_NAMESPACE
class SingletonModifier;
class SingletonScore;


//! A shared container for particles
/** Stores a searchable shared collection of particles.
    \ingroup restraints

    \implementationwithoutexample{SingletonContainer, IMP_SINGLETON_CONTAINER}
 */
class IMPEXPORT SingletonContainer : public Container
{
  internal::OwnerPointer<Container> added_, removed_;
  struct Accessor {
    typedef Accessor This;
    typedef Particle* result_type;
    typedef unsigned int argument_type;
    result_type operator()(argument_type i) const {
      return o_->get_particle(i);
    }
    Accessor(SingletonContainer *pc): o_(pc){}
    Accessor(): o_(NULL){}
    IMP_COMPARISONS_1(o_);
  private:
    // This should be ref counted, but swig memory management is broken
    SingletonContainer* o_;
  };
 protected:
  /** Containers must have containers that keep track of the particles
      which have been added or since the last step. These containers
      must be registered with the parent SingletonContainer.

      Containers which are themselves returned by the get_added/removed
      functions do not have to register such containers.
  */
  void set_added_and_removed_containers(SingletonContainer* added,
                                        SingletonContainer* removed) {
    added_=added;
    removed_=removed;
  }

public:
#ifndef IMP_DOXYGEN
  bool get_is_added_or_removed_container() {
    return !added_;
  }
#endif

  SingletonContainer(std::string name="SingletonContainer %1%");

  /** \note This function may be linear. Be aware of the complexity
      bounds of your particular container.
   */
  virtual bool get_contains_particle(Particle* v) const =0;
  //! return the number of particles in the container
  /** \note this isn't always constant time
   */
  virtual unsigned int get_number_of_particles() const =0;

  ParticlesTemp get_particles() const {
    return ParticlesTemp(particles_begin(),
                          particles_end());
  }
  virtual Particle* get_particle(unsigned int i) const=0;

#ifdef IMP_DOXYGEN
  //! An iterator through the contents of the container
  class ParticleIterator;
#else
  typedef internal::IndexingIterator<Accessor> ParticleIterator;
#endif
  //! begin iterating through the particles
  ParticleIterator particles_begin() const {
    // Since I can't make the count mutable in Object
    return
      ParticleIterator(Accessor(const_cast<SingletonContainer*>(this)),
                        0);
  }
  //! iterate through the particles
  ParticleIterator particles_end() const {
    return
      ParticleIterator(Accessor(const_cast<SingletonContainer*>(this)),
                        get_number_of_particles());
    }

  //! Apply a SingletonModifier to the contents
  virtual void apply(const SingletonModifier *sm)=0;

  //! Apply a SingletonModifier to the contents
  virtual void apply(const SingletonModifier *sm, DerivativeAccumulator &da)=0;

  //! Avaluate a score on the contents
  virtual double evaluate(const SingletonScore *s,
                          DerivativeAccumulator *da) const=0;

  /** \name Incremental Scoring
      When incremental scoring is used, the container keeps track of
      changes to it since the last Model::evaluate() call.
      \unstable{ParticleContainer::get_removed_singletons_container()}
      The address of the objects returned should not change over the lifetime
      of this container (but, of course, their contents will).
      @{
  */
  SingletonContainer* get_removed_singletons_container() const {
    IMP_USAGE_CHECK(added_, "The containers returned by "
                    << " get_added_singletons_container() do not "
                    << " track their own added and removed contents.");
    SingletonContainer *ret= dynamic_cast<SingletonContainer*>(removed_.get());
    IMP_INTERNAL_CHECK(ret, "Cannot cast object " << removed_->get_name()
                       << " to a SingletonContainer.");
    return ret;
  }
  SingletonContainer* get_added_singletons_container() const {
    IMP_USAGE_CHECK(added_, "The containers returned by "
                    << " get_added_singletons_container() do not "
                    << " track their own added and removed contents.");
    SingletonContainer *ret= dynamic_cast<SingletonContainer*>(added_.get());
    IMP_INTERNAL_CHECK(ret, "Cannot cast object " << added_->get_name()
                       << " to a SingletonContainer.");
    return ret;
  }
  /** Return the change in score (and derivatives) since the last
      evaluate of the current contents of the container.
  */
  virtual double evaluate_change(const SingletonScore *o,
                                DerivativeAccumulator *da) const = 0;


  /** Return the score of the last evaluate for the current contents of the
      container.
  */
  virtual double evaluate_prechange(const SingletonScore *o,
                                    DerivativeAccumulator *da) const = 0;
  /** @} */

#ifndef IMP_DOXYGEN
  Particle* get(unsigned int i) const {return get_particle(i);}
  bool get_contains(Particle* v) const {
    return get_contains_particle(v);
  }
  unsigned int get_number() const {return get_number_of_particles();}
#endif

  IMP_REF_COUNTED_NONTRIVIAL_DESTRUCTOR(SingletonContainer);
};

IMP_OUTPUT_OPERATOR(SingletonContainer);

IMP_OBJECTS(SingletonContainer);

IMP_END_NAMESPACE

#endif  /* IMP_SINGLETON_CONTAINER_H */
