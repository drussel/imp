/**
 *  \file QuadContainer.h    \brief A container for particle_quads.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 */

#ifndef IMP_QUAD_CONTAINER_H
#define IMP_QUAD_CONTAINER_H

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
class QuadModifier;
class QuadScore;


//! A shared container for particle_quads
/** Stores a searchable shared collection of particle_quads.
    \ingroup restraints

    \implementationwithoutexample{QuadContainer, IMP_QUAD_CONTAINER}
 */
class IMPEXPORT QuadContainer : public Container
{
  internal::OwnerPointer<Container> added_, removed_;
  struct Accessor {
    typedef Accessor This;
    typedef ParticleQuad result_type;
    typedef unsigned int argument_type;
    result_type operator()(argument_type i) const {
      return o_->get_particle_quad(i);
    }
    Accessor(QuadContainer *pc): o_(pc){}
    Accessor(): o_(NULL){}
    IMP_COMPARISONS_1(o_);
  private:
    // This should be ref counted, but swig memory management is broken
    QuadContainer* o_;
  };
 protected:
  /** Containers must have containers that keep track of the particles
      which have been added or since the last step. These containers
      must be registered with the parent QuadContainer.

      Containers which are themselves returned by the get_added/removed
      functions do not have to register such containers.
  */
  void set_added_and_removed_containers(QuadContainer* added,
                                        QuadContainer* removed) {
    added_=added;
    removed_=removed;
  }

public:
#ifndef IMP_DOXYGEN
  bool get_is_added_or_removed_container() {
    return !added_;
  }
#endif

  QuadContainer(std::string name="QuadContainer %1%");

  /** \note This function may be linear. Be aware of the complexity
      bounds of your particular container.
   */
  virtual bool get_contains_particle_quad(const ParticleQuad& v) const =0;
  //! return the number of particle_quads in the container
  /** \note this isn't always constant time
   */
  virtual unsigned int get_number_of_particle_quads() const =0;

  ParticleQuadsTemp get_particle_quads() const {
    return ParticleQuadsTemp(particle_quads_begin(),
                          particle_quads_end());
  }
  virtual ParticleQuad get_particle_quad(unsigned int i) const=0;

#ifdef IMP_DOXYGEN
  //! An iterator through the contents of the container
  class ParticleQuadIterator;
#else
  typedef internal::IndexingIterator<Accessor> ParticleQuadIterator;
#endif
  //! begin iterating through the particle_quads
  ParticleQuadIterator particle_quads_begin() const {
    // Since I can't make the count mutable in Object
    return
      ParticleQuadIterator(Accessor(const_cast<QuadContainer*>(this)),
                        0);
  }
  //! iterate through the particle_quads
  ParticleQuadIterator particle_quads_end() const {
    return
      ParticleQuadIterator(Accessor(const_cast<QuadContainer*>(this)),
                        get_number_of_particle_quads());
    }

  //! Apply a SingletonModifier to the contents
  virtual void apply(const QuadModifier *sm)=0;

  //! Apply a SingletonModifier to the contents
  virtual void apply(const QuadModifier *sm, DerivativeAccumulator &da)=0;

  //! Avaluate a score on the contents
  virtual double evaluate(const QuadScore *s,
                          DerivativeAccumulator *da) const=0;

  /** \name Incremental Scoring
      When incremental scoring is used, the container keeps track of
      changes to it since the last Model::evaluate() call.
      \unstable{ParticleQuadContainer::get_removed_quads_container()}
      The address of the objects returned should not change over the lifetime
      of this container (but, of course, their contents will).
      @{
  */
  QuadContainer* get_removed_quads_container() const {
    IMP_USAGE_CHECK(added_, "The containers returned by "
                    << " get_added_quads_container() do not "
                    << " track their own added and removed contents.");
    QuadContainer *ret= dynamic_cast<QuadContainer*>(removed_.get());
    IMP_INTERNAL_CHECK(ret, "Cannot cast object " << removed_->get_name()
                       << " to a QuadContainer.");
    return ret;
  }
  QuadContainer* get_added_quads_container() const {
    IMP_USAGE_CHECK(added_, "The containers returned by "
                    << " get_added_quads_container() do not "
                    << " track their own added and removed contents.");
    QuadContainer *ret= dynamic_cast<QuadContainer*>(added_.get());
    IMP_INTERNAL_CHECK(ret, "Cannot cast object " << added_->get_name()
                       << " to a QuadContainer.");
    return ret;
  }
  /** Return the change in score (and derivatives) since the last
      evaluate of the current contents of the container.
  */
  virtual double evaluate_change(const QuadScore *o,
                                DerivativeAccumulator *da) const = 0;


  /** Return the score of the last evaluate for the current contents of the
      container.
  */
  virtual double evaluate_prechange(const QuadScore *o,
                                    DerivativeAccumulator *da) const = 0;
  /** @} */

#ifndef IMP_DOXYGEN
  ParticleQuad get(unsigned int i) const {return get_particle_quad(i);}
  bool get_contains(const ParticleQuad& v) const {
    return get_contains_particle_quad(v);
  }
  unsigned int get_number() const {return get_number_of_particle_quads();}
#endif

  IMP_REF_COUNTED_NONTRIVIAL_DESTRUCTOR(QuadContainer);
};

IMP_OUTPUT_OPERATOR(QuadContainer);

IMP_OBJECTS(QuadContainer,QuadContainers);

IMP_END_NAMESPACE

#endif  /* IMP_QUAD_CONTAINER_H */
