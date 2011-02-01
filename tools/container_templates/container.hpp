/**
 *  \file CLASSNAMEContainer.h    \brief A container for CLASSNAMEs.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 */

#ifndef IMP_HEADERNAME_CONTAINER_H
#define IMP_HEADERNAME_CONTAINER_H

#include "kernel_config.h"
#include "internal/IndexingIterator.h"
#include "Particle.h"
#include "container_base.h"
#include "VersionInfo.h"
#include "VectorOfRefCounted.h"
#include "DerivativeAccumulator.h"
#include "internal/OwnerPointer.h"
#include "ParticleTuple.h"
#include "macros.h"

IMP_BEGIN_NAMESPACE
class CLASSNAMEModifier;
class CLASSNAMEScore;

class CLASSNAMEContainer;
typedef std::pair<CLASSNAMEContainer*,
                  CLASSNAMEContainer*> CLASSNAMEContainerPair;

//! A shared container for CLASSNAMEs
/** Stores a searchable shared collection of CLASSNAMEs.
    \ingroup restraints

    \implementationwithoutexample{CLASSNAMEContainer, IMP_HEADERNAME_CONTAINER}
 */
class IMPEXPORT CLASSNAMEContainer : public Container
{
  mutable internal::OwnerPointer<Container> added_, removed_;
  struct Accessor {
    typedef VARIABLETYPE result_type;
    typedef unsigned int argument_type;
    result_type operator()(argument_type i) const {
      return o_->get_FUNCTIONNAME(i);
    }
    Accessor(CLASSNAMEContainer *pc): o_(pc){}
    Accessor(): o_(NULL){}
    IMP_COMPARISONS_1(Accessor, o_);
  private:
    // This should be ref counted, but swig memory management is broken
    CLASSNAMEContainer* o_;
  };
 protected:
  /** Containers must have containers that keep track of the particles
      which have been added or since the last step. These containers
      must be registered with the parent CLASSNAMEContainer.

      Containers which are themselves returned by the get_added/removed
      functions do not have to register such containers.
  */
  virtual CLASSNAMEContainerPair
    get_added_and_removed_containers() const =0;
  bool get_has_added_and_removed_containers() const {
    return added_ && added_->get_is_shared()
      || removed_ && removed_->get_is_shared();
  }
  CLASSNAMEContainer(){}
  CLASSNAMEContainer(Model *m,
                     std::string name="CLASSNAMEContainer %1%");
public:
  typedef VARIABLETYPE ContainedType;
  /** \note This function may be linear. Be aware of the complexity
      bounds of your particular container.
   */
  virtual bool get_contains_FUNCTIONNAME(ARGUMENTTYPE v) const =0;
  //! return the number of CLASSNAMEs in the container
  /** \note this isn't always constant time
   */
  virtual unsigned int get_number_of_FUNCTIONNAMEs() const =0;

  PLURALVARIABLETYPE get_FUNCTIONNAMEs() const {
    return PLURALVARIABLETYPE(FUNCTIONNAMEs_begin(),
                              FUNCTIONNAMEs_end());
  }
  virtual VARIABLETYPE get_FUNCTIONNAME(unsigned int i) const=0;

#ifdef IMP_DOXYGEN
  //! An iterator through the contents of the container
  class TYPENAMEIterator;
#else
  typedef internal::IndexingIterator<Accessor> TYPENAMEIterator;
#endif
#ifndef SWIG
  //! begin iterating through the CLASSNAMEs
  TYPENAMEIterator FUNCTIONNAMEs_begin() const {
    // Since I can't make the count mutable in Object
    return
      TYPENAMEIterator(Accessor(const_cast<CLASSNAMEContainer*>(this)),
                        0);
  }
  //! iterate through the CLASSNAMEs
  TYPENAMEIterator FUNCTIONNAMEs_end() const {
    return
      TYPENAMEIterator(Accessor(const_cast<CLASSNAMEContainer*>(this)),
                        get_number_of_FUNCTIONNAMEs());
    }
#endif

  //! Apply a SingletonModifier to the contents
  virtual void apply(const CLASSNAMEModifier *sm)=0;

  //! Apply a SingletonModifier to the contents
  virtual void apply(const CLASSNAMEModifier *sm, DerivativeAccumulator &da)=0;

  //! Evaluate a score on the contents
  virtual double evaluate(const CLASSNAMEScore *s,
                          DerivativeAccumulator *da) const=0;


  /** \name Incremental Scoring
      When incremental scoring is used, the container keeps track of
      changes to it since the last Model::evaluate() call.
      \unstable{CLASSNAMEContainer::get_removed_container()}
      The address of the objects returned should not change over the lifetime
      of this container (but, of course, their contents will).
      @{
  */
  CLASSNAMEContainer* get_removed_container() const {
    // must not be an added or removed container
    get_model();
    if (!added_) {
      std::pair<CLASSNAMEContainer*, CLASSNAMEContainer*>
        cp= get_added_and_removed_containers();
      added_=cp.first;
      removed_=cp.second;
    }
    IMP_USAGE_CHECK(added_, "The containers returned by "
                    << " get_added_container() do not "
                    << " track their own added and removed contents.");
    CLASSNAMEContainer *ret= dynamic_cast<CLASSNAMEContainer*>(removed_.get());
    IMP_INTERNAL_CHECK(ret, "Cannot cast object " << removed_->get_name()
                       << " to a CLASSNAMEContainer.");
    return ret;
  }
  CLASSNAMEContainer* get_added_container() const {
    // must not be an added or removed container
    if (!added_) {
      std::pair<CLASSNAMEContainer*, CLASSNAMEContainer*>
        cp= get_added_and_removed_containers();
      added_=cp.first;
      removed_=cp.second;
    }
    IMP_USAGE_CHECK(added_, "The containers returned by "
                    << " get_added_container() do not "
                    << " track their own added and removed contents.");
    CLASSNAMEContainer *ret= dynamic_cast<CLASSNAMEContainer*>(added_.get());
    IMP_INTERNAL_CHECK(ret, "Cannot cast object " << added_->get_name()
                       << " to a CLASSNAMEContainer.");
    return ret;
  }
  /** Return the change in score (and derivatives) since the last
      evaluate of the current contents of the container.
  */
  virtual double evaluate_change(const CLASSNAMEScore *o,
                                DerivativeAccumulator *da) const = 0;


  /** Return the score of the last evaluate for the current contents of the
      container.
  */
  virtual double evaluate_prechange(const CLASSNAMEScore *o,
                                    DerivativeAccumulator *da) const = 0;
  /** @} */

#ifndef IMP_DOXYGEN
  typedef VARIABLETYPE value_type;
  VARIABLETYPE get(unsigned int i) const {return get_FUNCTIONNAME(i);}
  PLURALVARIABLETYPE get() const {
    return get_FUNCTIONNAMEs();
  }
  bool get_contains(ARGUMENTTYPE v) const {
    return get_contains_FUNCTIONNAME(v);
  }
  unsigned int get_number() const {return get_number_of_FUNCTIONNAMEs();}
  virtual bool get_provides_access() const {return false;}
  virtual const PLURALVARIABLETYPE& get_access() const {
    IMP_THROW("Object not implemented properly.", IndexException);
  }
#endif

  IMP_REF_COUNTED_NONTRIVIAL_DESTRUCTOR(CLASSNAMEContainer);
};

IMP_OBJECTS(CLASSNAMEContainer,CLASSNAMEContainers);

IMP_END_NAMESPACE

#endif  /* IMP_HEADERNAME_CONTAINER_H */
