/**
 *  \file GroupnameContainer.h    \brief A container for classnames.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2010 Sali Lab. All rights reserved.
 */

#ifndef IMP_GROUPNAME_CONTAINER_H
#define IMP_GROUPNAME_CONTAINER_H

#include "config.h"
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
class GroupnameModifier;
class GroupnameScore;


//! A shared container for classnames
/** Stores a searchable shared collection of classnames.
    \ingroup restraints

    \implementationwithoutexample{GroupnameContainer, IMP_GROUPNAME_CONTAINER}
 */
class IMPEXPORT GroupnameContainer : public Container
{
  internal::OwnerPointer<Container> added_, removed_;
  struct Accessor {
    typedef Accessor This;
    typedef Value result_type;
    typedef unsigned int argument_type;
    result_type operator()(argument_type i) const {
      return o_->get_classname(i);
    }
    Accessor(GroupnameContainer *pc): o_(pc){}
    Accessor(): o_(NULL){}
    IMP_COMPARISONS_1(o_);
  private:
    // This should be ref counted, but swig memory management is broken
    GroupnameContainer* o_;
  };
 protected:
  /** Containers must have containers that keep track of the particles
      which have been added or since the last step. These containers
      must be registered with the parent GroupnameContainer.

      Containers which are themselves returned by the get_added/removed
      functions do not have to register such containers.
  */
  void set_added_and_removed_containers(GroupnameContainer* added,
                                        GroupnameContainer* removed) {
    added_=added;
    removed_=removed;
  }

public:
#ifndef IMP_DOXYGEN
  bool get_is_added_or_removed_container() {
    return !added_;
  }
#endif

  GroupnameContainer(std::string name="GroupnameContainer %1%");

  /** \note This function may be linear. Be aware of the complexity
      bounds of your particular container.
   */
  virtual bool get_contains_classname(PassValue v) const =0;
  //! return the number of classnames in the container
  /** \note this isn't always constant time
   */
  virtual unsigned int get_number_of_classnames() const =0;

  virtual Value get_classname(unsigned int i) const=0;

#ifdef IMP_DOXYGEN
  //! An iterator through the contents of the container
  class ClassnameIterator;
#else
  typedef internal::IndexingIterator<Accessor> ClassnameIterator;
#endif
  //! begin iterating through the classnames
  ClassnameIterator classnames_begin() const {
    // Since I can't make the count mutable in Object
    return
      ClassnameIterator(Accessor(const_cast<GroupnameContainer*>(this)),
                        0);
  }
  //! iterate through the classnames
  ClassnameIterator classnames_end() const {
    return
      ClassnameIterator(Accessor(const_cast<GroupnameContainer*>(this)),
                        get_number_of_classnames());
    }

  //! Apply a SingletonModifier to the contents
  virtual void apply(const GroupnameModifier *sm)=0;

  //! Apply a SingletonModifier to the contents
  virtual void apply(const GroupnameModifier *sm, DerivativeAccumulator &da)=0;

  //! Avaluate a score on the contents
  virtual double evaluate(const GroupnameScore *s,
                          DerivativeAccumulator *da) const=0;

  /** \name Incremental Scoring
      When incremental scoring is used, the container keeps track of
      changes to it since the last Model::evaluate() call.
      \unstable{ClassnameContainer::get_removed_groupnames_container()}
      The address of the objects returned should not change over the lifetime
      of this container (but, of course, their contents will).
      @{
  */
  GroupnameContainer* get_removed_groupnames_container() const {
    IMP_USAGE_CHECK(added_, "The containers returned by "
                    << " get_added_groupnames_container() do not "
                    << " track their own added and removed contents.",
                    ValueException);
    GroupnameContainer *ret= dynamic_cast<GroupnameContainer*>(removed_.get());
    IMP_INTERNAL_CHECK(ret, "Cannot cast object " << removed_->get_name()
                       << " to a GroupnameContainer.");
    return ret;
  }
  GroupnameContainer* get_added_groupnames_container() const {
    IMP_USAGE_CHECK(added_, "The containers returned by "
                    << " get_added_groupnames_container() do not "
                    << " track their own added and removed contents.",
                    ValueException);
    GroupnameContainer *ret= dynamic_cast<GroupnameContainer*>(added_.get());
    IMP_INTERNAL_CHECK(ret, "Cannot cast object " << added_->get_name()
                       << " to a GroupnameContainer.");
    return ret;
  }
  /** Return the change in score (and derivatives) since the last
      evaluate of the current contents of the container.
  */
  virtual double evaluate_change(const GroupnameScore *o,
                                DerivativeAccumulator *da) const = 0;


  /** Return the score of the last evaluate for the current contents of the
      container.
  */
  virtual double evaluate_prechange(const GroupnameScore *o,
                                    DerivativeAccumulator *da) const = 0;
  /** @} */

  //! Get all the Classnames from the container
  virtual ClassnamesTemp get_classnames() const=0;

#ifndef IMP_DOXYGEN
  Value get(unsigned int i) const {return get_classname(i);}
  bool get_contains(PassValue v) const {
    return get_contains_classname(v);
  }
  unsigned int get_number() const {return get_number_of_classnames();}
#endif

  IMP_REF_COUNTED_NONTRIVIAL_DESTRUCTOR(GroupnameContainer);
};

IMP_OUTPUT_OPERATOR(GroupnameContainer);

//! A collection of containers
IMP_OBJECTS(GroupnameContainer);
/** \objects{GroupnameContainer}
*/
/** \objectstemp{GroupnameContainer}
*/

IMP_END_NAMESPACE

#endif  /* IMP_GROUPNAME_CONTAINER_H */
