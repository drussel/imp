/**
 *  \file GroupnameContainer.h    \brief A container for classnames.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-8 Sali Lab. All rights reserved.
 */

#ifndef IMPCORE_GROUPNAME_CONTAINER_H
#define IMPCORE_GROUPNAME_CONTAINER_H

#include "config.h"
#include "internal/IndexingIterator.h"
#include "internal/container_helpers.h"
#include "Particle.h"
#include "base_types.h"
#include "Pointer.h"
#include "VersionInfo.h"

IMP_BEGIN_NAMESPACE

//! A shared container for classnames
/** Stores a searchable shared collection of classnames.
    \ingroup restraints
 */
class IMPEXPORT GroupnameContainer : public RefCountedObject
{
  struct Accessor {
    typedef Accessor This;
    typedef Value result_type;
    typedef unsigned int argument_type;
    result_type operator()(argument_type i) const {
      return o_->get_classname(i);
    }
    Accessor(GroupnameContainer *pc): o_(pc){}
    Accessor(){}
    IMP_COMPARISONS_1(o_);
  private:
    bool is_default() const { return false;}
    // This should be ref counted, but swig memory management is broken
    GroupnameContainer* o_;
  };

public:
  GroupnameContainer();

  //!
  /** \note This function may be linear. Be aware of the complexity
      bounds of your particular container.
   */
  virtual bool get_contains_classname(Value p) const =0;
  //! return the number of classnames in the container
  /** \note this isn't always constant time
   */
  virtual unsigned int get_number_of_classnames() const =0;

  //! get one classname
  virtual Value get_classname(unsigned int i) const=0;

  //! print information about the container
  virtual void show(std::ostream &out = std::cout) const;

  //! provide information about who implemeneted the container
  virtual VersionInfo get_version_info() const=0;

  //! An iterator through the contents of the container
  typedef internal::IndexingIterator<Accessor> ClassnameIterator;
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

  IMP_REF_COUNTED_DESTRUCTOR(GroupnameContainer)
};

IMP_OUTPUT_OPERATOR(GroupnameContainer);

//! A collection of containers
typedef std::vector<GroupnameContainer*> GroupnameContainers;
//! The index to use when this container is store in another object
typedef Index<GroupnameContainer> GroupnameContainerIndex;

IMP_END_NAMESPACE

#define IMP_GROUPNAME_CONTAINER(version_info)                      \
  bool get_contains_classname(Value p) const;                      \
  unsigned int get_number_of_classnames() const;                   \
  Value get_classname(unsigned int i) const;                       \
  void show(std::ostream &out= std::cout) const;                   \
  IMP::VersionInfo get_version_info() const { return version_info; }

#endif  /* IMPCORE_GROUPNAME_CONTAINER_H */
