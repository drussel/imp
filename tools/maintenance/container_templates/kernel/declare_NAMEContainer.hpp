/**
 *  \file IMP/declare_CLASSNAMEContainer.h
 *  \brief A container for CLASSNAMEs.
 *
 *  BLURB
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */

#ifndef IMPKERNEL_DECLARE_HEADERNAME_CONTAINER_H
#define IMPKERNEL_DECLARE_HEADERNAME_CONTAINER_H

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
#include <IMP/base/deprecation_macros.h>
#include <algorithm>


IMP_BEGIN_NAMESPACE
class CLASSNAMEModifier;
class CLASSNAMEScore;

//! A shared container for CLASSNAMEs
/** Stores a searchable shared collection of CLASSNAMEs.
    \headerfile CLASSNAMEContainer.h "IMP/CLASSNAMEContainer.h"
    \implementationwithoutexample{CLASSNAMEContainer, IMP_HEADERNAME_CONTAINER}
 */
class IMPEXPORT CLASSNAMEContainer : public Container
{
  IMP_PROTECTED_CONSTRUCTOR(CLASSNAMEContainer, (Model *m,
                           std::string name="CLASSNAMEContainer %1%"), );
public:
  typedef VARIABLETYPE ContainedType;
  typedef PLURALVARIABLETYPE ContainedTypes;
  typedef PLURALINDEXTYPE ContainedIndexTypes;
  typedef INDEXTYPE ContainedIndexType;

  //! Just use apply() in the base class
  void apply_generic(const CLASSNAMEModifier *m) const {
    apply(m);
  }

  //! Apply a SingletonModifier to the contents
  void apply(const CLASSNAMEModifier *sm) const {
    do_apply(sm);
  }

  /** Get all the indexes contained in the container.*/
  virtual PLURALINDEXTYPE get_indexes() const=0;
  /** Get all the indexes that might possibly be contained in the
      container, useful with dynamic containers.*/
  virtual PLURALINDEXTYPE get_range_indexes() const=0;

#ifndef IMP_DOXYGEN
  PLURALVARIABLETYPE get() const {
    return IMP::internal::get_particle(get_model(),
                                       get_indexes());
  }

  VARIABLETYPE get(unsigned int i) const {
    return IMP::internal::get_particle(get_model(),
                                       get_indexes()[i]);
  }
  unsigned int get_number() const {return get_indexes().size();}
#ifndef SWIG
  virtual bool get_provides_access() const {return false;}
  virtual const PLURALINDEXTYPE& get_access() const {
    IMP_THROW("Object not implemented properly.", base::IndexException);
  }


  template <class Functor>
    Functor for_each(Functor f) {
    PLURALINDEXTYPE vs=get_indexes();
    // use boost range instead
    return std::for_each(vs.begin(), vs.end(), f);
  }

#endif
#endif

  /** Use this for debugging only.
   */
  PLURALVARIABLETYPE get_FUNCTIONNAMEs() const;

#if defined(IMP_USE_DEPRECATED)
  /** \brief This function is very slow and you should think hard about using
      it.

      \deprecated This is slow and dependent on the order of elements in the
      tuple.

      Return whether the container has the given element.*/
  IMP_DEPRECATED_WARN
    bool get_contains_FUNCTIONNAME(VARIABLETYPE v) const;

  /** \deprecated This can be very slow and is probably not useful
   */
  IMP_DEPRECATED_WARN unsigned int get_number_of_FUNCTIONNAMEs() const;

  /** \deprecated Use indexes instead and thing about using the
      IMP_CONTAINER_FOREACH() macro.*/
  IMP_DEPRECATED_WARN VARIABLETYPE
    get_FUNCTIONNAME(unsigned int i) const;

#endif

  IMP_PROTECTED_METHOD(virtual void,
                       do_apply, (const CLASSNAMEModifier *sm), const=0,);

  IMP_REF_COUNTED_NONTRIVIAL_DESTRUCTOR(CLASSNAMEContainer);
};


/** This class allows either a list or a container to be
    accepted as input.
*/
class IMPEXPORT CLASSNAMEContainerAdaptor:
#if !defined(SWIG) && !defined(IMP_DOXYGEN)
public base::Pointer<CLASSNAMEContainer>
#else
public base::InputAdaptor
#endif
{
  typedef base::Pointer<CLASSNAMEContainer> P;
 public:
  CLASSNAMEContainerAdaptor(){}
  CLASSNAMEContainerAdaptor(CLASSNAMEContainer *c);
  template <class C>
  CLASSNAMEContainerAdaptor(base::internal::PointerBase<C> c): P(c){}
  CLASSNAMEContainerAdaptor(const PLURALVARIABLETYPE &t,
                          std::string name="CLASSNAMEContainerAdaptor%1%");
};


IMP_END_NAMESPACE

#endif  /* IMPKERNEL_DECLARE_HEADERNAME_CONTAINER_H */
