/**
 *  \file declare_CLASSNAMEContainer.h    \brief A container for CLASSNAMEs.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
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
#include "macros.h"

IMP_BEGIN_NAMESPACE
class CLASSNAMEModifier;
class CLASSNAMEDerivativeModifier;
class CLASSNAMEScore;

//! A shared container for CLASSNAMEs
/** Stores a searchable shared collection of CLASSNAMEs.
    \ingroup restraints

    \implementationwithoutexample{CLASSNAMEContainer, IMP_HEADERNAME_CONTAINER}
 */
class IMPEXPORT CLASSNAMEContainer : public Container
{
 protected:
  CLASSNAMEContainer(){}
  CLASSNAMEContainer(Model *m,
                     std::string name="CLASSNAMEContainer %1%");
public:
  typedef VARIABLETYPE ContainedType;
  typedef PLURALVARIABLETYPE ContainedTypes;
  /** \note This function may be linear. Be aware of the complexity
      bounds of your particular container.
   */
  virtual bool get_contains_FUNCTIONNAME(ARGUMENTTYPE v) const =0;

  PLURALVARIABLETYPE get_FUNCTIONNAMEs() const {
    return IMP::internal::get_particle(get_model(),
                                       get_indexes());
  }
#ifndef IMP_DOXGEN
  //! return the number of CLASSNAMEs in the container
  /** \note this isn't always constant time
   */
  virtual unsigned int get_number_of_FUNCTIONNAMEs() const {
    return get_number();
  }

  virtual VARIABLETYPE get_FUNCTIONNAME(unsigned int i) const {
    return get(i);
  }

#endif

  //! Apply a SingletonModifier to the contents
  virtual void apply(const CLASSNAMEModifier *sm) const=0;
  //! Apply a SingletonModifier to the contents
  virtual void apply(const CLASSNAMEDerivativeModifier *sm,
                     DerivativeAccumulator &da) const=0;

  //! Evaluate a score on the contents
  virtual double evaluate(const CLASSNAMEScore *s,
                          DerivativeAccumulator *da) const=0;

  //! Evaluate a score on the contents
  virtual double evaluate_if_good(const CLASSNAMEScore *s,
                                  DerivativeAccumulator *da,
                                  double max) const=0;

  /** Return true if the contents of the container changed since the last
      evaluate.
  */
  virtual bool get_contents_changed() const=0;

#ifndef IMP_DOXYGEN
  typedef VARIABLETYPE value_type;
  VARIABLETYPE get(unsigned int i) const {
    return IMP::internal::get_particle(get_model(),
                                       get_indexes()[i]);
  }
  PLURALVARIABLETYPE get() const {
    return IMP::internal::get_particle(get_model(), get_indexes());
  }
  bool get_contains(ARGUMENTTYPE v) const {
    return get_contains_FUNCTIONNAME(v);
  }
  unsigned int get_number() const {return get_indexes().size();}
  virtual PLURALINDEXTYPE get_indexes() const=0;
  virtual PLURALINDEXTYPE get_all_possible_indexes() const=0;
  virtual Restraints create_decomposition(CLASSNAMEScore *s) const=0;
#ifndef SWIG
  virtual bool get_provides_access() const {return false;}
  virtual const PLURALINDEXTYPE& get_access() const {
    IMP_THROW("Object not implemented properly.", IndexException);
  }
#endif
#endif

  IMP_REF_COUNTED_NONTRIVIAL_DESTRUCTOR(CLASSNAMEContainer);
};

IMP_END_NAMESPACE

#endif  /* IMPKERNEL_DECLARE_HEADERNAME_CONTAINER_H */