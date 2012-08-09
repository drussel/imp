/**
 *  \file IMP/core/AttributeSingletonScore.h
 *  \brief A score based on the unmodified value of an attribute.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */

#ifndef IMPCORE_ATTRIBUTE_SINGLETON_SCORE_H
#define IMPCORE_ATTRIBUTE_SINGLETON_SCORE_H

#include "core_config.h"
#include <IMP/generic.h>
#include <IMP/SingletonScore.h>
#include <IMP/Pointer.h>
#include <IMP/UnaryFunction.h>
#include <IMP/singleton_macros.h>

IMPCORE_BEGIN_NAMESPACE


//! Apply a function to an attribute.
/** This Score scores a particle by passing an attribute value directly
    to a unary function.*/
template <class UF>
class GenericAttributeSingletonScore: public SingletonScore {
  IMP::OwnerPointer<UF> f_;
  FloatKey k_;
public:
  //! Apply function f to attribete k
  GenericAttributeSingletonScore(UnaryFunction *f, FloatKey k);
  IMP_SIMPLE_SINGLETON_SCORE(GenericAttributeSingletonScore);
};
#if !defined(SWIG) && !defined(IMP_DOXYGEN)
template <class UF>
inline GenericAttributeSingletonScore<UF>
::GenericAttributeSingletonScore(UnaryFunction *f,
                                 FloatKey k): f_(f),
                                              k_(k){}
template <class UF>
inline Float GenericAttributeSingletonScore<UF>::evaluate(Particle *b,
                                        DerivativeAccumulator *da) const
{
  if (da) {
    Float v, d;
    boost::tie(v,d) = f_->UF::evaluate_with_derivative(b->get_value(k_));
    b->add_to_derivative(k_, d, *da);
    return v;
  } else {
    return f_->UF::evaluate(b->get_value(k_));
  }
}
template <class UF>
inline void GenericAttributeSingletonScore<UF>::do_show(std::ostream &out) const
{
  out << "function is " << *f_ << " on " << k_;
}
#endif

/** For python users. C++ users should use create_attribute_singleton_score().*/
typedef GenericAttributeSingletonScore<UnaryFunction> AttributeSingletonScore;

/** Provide a compile time binding version of AttributeSingletonScore,
    which runs faster than the runtime bound version. UF should be a subclass of
    UnaryFunction (and not pointer to a generic UnaryFunction).*/
template <class UF>
inline GenericAttributeSingletonScore<UF>*
create_attribute_singleton_score(UF *uf,
                                 FloatKey k) {
  return new GenericAttributeSingletonScore<UF>(uf, k);
}
IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_ATTRIBUTE_SINGLETON_SCORE_H */
