/**
 *  \file UnaryFunction.h    \brief Single variable function.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 */

#ifndef IMPKERNEL_UNARY_FUNCTION_H
#define IMPKERNEL_UNARY_FUNCTION_H

#include "kernel_config.h"
#include "base_types.h"
#include "Object.h"
#include "Pointer.h"

IMP_BEGIN_NAMESPACE

//! Abstract single variable functor class for score functions.
/** These functors take a single feature value, and return a corresponding
    score (and optionally also the first derivative).

    Implementers should check out IMP_UNARY_FUNCTION() and
    IMP_UNARY_FUNCTION_INLINE().
 */
class IMPEXPORT UnaryFunction : public IMP::base::Object
{
public:
  UnaryFunction();

  //! Calculate score with respect to the given feature.
  /** \param[in] feature Value of feature being tested.
      \return Score
   */
  virtual double evaluate(double feature) const
#ifdef SWIG
      =0;
#else
  {
    // to support easy generic classes
    return evaluate(feature);
  }
#endif

  //! Calculate score and derivative with respect to the given feature.
  /** \param[in] feature Value of feature being tested.
      \return a FloatPair containing the score and its partial derivative
              with respect to the given feaure.
   */
  virtual DerivativePair evaluate_with_derivative(double feature) const {
    // to support easy generic classes
    return evaluate_with_derivative(feature);
  }

  IMP_REF_COUNTED_DESTRUCTOR(UnaryFunction);
};

IMP_OBJECTS(UnaryFunction,UnaryFunctions);

IMP_END_NAMESPACE

#endif  /* IMPKERNEL_UNARY_FUNCTION_H */
