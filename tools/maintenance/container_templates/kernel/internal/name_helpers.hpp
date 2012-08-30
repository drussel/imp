/**
 *  \file internal/FUNCTIONNAME_helpers.h
 *  \brief A container for CLASSNAMEs.
 *
 *  BLURB
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */

#ifndef IMPKERNEL_INTERNAL_HEADERNAME_HELPERS_H
#define IMPKERNEL_INTERNAL_HEADERNAME_HELPERS_H

#include "../kernel_config.h"
#include "../CLASSNAMEContainer.h"
#include "../CLASSNAMEModifier.h"
#include "../CLASSNAMEDerivativeModifier.h"
#include "../CLASSNAMEScore.h"
#include "container_helpers.h"
#include <algorithm>

IMP_BEGIN_NAMESPACE
class Model;
IMP_END_NAMESPACE

IMP_BEGIN_INTERNAL_NAMESPACE

template <class S>
inline double call_evaluate_index(Model *m, const S *s,
                           PASSINDEXTYPE a,
                           DerivativeAccumulator *da) {
  return s->S::evaluate_index(m, a, da);
}
inline double call_evaluate_index(Model *m, const CLASSNAMEScore *s,
                           PASSINDEXTYPE a,
                           DerivativeAccumulator *da) {
    return s->evaluate_index(m, a, da);
}
template <class S>
inline double call_evaluate_if_good_index(Model *m, const S *s,
                                   PASSINDEXTYPE a,
                                   DerivativeAccumulator *da,
                                   double max) {
  return s->S::evaluate_if_good_index(m, a, da, max);
}
inline double call_evaluate_if_good_index(Model *m, const CLASSNAMEScore *s,
                                   PASSINDEXTYPE a,
                                   DerivativeAccumulator *da,
                                   double max) {
  return s->evaluate_if_good_index(m, a, da, max);
}
template <class S>
inline void call_apply_index(Model *m, const S *s,
                      PASSINDEXTYPE a) {
  s->S::apply_index(m, a);
}
inline void call_apply(Model *m, const CLASSNAMEModifier *s,
                PASSINDEXTYPE a) {
  s->apply_index(m, a);
}
template <class S>
inline void call_apply_index(Model *m, const S *s,
                      PASSINDEXTYPE a,
                      DerivativeAccumulator *&da) {
  s->S::apply_index(m, a, da);
}
inline void call_apply_index(Model *m, const CLASSNAMEDerivativeModifier *s,
                      PASSINDEXTYPE a,
                  DerivativeAccumulator &da) {
  s->apply_index(m, a, da);
}

IMP_END_INTERNAL_NAMESPACE


#endif  /* IMPKERNEL_INTERNAL_HEADERNAME_HELPERS_H */
