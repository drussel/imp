/**
 *  \file AttributeSingletonScore.cpp
 *  \brief A score based on an unmodified attribute value.
 *
 *  Copyright 2007-2010 Sali Lab. All rights reserved.
 */

#include <IMP/core/AttributeSingletonScore.h>

#include <IMP/UnaryFunction.h>
#include <IMP/Particle.h>

#include <boost/tuple/tuple.hpp>

IMPCORE_BEGIN_NAMESPACE

AttributeSingletonScore::AttributeSingletonScore(UnaryFunction *f,
                                                 FloatKey k): f_(f),
                                                              k_(k){}

Float AttributeSingletonScore::evaluate(Particle *b,
                                        DerivativeAccumulator *da) const
{
  if (da) {
    Float v, d;
    boost::tie(v,d) = f_->evaluate_with_derivative(b->get_value(k_));
    b->add_to_derivative(k_, d, *da);
    return v;
  } else {
    return f_->evaluate(b->get_value(k_));
  }
}

void AttributeSingletonScore::do_show(std::ostream &out) const
{
  out << "function is " << *f_ << " on " << k_;
}

IMPCORE_END_NAMESPACE
