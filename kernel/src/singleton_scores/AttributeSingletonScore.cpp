/**
 *  \file AttributeSingletonScore.cpp
 *  \brief A score based on an unmodified attribute value.
 *
 *  Copyright 2007-8 Sali Lab. All rights reserved.
 */

#include "IMP/singleton_scores/AttributeSingletonScore.h"
#include "IMP/UnaryFunction.h"
#include "IMP/Particle.h"
#include <boost/tuple/tuple.hpp>

IMP_BEGIN_NAMESPACE

AttributeSingletonScore::AttributeSingletonScore(UnaryFunction *f,
                                                 FloatKey k): f_(f),
                                                              k_(k){}

Float AttributeSingletonScore::evaluate(Particle *b,
                                        DerivativeAccumulator *da) const
{
  if (da) {
    Float d, r;
    boost::tie(d,r) = f_->evaluate_with_derivative(b->get_value(k_));
    b->add_to_derivative(k_, d, *da);
    return r;
  } else {
    return f_->evaluate(b->get_value(k_));
  }
}

void AttributeSingletonScore::show(std::ostream &out) const
{
  out << "AttributeSingletonScore using ";
  f_->show(out);
  out << " on " << k_;
}

IMP_END_NAMESPACE
