/**
 *  \file OpenCubicSpline.cpp  \brief Open cubic spline function.
 *
 *  Copyright 2007-2010 Sali Lab. All rights reserved.
 *
 */

#include <IMP/core/OpenCubicSpline.h>

IMPCORE_BEGIN_NAMESPACE

OpenCubicSpline::OpenCubicSpline(const std::vector<Float> &values,
                                 Float minrange, Float spacing,
                                 bool extend) :
  values_(values), minrange_(minrange), spacing_(spacing),
  extend_(extend)
{
  IMP_USAGE_CHECK(spacing >0, "The spacing between values must be positive.");
  IMP_USAGE_CHECK(values.size() >=1, "You must provide at least one value.");
  int npoints = values_.size();
  maxrange_ = minrange_ + spacing_ * (npoints - 1);

  // Precalculate second derivatives for a natural cubic spline (open) by
  // inversion of the tridiagonal matrix (Thomas algorithm)
  second_derivs_.resize(npoints);
  std::vector<Float> tmp(npoints);

  // Forward elimination phase
  second_derivs_[0] = 0.;
  tmp[0] = 0.;

  Float doublespacing = spacing + spacing;
  for (int i = 1; i < npoints - 1; ++i) {
    Float m = 0.5 * second_derivs_[i - 1] + 2.;
    second_derivs_[i] = -0.5 / m;
    tmp[i] = (6. * ((values_[i + 1] - values_[i]) / spacing_
                    - (values_[i] - values_[i - 1]) / spacing) / doublespacing
              - 0.5 * tmp[i - 1]) / m;
  }

  // Backward substitution phase
  second_derivs_[npoints - 1] = 0.;
  for (int i = npoints - 2; i >= 0; --i) {
    second_derivs_[i] = second_derivs_[i] * second_derivs_[i + 1] + tmp[i];
  }
}


double OpenCubicSpline::evaluate(double feature) const
{
  // check for feature in range
  if (feature < minrange_ || feature > maxrange_) {
    if (extend_) {
      if (feature < minrange_) return values_.front();
      else return values_.back();
    } else {
      throw ValueException("Value out of range for open cubic spline");
    }
  }

  // determine bin index and thus the cubic fragment to use:
  size_t lowbin = static_cast<size_t>((feature - minrange_) / spacing_);
  // handle the case where feature ~= maxrange
  lowbin = std::min(lowbin, values_.size() - 2);
  size_t highbin = lowbin + 1;
  double lowfeature = minrange_ + lowbin * spacing_;

  double b = (feature - lowfeature) / spacing_;
  double a = 1. - b;

  return a * values_[lowbin] + b * values_[highbin]
         + ((a * (a * a - 1.)) * second_derivs_[lowbin]
            + (b * (b * b - 1.)) * second_derivs_[highbin])
           * (spacing_ * spacing_) / 6.;
}

DerivativePair OpenCubicSpline::evaluate_with_derivative(double feature) const
{
  size_t lowbin = static_cast<size_t>((feature - minrange_) / spacing_);
  // handle the case where feature ~= maxrange
  lowbin = std::min(lowbin, values_.size() - 2);
  size_t highbin = lowbin + 1;
  double lowfeature = minrange_ + lowbin * spacing_;

  double b = (feature - lowfeature) / spacing_;
  double a = 1. - b;
  double sixthspacing = spacing_ / 6.;

  double deriv = (values_[highbin] - values_[lowbin]) / spacing_
    - (3. * a * a - 1.) * sixthspacing * second_derivs_[lowbin]
    + (3. * b * b - 1.) * sixthspacing * second_derivs_[highbin];

  return std::make_pair(evaluate(feature), deriv);
}

void OpenCubicSpline::do_show(std::ostream &out) const {
    out << values_.size() << " values from "
        << minrange_ << " to " << maxrange_ << std::endl;
}

IMPCORE_END_NAMESPACE
