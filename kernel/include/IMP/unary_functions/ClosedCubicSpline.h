/**
 *  \file ClosedCubicSpline.h    \brief Closed cubic spline function.
 *
 *  Copyright 2007-8 Sali Lab. All rights reserved.
 */

#ifndef __IMP_CLOSED_CUBIC_SPLINE_H
#define __IMP_CLOSED_CUBIC_SPLINE_H

#include "../UnaryFunction.h"

namespace IMP
{

//! Closed cubic spline function.
/** This function is defined by evenly-spaced spline values over a finite
    range, and is periodic.
 */
class IMPDLLEXPORT ClosedCubicSpline : public UnaryFunction
{
public:
  //! Constructor.
  /** \param[in] values   Score value at each spline point. Since the function
      is periodic, the score will also have the same value at
      minrange + spacing * values.size()
      \param[in] minrange Feature value at first spline point.
      \param[in] spacing  Distance (in feature space) between points
   */
  ClosedCubicSpline(const std::vector<Float> &values, Float minrange,
                    Float spacing);

  virtual ~ClosedCubicSpline() {}

  //! Calculate score with respect to the given feature.
  /** \param[in] feature Value of feature being tested.
      \exception ValueException Feature is out of defined range.
      \return Score
   */
  virtual Float evaluate(Float feature);

  //! Calculate score and derivative with respect to the given feature.
  /** \param[in] feature Value of feature being tested.
      \param[out] deriv Partial derivative of the score with respect to
                        the feature value.
      \exception ValueException Feature is out of defined range.
      \return Score
   */
  virtual Float evaluate_deriv(Float feature, Float& deriv);

  void show(std::ostream &out=std::cout) const {
    out << "Closed cubic spline of " << values_.size() << " values from "
        << minrange_ << " to " << maxrange_ << std::endl;
  }

protected:
  std::vector<Float> values_;
  std::vector<Float> second_derivs_;
  Float minrange_;
  Float maxrange_;
  Float spacing_;
};

} // namespace IMP

#endif  /* __IMP_CLOSED_CUBIC_SPLINE_H */
