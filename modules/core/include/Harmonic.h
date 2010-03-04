/**
 *  \file Harmonic.h    \brief Harmonic function.
 *
 *  Copyright 2007-2010 Sali Lab. All rights reserved.
 */

#ifndef IMPCORE_HARMONIC_H
#define IMPCORE_HARMONIC_H

#include "core_config.h"
#include <IMP/UnaryFunction.h>
#include <IMP/utility.h>

IMPCORE_BEGIN_NAMESPACE

//! %Harmonic function (symmetric about the mean)
/** This is a simple score modeling an harmonic oscillator. The score is
    0.5 * k * x * x, where k is the 'force constant' and x is the distance
    from the mean.
    \see TruncatedHarmonic
    \see HarmonicUpperBound
    \see HarmonicLowerBound
 */
class Harmonic : public UnaryFunction
{
public:
  /** Create with the given mean and the spring constant k */
  Harmonic(Float mean, Float k) : mean_(mean), k_(k) {}

  IMP_UNARY_FUNCTION_INLINE(Harmonic,
                            0.5 * k_ * square(feature-mean_),
                            k_*(feature - mean_),
                            "Harmonic: " << mean_ << " and " << k_
                            << std::endl);

  //! \return the mean of this function
  Float get_mean() const {
    return mean_;
  }

  //! \return the spring constant
  Float get_k() const {
    return k_;
  }

  //! Set the mean of this function
  void set_mean(Float mean) {
    mean_ = mean;
  }

  //! Set the spring constant
  void set_k(Float k) {
    k_ = k;
  }


  //! Return the k to use for a given Gaussian standard deviation.
  /** Given the standard deviation of a Gaussian distribution, get
      the force constant of the harmonic score function that yields that
      same distribution. For temperature in Kelvin, this assumes the score
      function is energy in kcal/mol, and thus returns a force constant in
      kcal/mol/A/A.
      \param[in] sd  Gaussian standard deviation, in angstroms
      \param[in] t   System temperature, in Kelvin
      \return        Force constant
   */
  static Float k_from_standard_deviation(Float sd, Float t=297.15) {
    // Gas constant in kcal/mol K
    const static Float R = 8.31441 / 4186.8;
    return R * t / square(sd);
  }

private:
  Float mean_;
  Float k_;
};

IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_HARMONIC_H */
