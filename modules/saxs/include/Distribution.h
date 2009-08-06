/**
 * \file Distribution.h \brief computes distribution functions
 *
 * Distribution - base distance distribution class
 * RadialDistributionFunction required for calculation of SAXS profile
 * DeltaDistributionFunction requires for chi-square derivatives
 *
 * Copyright 2007-9 Sali Lab. All rights reserved.
 *
 */

#ifndef IMPSAXS_DISTRIBUTION_H
#define IMPSAXS_DISTRIBUTION_H

#include "config.h"
#include "FormFactorTable.h"
#include "Profile.h"
#include "Score.h"
#include <IMP/algebra/utility.h>
#include <IMP/Particle.h>

#include <iostream>
#include <vector>

IMPSAXS_BEGIN_NAMESPACE

namespace { // anonymous
  static const Float pr_resolution = 0.5;
}

/**
base class for distribution classes
*/
template<class ValueT>
class Distribution : public std::vector< ValueT > {
public:
  //! Constructor
  Distribution(Float bin_size = pr_resolution) { init(bin_size); }

  //! returns maximal distance value of distribution
  Float get_max_distance() { return max_distance_; }

  //! returns bin size
  Float get_bin_size() { return bin_size_; }

protected:
  void init(Float bin_size) {
    //  clear();
    bin_size_ = bin_size;
    one_over_bin_size_ = 1.0/bin_size_;     // for faster calculation
    max_distance_ = 50.0;      // start with ~50A (by default)
    reserve(dist2index(max_distance_) + 1);
  }
  unsigned int dist2index(Float dist) const {
    return algebra::round( dist * one_over_bin_size_ );
  }
  Float index2dist(unsigned int index) const { return index * bin_size_; }
protected:
  Float bin_size_, one_over_bin_size_; // resolution of discretization
  Float max_distance_;  // parameter for maximum r value for p(r) function
};

/**
 Radial Distribution class for calculating SAXS Profile
 this is distance distribution multiplied by form factors of atoms
*/
class IMPSAXSEXPORT RadialDistributionFunction : public Distribution<Float> {

public:
  //! Constructor (default)
  RadialDistributionFunction(FormFactorTable* ff_table =
                             default_form_factor_table(),
                             Float bin_size = pr_resolution);

  //! Constructor from gnom file \untested{read_pr_file}
  RadialDistributionFunction(const std::string& file_name);

  friend class Profile;

  //! computes radial distribution function for a set of particles
  void calculate_distribution(const Particles& particles,
                              bool autocorrelation = true);

  //! computes distribution contribution from inter-molecular
  //! interactions between the particles
  void calculate_distribution(const Particles& particles1,
                              const Particles& particles2);

  //! the distribution is a function of squared distance
  void calculate_squared_distribution(const Particles& particles,
                                      bool autocorrelation = true);

  //! the distribution is a function of squared distance
  void calculate_squared_distribution(const Particles& particles1,
                                      const Particles& particles2);

  //! scale distribution by a constant
  void scale(Float c);

  //! add another distribution
  void add(const RadialDistributionFunction& model_pr);

  //! print tables
  void show(std::ostream &out=std::cout, std::string prefix="") const;

  //! analogy crystallographic R-factor score \untested{R_factor}
  Float R_factor_score(const RadialDistributionFunction& model_pr);

  //! analogy to chi score \untested{chi_score}
  Float chi_score(const RadialDistributionFunction& model_pr);

  //! write fit file for the two distributions
  void write_fit_file(const std::string& file_name,
                      const RadialDistributionFunction& model_pr);

  //! normalize to area = 1.0
  void normalize();

 private:
  void add_to_distribution(Float dist, Float value) {
    unsigned int index = dist2index(dist);
    if(index >= size()) {
      if(capacity() <= index)
        reserve(2 * index);   // to avoid many re-allocations
      resize(index + 1, 0);
      max_distance_ = index2dist(index + 1);
    }
    (*this)[index] += value;
  }

  void read_pr_file(const std::string& file_name);

 protected:
  FormFactorTable* ff_table_; // pointer to form factors table
};


/**
Delta Distribution class for calculating the derivatives of SAXS Score
this distribution is:
sum_i [f_p(0) * f_i(0) * (x_p - x_i)]
sum_i [f_p(0) * f_i(0) * (y_p - y_i)]
sum_i [f_p(0) * f_i(0) * (z_p - z_i)]
*/
class IMPSAXSEXPORT
DeltaDistributionFunction : public Distribution<algebra::Vector3D> {
public:
  //! Constructor
  DeltaDistributionFunction(FormFactorTable* ff_table,
                            const Particles& particles,
                            Float max_distance = 0.0,
                            Float bin_size = pr_resolution);

  friend class Score;

  //! calculates distribution for an atom defined by particle
  void calculate_derivative_distribution(Particle* particle);

  //! print tables
  void show(std::ostream &out=std::cout, std::string prefix="") const;

 private:
  void add_to_distribution(Float dist, const algebra::Vector3D& value) {
    unsigned int index = dist2index(dist);
    if(index >= size()) {
      if(capacity() <= index)
        reserve(2 * index);   // to avoid many re-allocations
      resize(index + 1, algebra::Vector3D(0.0, 0.0, 0.0));
      max_distance_ = index2dist(index + 1);
    }
    (*this)[index] += value;
  }

  void init() {
    clear();
    insert(begin(), dist2index(max_distance_) + 1,
           algebra::Vector3D(0.0, 0.0, 0.0));
  }

 protected:
  FormFactorTable* ff_table_; // pointer to form factors table
  std::vector<algebra::Vector3D> coordinates_;
  Floats form_factors_;
};

IMPSAXS_END_NAMESPACE

#endif /* IMPSAXS_DISTRIBUTION_H */
