/**
 *  \file Profile.h   \brief A class for profile storing and computation
 *
 *  Copyright 2007-2010 Sali Lab. All rights reserved.
 *
 */
#ifndef IMPSAXS_PROFILE_H
#define IMPSAXS_PROFILE_H

#include "saxs_config.h"
#include "FormFactorTable.h"
#include "Distribution.h"

#include <IMP/Model.h>

#include <iostream>
#include <vector>

IMPSAXS_BEGIN_NAMESPACE

class RadialDistributionFunction;

/**
   Basic profile class, can be initialized from the input file
   (experimental or theoretical) or computed from a set of Model
   Particles (theoretical)
*/
class IMPSAXSEXPORT Profile {
public:
  //! init from file
  Profile(const String& file_name);

  //! init for theoretical profile
  Profile(Float qmin = 0.0, Float qmax = 0.5, Float delta = 0.005);

private:
  class IntensityEntry {
  public:
    IntensityEntry() : q_(0.0), intensity_(0.0), error_(1.0), weight_(1.0) {}
    IntensityEntry(Float q) : q_(q),intensity_(0.0),error_(1.0),weight_(1.0) {}
    IntensityEntry(Float q, Float intensity, Float error)
      : q_(q), intensity_(intensity), error_(error), weight_(1.0) {}

    Float q_;
    Float intensity_;
    Float error_;
    Float weight_;
  };

  friend std::ostream& operator<<(std::ostream& q, const IntensityEntry& e);

  friend std::istream& operator>>(std::istream& q, IntensityEntry& e);

public:
  //! computes theoretical profile
  void calculate_profile(const Particles& particles,
                         bool reciprocal=false, bool autocorrelation = true) {
    if(!reciprocal) calculate_profile_real(particles, autocorrelation);
    else calculate_profile_reciprocal(particles);
  }

  //! computes theoretical profile faster for cyclically symmetric particles
  //! assumes that the units particles are ordered one after another in the
  //! input particles vector (n - symmetry order)
  void calculate_profile(const Particles& particles, unsigned int n) {
    calculate_profile_real(particles, n);
  }

  //! compute profile for fitting with hydration layer and excluded volume
  void calculate_profile_partial(const Particles& particles,
                                 const Floats& surface = Floats(),
                                 bool autocorrelation = true);

  void calculate_profile_partial(const Particles& particles1,
                                 const Particles& particles2);

  //! computes theoretical profile contribution from iter-molecular
  //! interactions between the particles
  void calculate_profile(const Particles& particles1,
                         const Particles& particles2) {
    calculate_profile_real(particles1, particles2);
  }

  //! convert to real space P(r) function P(r) = 1/2PI^2 Sum(I(q)*qr*sin(qr))
  void profile_2_distribution(RadialDistributionFunction& rd,
                              Float max_distance) const;

  //! convert to reciprocal space I(q) = Sum(P(r)*sin(qr)/qr)
  void distribution_2_profile(const RadialDistributionFunction& r_dist);

  //! add another profile - useful for rigid bodies
  void add(const Profile& other_profile);

  //! add partial profiles
  void add_partial_profiles(const Profile& other_profile);

  //! background adjustment option
  void background_adjust(double start_q);

  //! scale
  void scale(Float c);

  //! offset profile by c, I(q) = I(q) - c
  void offset(Float c);

  //! reads SAXS profile from file
  void read_SAXS_file(const String& file_name);

  //! print to file
  void write_SAXS_file(const String& file_name);

  //! return sampling resolution
  Float get_delta_q() const { return delta_q_; }

  //! return minimal sampling point
  Float get_min_q() const { return min_q_; }

  //! return maximal sampling point
  Float get_max_q() const { return max_q_; }

  //! return number of entries in SAXS profile
  unsigned int size() const { return profile_.size(); }

  Float get_intensity(unsigned int i) const { return profile_[i].intensity_; }
  Float get_q(unsigned int i) const { return profile_[i].q_; }
  Float get_error(unsigned int i) const { return profile_[i].error_; }
  Float get_weight(unsigned int i) const { return profile_[i].weight_; }

  void set_intensity(unsigned int i, Float iq) { profile_[i].intensity_ = iq; }

  //! add intensity entry to profile
  void add_entry(Float q, Float intensity, Float error=1.0) {
    profile_.push_back(IntensityEntry(q, intensity, error));
  }

  //! checks the sampling of experimental profile
  bool is_uniform_sampling() const;

  //! add simulated error
  void add_errors();

  //! computes full profile for given fitting parameters
  void sum_partial_profiles(Float c1, Float c2, Profile& out_profile);

  // parameter for E^2(q), used in faster calculation
  static const Float modulation_function_parameter_;

 private:
  void init();

  void calculate_profile_reciprocal(const Particles& particles,
                                    bool autocorrelation = true);

  void calculate_profile_reciprocal(const Particles& particles1,
                                    const Particles& particles2);

  void calculate_profile_real(const Particles& particles,
                              bool autocorrelation = true);

  void calculate_profile_real(const Particles& particles1,
                              const Particles& particles2);

  // // for symmetry
  // void calculate_profile_real(const Particles& particles,
  //                             unsigned int n);


  void squared_distribution_2_profile(const RadialDistributionFunction& r_dist);
  void squared_distributions_2_partial_profiles(
                         const std::vector<RadialDistributionFunction>& r_dist);

 protected:
  std::vector<IntensityEntry> profile_; // the profile
  Float min_q_, max_q_; // minimal and maximal s values  in the profile
  Float delta_q_; // profile sampling resolution
  FormFactorTable* ff_table_; // pointer to form factors table
  std::vector<Profile> partial_profiles_;
  bool experimental_; // experimental profile read from file
};

IMPSAXS_END_NAMESPACE

#endif /* IMPSAXS_PROFILE_H */
