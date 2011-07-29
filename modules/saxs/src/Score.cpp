/**
 *  \file Score.h   \brief A class for profile storing and computation
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 *
 */
#include <IMP/saxs/Score.h>
#include <IMP/algebra/utility.h>
#include <IMP/saxs/utility.h>

#include <map>

IMPSAXS_BEGIN_NAMESPACE

Score::Score(const Profile& exp_profile) : exp_profile_(exp_profile)
{}

void Score::resample(const Profile& model_profile,
                     Profile& resampled_profile) const
{
  // map of q values for fast search
  std::map<float, unsigned int> q_mapping;
  for (unsigned int k=0; k<model_profile.size(); k++) {
    q_mapping[model_profile.get_q(k)] = k;
  }

  for (unsigned int k=0; k<exp_profile_.size(); k++) {
    Float q = exp_profile_.get_q(k);
    std::map<float, unsigned int>::iterator it = q_mapping.lower_bound(q);
    if(it == q_mapping.end()) break;
    unsigned int i = it->second;
    if(i == 0) {
      resampled_profile.add_entry(q, model_profile.get_intensity(i));
    } else {
      Float delta_q = model_profile.get_q(i)-model_profile.get_q(i-1);
      if(delta_q <= 1.0e-16) {
        resampled_profile.add_entry(q, model_profile.get_intensity(i));
      } else {
        Float alpha = (q - model_profile.get_q(i-1)) / delta_q;
        if(alpha > 1.0) alpha = 1.0; // handle rounding errors
        Float intensity = model_profile.get_intensity(i-1)
          + (alpha)*(model_profile.get_intensity(i)
                     - model_profile.get_intensity(i-1));
        resampled_profile.add_entry(q, intensity);
      }
    }
  }
}

Float Score::compute_scale_factor(const Profile& model_profile,
                                  const Float offset) const
{
  Float sum1=0.0, sum2=0.0;
  unsigned int profile_size = std::min(model_profile.size(),
                                       exp_profile_.size());
  for (unsigned int k=0; k<profile_size; k++) {
    Float square_error = square(exp_profile_.get_error(k));
    Float weight_tilda = model_profile.get_weight(k) / square_error;

    sum1 += weight_tilda * model_profile.get_intensity(k)
                         * (exp_profile_.get_intensity(k) + offset);
    sum2 += weight_tilda * square(model_profile.get_intensity(k));
  }
  // std::cerr << "c = " << sum1 / sum2 << std::endl;
  return sum1 / sum2;
}

Float Score::compute_offset(const Profile& model_profile) const {
  Float sum_iexp_imod=0.0, sum_imod=0.0, sum_iexp=0.0, sum_imod2=0.0;
  Float sum_weight=0.0;
  unsigned int profile_size = std::min(model_profile.size(),
                                       exp_profile_.size());
  for (unsigned int k=0; k<profile_size; k++) {
    Float square_error = square(exp_profile_.get_error(k));
    Float weight_tilda = model_profile.get_weight(k) / square_error;

    sum_iexp_imod += weight_tilda * model_profile.get_intensity(k)
                                  * exp_profile_.get_intensity(k);
    sum_imod += weight_tilda * model_profile.get_intensity(k);
    sum_iexp += weight_tilda * exp_profile_.get_intensity(k);
    sum_imod2 += weight_tilda * square(model_profile.get_intensity(k));
    sum_weight += weight_tilda;
  }
  Float offset = sum_iexp_imod / sum_imod2 * sum_imod - sum_iexp;
  offset /= (sum_weight - sum_imod*sum_imod/sum_imod2);
  return offset;
}


Float Score::fit_profile(Profile partial_profile, float& C1, float& C2,
                         bool fixed_c1, bool fixed_c2, bool use_offset,
                         const std::string fit_file_name) const {
  // compute chi value for default c1/c1 (remove?)
  float default_c1 = 1.0, default_c2 = 0.0;
  partial_profile.sum_partial_profiles(default_c1, default_c2, partial_profile);
  float default_chi = compute_chi_score(partial_profile, use_offset);

  float best_c1(default_c1), best_c2(default_c2), best_chi(default_chi);
  bool best_set = false;

  // set up the range for c1 and c2
  float min_c1 = 0.95, max_c1 = 1.12;
  float min_c2 = -4.0, max_c2 = 4.0, delta_c2 = 0.1;
  if(fixed_c1) {
    min_c1 = max_c1 = C1;
    delta_c2 = 0.01; // finer c2 enumeration
  }
  if(fixed_c2) {
    min_c2 = max_c2 = C2;
  }

  // enumerate
  for(float c1 = min_c1; c1<=max_c1; c1+= 0.005) {
    for(float c2 = min_c2; c2<=max_c2; c2+= delta_c2) {
      partial_profile.sum_partial_profiles(c1, c2, partial_profile);
      float curr_chi = compute_chi_score(partial_profile, use_offset);
      if(!best_set || curr_chi < best_chi) {
        best_set = true;
        best_chi = curr_chi;
        best_c1 = c1;
        best_c2 = c2;
      }
      //std::cerr << "c1 = " << c1 << " c2 = " << c2
      //<< " chi = " << curr_chi << std::endl;
    }
  }

  // compute a profile for best c1/c2 combination
  partial_profile.sum_partial_profiles(best_c1, best_c2, partial_profile);
  compute_chi_score(partial_profile, use_offset, fit_file_name);
  if(!fixed_c1) C1 = best_c1;
  if(!fixed_c2) C2 = best_c2;

  std::cout << " Chi = " << best_chi << " c1 = " << best_c1 << " c2 = "
            << best_c2 << " default chi = " << default_chi << std::endl;
  return best_chi;
}

Float Score::compute_chi_square_score(const Profile& model_profile,
                                      bool use_offset,
                                      const std::string fit_file_name) const
{
  Profile resampled_profile(exp_profile_.get_min_q(),
                            exp_profile_.get_max_q(),
                            exp_profile_.get_delta_q());
  resample(model_profile, resampled_profile);
  return compute_chi_square_score_internal(
                            resampled_profile, fit_file_name, use_offset);
}

Float Score::compute_chi_square_score_internal(
                             const Profile& model_profile,
                             const std::string& fit_file_name,
                             bool use_offset) const
{
  Float offset = 0.0;
  if(use_offset) offset = compute_offset(model_profile);
  Float c = compute_scale_factor(model_profile, offset);
  Float chi_square =
    compute_chi_square_score_internal(model_profile, c, offset);

  if(fit_file_name.length() > 0) {
    write_SAXS_fit_file(fit_file_name, model_profile,
                        chi_square, c, offset);
  }
  return chi_square;
}

/*
compute SAXS Chi square of experimental data and model
weight_tilda function w_tilda(q) = w(q) / sigma_exp^2
*/
// TODO: define a weight function (w(q) = 1, q^2, or hybrid)
Float Score::compute_chi_square_score_internal(
                             const Profile& model_profile,
                             const Float c, const Float offset) const
{
  Float chi_square = 0.0;
  unsigned int profile_size = std::min(model_profile.size(),
                                       exp_profile_.size());
  // compute chi square
  for (unsigned int k=0; k<profile_size; k++) {
    // in the theoretical profile the error equals to 1
    Float square_error = square(exp_profile_.get_error(k));
    Float weight_tilda = model_profile.get_weight(k) / square_error;
    Float delta = exp_profile_.get_intensity(k) + offset
                    - c * model_profile.get_intensity(k);

    // Exclude the uncertainty originated from limitation of floating number
    if (fabs(delta/exp_profile_.get_intensity(k)) >= IMP_SAXS_DELTA_LIMIT)
      chi_square += weight_tilda * square(delta);
  }
  chi_square /= profile_size;
  return chi_square;
}


void Score::write_SAXS_fit_file(const std::string& file_name,
                                const Profile& model_profile,
                                const Float chi_square,
                                const Float c, const Float offset) const {
  std::ofstream out_file(file_name.c_str());
  if (!out_file) {
    IMP_THROW("Can't open file " << file_name,
              IOException);
  }

  unsigned int profile_size = std::min(model_profile.size(),
                                       exp_profile_.size());
  // header line
  out_file.precision(15);
  out_file << "# SAXS profile: number of points = " << profile_size
           << ", q_min = " << exp_profile_.get_min_q()
           << ", q_max = " << exp_profile_.get_max_q();
  out_file << ", delta_q = " << exp_profile_.get_delta_q() << std::endl;

  out_file.setf(std::ios::showpoint);
  out_file << "# offset = " << offset << ", scaling c = " << c
           << ", Chi = " << sqrt(chi_square) << std::endl;
  out_file << "#  q       exp_intensity   model_intensity"
           << std::endl;

  out_file.setf(std::ios::fixed, std::ios::floatfield);
  // Main data
  for (unsigned int i = 0; i < profile_size; i++) {
    out_file.setf(std::ios::left);
    out_file.width(10);
    out_file.precision(5);
    out_file << exp_profile_.get_q(i) << " ";

    out_file.setf(std::ios::left);
    out_file.width(15);
    out_file.precision(8);
    out_file << exp_profile_.get_intensity(i)  << " ";

    out_file.setf(std::ios::left);
    out_file.width(15);
    out_file.precision(8);
    out_file << model_profile.get_intensity(i)*c - offset << std::endl;
  }
  out_file.close();
}

IMPSAXS_END_NAMESPACE
