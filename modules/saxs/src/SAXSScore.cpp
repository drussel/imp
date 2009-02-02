/**
 *  \file SAXSScore.h   \brief A class for profile storing and computation
 *
 *  Copyright 2007-8 Sali Lab. All rights reserved.
 *
 */
#include <IMP/saxs/SAXSScore.h>

#define IMP_SAXS_DELTA_LIMIT  1.0e-15

IMPSAXS_BEGIN_NAMESPACE

SAXSScore::SAXSScore(FormFactorTable* ff_table,
                     SAXSProfile* exp_saxs_profile,
                     const std::vector<Particle*>& particles) :
  ff_table_(ff_table), exp_saxs_profile_(exp_saxs_profile)
{
  init(particles);
}


/*
 !----------------------------------------------------------------------
 !------------ Precalculate common array data for faster calculation
 !----------------------------------------------------------------------
*/
// TODO: need to get a number of distribution index (nr) and bin_size (dr)
// from RadialDistirubionFunction class
int SAXSScore::init(const std::vector<Particle*>& particles)
{
  //!------ setup lookup tables for sinc and cosine function
  nr_ = 100;
  dr_ = 0.5;
  mesh_sinc_ = 1000;
  offset_ = 0.0;
  int nsinc = (int)( exp_saxs_profile_->get_s( exp_saxs_profile_->size()-1 )
                    * nr_ * dr_ * mesh_sinc_ ) + 1;
  const double sinc_dense = 1.0 / (double)mesh_sinc_;
  sinc_lookup_.clear();   sinc_lookup_.resize(nsinc, 0.0);
  cos_lookup_.clear();    cos_lookup_.resize(nsinc, 0.0);
  // to avoid the singularity of sinc function at zero
  sinc_lookup_.push_back(1.0);  cos_lookup_.push_back(1.0);
  for (int i=1; i<nsinc; i++) {
    double x = i * sinc_dense;
    sinc_lookup_[i] = sin(x) / x;
    cos_lookup_[i] = cos(x);
  }

  r_.clear();   r_.resize(nr_, 0.0);
  r_square_reciprocal_.clear();     r_square_reciprocal_.resize(nr_, 0.0);
  for (unsigned int i=0; i<nr_; i++) {
    r_[i] = dr_ * i;
    r_square_reciprocal_[i] = 1.0 / square(r_[i]);
  }

  sincval_array_.clear(); sincval_array_.resize(exp_saxs_profile_->size());
  std::vector<double> sincval_temp;
  sincval_temp.resize(nr_, 0.0);
  for (unsigned int is=0; is<exp_saxs_profile_->size(); is++) {
    for (unsigned int ir=0; ir<nr_; ir++) {
      double qr = exp_saxs_profile_->get_s(is) * r_[ir];
      int ilookup = (int)(mesh_sinc_ * qr + 0.5);
      sincval_temp[ir] = sinc_lookup_[ilookup] - cos_lookup_[ilookup];
    }
    sincval_array_[is] = sincval_temp;
  }

  // Pre-store zero_formfactor for all particles, for the faster calculation
  zero_formfactor_.clear();   zero_formfactor_.resize(particles.size(), 0.0);
  for (unsigned int iatom=0; iatom<particles.size(); iatom++)
    zero_formfactor_[iatom] = ff_table_->get_form_factor(particles[iatom]);

  return 0;
}


/*
 ! ----------------------------------------------------------------------
 !------------ scoring function with weighting by error      ------------
 ! calculate SAXS Chi square of experimental data and model
 ! added weight_tilda function w_tilda(q) = w(q) / sigma_exp^2
 ! ----------------------------------------------------------------------
*/
// TODO: add some tests for the same sampling for both profiles
// TODO: define a weight function (w(q) = 1, q^2, or hybrid)
double SAXSScore::compute_chi_score(const SAXSProfile& model_saxs_profile)
{
  if (exp_saxs_profile_->size() != model_saxs_profile.size()) {
    printf("Number of profile entries mismatch! (exp:%d, model:%d)\n",
           exp_saxs_profile_->size(), model_saxs_profile.size());
    return -1.0;
  }
  double sum1=0.0, sum2=0.0, chi_square=0.0;

  //! determine the scaling parameter c_
  for (unsigned int k=0; k<model_saxs_profile.size(); k++) {
    //! in the theoretical profile the error equals to 1 (?)
    double square_error = square(exp_saxs_profile_->get_error(k));
    double weight_tilda = model_saxs_profile.get_weight(k) / square_error;

    sum1 += weight_tilda * model_saxs_profile.get_intensity(k)
                         * exp_saxs_profile_->get_intensity(k);
    sum2 += weight_tilda * square(model_saxs_profile.get_intensity(k));
  }
  c_ = sum1 / sum2;

  //! compute Chi square
  for (unsigned int k=0; k<model_saxs_profile.size(); k++) {
    //! in the theoretical profile the error equals to 1 (?)
    double square_error = square(exp_saxs_profile_->get_error(k));
    double weight_tilda = model_saxs_profile.get_weight(k) / square_error;
    double delta = exp_saxs_profile_->get_intensity(k) + offset_
                    - c_ * model_saxs_profile.get_intensity(k);

    // Exclude the uncertainty originated from limitation of floating number
    if (fabs(delta/exp_saxs_profile_->get_intensity(k)) >= IMP_SAXS_DELTA_LIMIT)
      chi_square += weight_tilda * square(delta);
  }
  //chi /= model_saxs_profile.size();

  //! TODO: make this optional
  write_SAXS_fit_file("fitfile.dat", model_saxs_profile);

  return chi_square;
  //return sqrt(chi);
}


/*
 ! ----------------------------------------------------------------------
 !>   compute  derivatives on atom iatom - iatom is NOT part of rigid body
 !!   SCORING function : chi
 !!   This routine does the computations in REAL SPACE!
 !!          => FAST computations!
 !!
 !!   For calculation in real space the quantity Delta(r) is needed to get
 !!   derivatives on atom iatom
 !!
 !!   Delta(r) = f_iatom * sum_i f_i delta(r-r_{i,iatom}) (x_iatom-x_i)
 ! ----------------------------------------------------------------------
*/
// TODO: Combine with "RadialDistribution Class"
// TODO: s and q relationship? (Emailed to Frido)
std::vector<IMP::algebra::Vector3D> SAXSScore::calculate_chi_real_derivative (
                                       const SAXSProfile& model_saxs_profile,
                                       const std::vector<Particle*>& particles)
{
  std::vector<algebra::Vector3D> chi_derivatives;
  unsigned int i, is, iatom, ir;

  if (exp_saxs_profile_->size() != model_saxs_profile.size()) {
    printf("Number of profile entries mismatch! (exp:%d, model:%d)\n",
           exp_saxs_profile_->size(), model_saxs_profile.size());
    return chi_derivatives;
  }

  //!------ precalculate difference of intensities and squares of weight
  std::vector<double> delta_i, e_q, delta_i_and_e_q;
  delta_i.resize(model_saxs_profile.size(), 0.0);
  e_q.resize(model_saxs_profile.size(), 0.0);
  delta_i_and_e_q.resize(model_saxs_profile.size(), 0.0);
  for (is=0; is<model_saxs_profile.size(); is++) {
    double delta = exp_saxs_profile_->get_intensity(is)
                  - c_ * model_saxs_profile.get_intensity(is);
    double square_error = square(exp_saxs_profile_->get_error(is));
    double weight_tilda = model_saxs_profile.get_weight(is) / square_error;

    // Exclude the uncertainty originated from limitation of floating number
    if (fabs(delta/exp_saxs_profile_->get_intensity(is)) < IMP_SAXS_DELTA_LIMIT)
      delta = 0.0;
    delta_i[is] = weight_tilda * delta;
    e_q[is] = exp( -0.23 * square(exp_saxs_profile_->get_s(is)) );
    delta_i_and_e_q[is] = delta_i[is] * e_q[is];
  }
  //!------ copy coordinates in advance, to avoid n^2 copy operations
  std::vector<algebra::Vector3D> coordinates;
  coordinates.resize(particles.size());
  for (is=0; is<particles.size(); is++)
    coordinates[is] = core::XYZDecorator::cast(particles[is]).get_coordinates();

  const double dr_reciprocal = 1.0 / dr_;

  std::vector<algebra::Vector3D> Delta;
  algebra::Vector3D Delta_q(0.0, 0.0, 0.0), chi_derivative(0.0, 0.0, 0.0);
  // TODO: Very weird. Why did definition make huge differnce on Mac?
  std::vector<double> Delta_x;//, Delta_y, Delta_z;

  //!------ The real loop starts from here
  chi_derivatives.resize(particles.size());
  for (iatom=0; iatom<particles.size(); iatom++) {
    // TODO: Use Vector3D for simplicity.
    // But accuracy & performance are still problematic at this moment.
    Delta.clear();
    Delta.resize(nr_, algebra::Vector3D(0.0, 0.0, 0.0));

    //!------ precalculate difference delta
    for (i=0; i<particles.size(); i++) {
      if (i != iatom) {
        double dist = distance(coordinates[iatom], coordinates[i]);
        unsigned int ir = (unsigned int)(dist * dr_reciprocal + 0.5);
        double temp = zero_formfactor_[iatom] * zero_formfactor_[i]
                      * r_square_reciprocal_[ir];
        Delta[ir] += temp * (coordinates[iatom] - coordinates[i]);
      }
    }

    //!------ core of chi derivatives
    chi_derivative[0] = chi_derivative[1] = chi_derivative[2] = 0.0;
    for (is=0; is<model_saxs_profile.size(); is++) {
      Delta_q[0] = Delta_q[1] = Delta_q[2] = 0.0;

      for (ir=0; ir<nr_; ir++) {
        //!------ sincval_array_[][] = sinc_lookup_[] - cos_lookup_[]
        Delta_q += Delta[ir] * sincval_array_[is][ir];
      }
      //!----- delta_i_and_e_q[] = delta_i[] * e_q[]
      chi_derivative += Delta_q * delta_i_and_e_q[is];
    }
    chi_derivatives[iatom] = 4.0 * c_ * chi_derivative;
  }
  return chi_derivatives;
}


//!----------------------------------------------------------------------
//! TODO: This is a C-style file print, for the alignment of data like Modeller
//!----------------------------------------------------------------------
void SAXSScore::write_SAXS_fit_file(const std::string& file_name,
                                    const SAXSProfile& model_saxs_profile)
const {
  std::FILE *fp;
  fp = fopen(file_name.c_str(), "w");
  fprintf(fp, "# SAXS fit_file: number of points = %d",
          exp_saxs_profile_->size());
  fprintf(fp, " s_min = %.15g, s_max = %.15g, delta = %.16g\n",
          exp_saxs_profile_->get_min_s(),
          exp_saxs_profile_->get_max_s(),
          exp_saxs_profile_->get_delta_s());
  fprintf(fp, "# offset = %.16g, scaling c = %.16g\n", offset_, c_);
  fprintf(fp, "#        s             exp_intensity         model_intensity\n");

  for (unsigned int i = 0; i < exp_saxs_profile_->size(); i++) {
    fprintf(fp, "%22.15g  %22.15g  %22.16g\n",
            exp_saxs_profile_->get_s(i),
            exp_saxs_profile_->get_intensity(i) + offset_,
            model_saxs_profile.get_intensity(i) * c_);
  }
  fclose(fp);
}

IMPSAXS_END_NAMESPACE
