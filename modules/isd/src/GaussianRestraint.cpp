/**
 *  \file isd/GaussianRestraint.cpp
 *  \brief Restrain a list of particle pairs with a lognormal+ISPA.
 *  NOTE: for now, the derivatives are written to all variables.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 *
 */

#include <IMP/isd/GaussianRestraint.h>
#include <IMP/isd/FNormal.h>
#include <IMP/isd/Nuisance.h>
#include <IMP/isd/Scale.h>

IMPISD_BEGIN_NAMESPACE

  GaussianRestraint::GaussianRestraint(Particle *x, Particle *mu,
          Particle *sigma) : px_(x), pmu_(mu), psigma_(sigma), isx_(true),
    ismu_(true), issigma_(true) {check_particles();}

  GaussianRestraint::GaussianRestraint(double x, Particle *mu,
          Particle *sigma) : x_(x), pmu_(mu), psigma_(sigma), isx_(false),
    ismu_(true), issigma_(true) {check_particles();}

  GaussianRestraint::GaussianRestraint(Particle *x, double mu,
          Particle *sigma) : px_(x), mu_(mu), psigma_(sigma), isx_(true),
    ismu_(false), issigma_(true) {check_particles();}

  GaussianRestraint::GaussianRestraint(Particle *x, Particle *mu,
          double sigma) : px_(x), pmu_(mu), sigma_(sigma), isx_(true),
    ismu_(true), issigma_(false) {check_particles();}

  GaussianRestraint::GaussianRestraint(Particle *x, double mu,
          double sigma) : px_(x), mu_(mu), sigma_(sigma), isx_(true),
    ismu_(false), issigma_(false) {check_particles();}

  GaussianRestraint::GaussianRestraint(double x, double mu,
          Particle *sigma) : x_(x), mu_(mu), psigma_(sigma), isx_(false),
    ismu_(false), issigma_(true) {check_particles();}

  GaussianRestraint::GaussianRestraint(double x, Particle *mu,
          double sigma) : x_(x), pmu_(mu), sigma_(sigma), isx_(false),
    ismu_(true), issigma_(false) {check_particles();}

void GaussianRestraint::check_particles()
{
    IMP_IF_CHECK(USAGE) {
        if (isx_)
        {
            IMP_USAGE_CHECK(Nuisance::particle_is_instance(px_),
                    "x particle should be a Nuisance!");
        }
        if (ismu_)
        {
            IMP_USAGE_CHECK(Nuisance::particle_is_instance(pmu_),
                    "mu particle should be a Nuisance!");
        }
        if (issigma_)
        {
            IMP_USAGE_CHECK(Scale::particle_is_instance(psigma_),
                    "sigma particle should be a Scale!");
        }
    }
}


/* Apply the restraint to two atoms, two Scales, one experimental value.
 */
double
GaussianRestraint::unprotected_evaluate(DerivativeAccumulator *accum) const
{
  double x,mu,sigma;
  x = (isx_) ? Nuisance(px_).get_nuisance() : x_;
  mu = (ismu_) ? Nuisance(pmu_).get_nuisance() : mu_;
  sigma = (issigma_) ? Scale(psigma_).get_scale() : sigma_;
  /* compute all arguments to FNormal */
  double JA = 1.0;
  IMP_NEW(FNormal, normal, (x,JA,mu,sigma));
  //normal->set_was_used(true); // get rid of warning
  /* get score */
  double score= normal->evaluate();
  const_cast<GaussianRestraint *>(this)->set_chi(x-mu);

  if (accum)
  {
      if (isx_ || ismu_)
      {
          double DFM = normal->evaluate_derivative_FM();
          if (isx_) Nuisance(px_).add_to_nuisance_derivative(-DFM, *accum);
          if (ismu_) Nuisance(pmu_).add_to_nuisance_derivative(DFM, *accum);
      }
      if (issigma_)
          Scale(psigma_).add_to_scale_derivative(
                  normal->evaluate_derivative_sigma(), *accum);
  }
  return score;
}

/* Return all particles whose attributes are read by the restraints. To
   do this, ask the pair score what particles it uses.*/
ParticlesTemp GaussianRestraint::get_input_particles() const
{
  ParticlesTemp ret;
  if (isx_) ret.push_back(px_);
  if (ismu_) ret.push_back(pmu_);
  if (issigma_) ret.push_back(psigma_);
  return ret;
}

/* The only container used is pc_. */
ContainersTemp GaussianRestraint::get_input_containers() const
{
  return ContainersTemp();
}

void GaussianRestraint::do_show(std::ostream& out) const
{
  out << "Gaussian restraint" << std::endl;
  if (isx_) {
      out << "x= " << px_->get_name() << std::endl;
  } else {
      out << "x= " << x_ << std::endl;
  }
  if (ismu_) {
      out << "mu= " << pmu_->get_name() << std::endl;
  } else {
      out << "mu= " << mu_ << std::endl;
  }
  if (issigma_) {
      out << "sigma= " << psigma_->get_name() << std::endl;
  } else {
      out << "sigma= " << sigma_ << std::endl;
  }
}

IMPISD_END_NAMESPACE
