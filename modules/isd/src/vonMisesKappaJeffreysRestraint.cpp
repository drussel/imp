/**
 *  \file isd/vonMisesKappaJeffreysRestraint.cpp
 *  \brief Restrain a scale particle with log(scale)
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 *
 */

#include <IMP/Particle.h>
#include <IMP/isd/Scale.h>
#include <IMP/isd/vonMisesKappaJeffreysRestraint.h>
#include <math.h>
#include <IMP/isd/internal/bessel.h>

IMPISD_BEGIN_NAMESPACE

vonMisesKappaJeffreysRestraint::vonMisesKappaJeffreysRestraint(Particle *p)
  : kappa_(p) {}

void vonMisesKappaJeffreysRestraint::update_bessel(double kappaval) {
  //compute bessel functions
  I0_ = internal::bessel_i0(kappaval);
  I1_ = internal::bessel_i1(kappaval);
  old_kappaval=kappaval;
}

double vonMisesKappaJeffreysRestraint::get_probability() const
{
    Scale kappascale(kappa_);
    double kappaval=kappascale.get_scale();
    if (kappaval <= 0) {
        IMP_THROW("cannot use jeffreys prior on negative or zero scale",
                ModelException);
    }
    if ( kappaval != old_kappaval) {
        const_cast<vonMisesKappaJeffreysRestraint*>(this)->update_bessel(
                                                               kappaval);
    }
    double ratio = I1_/I0_;
    return sqrt(ratio*(kappaval-ratio-kappaval*ratio*ratio));

}

/* Apply the score if it's a scale decorator.
 */
double vonMisesKappaJeffreysRestraint::unprotected_evaluate(
                   DerivativeAccumulator *accum) const
{
  double score;
  Scale kappascale(kappa_);
  double kappaval=kappascale.get_scale();
  //computes the bessel functions if necessary
  score=-std::log(get_probability());
  if (accum) {
    /* calculate derivative and add to 1st coordinate of kappascale */
    double ratio=I1_/I0_;
    double deriv=0.5*(-1.0/ratio + 3*ratio + 1/kappaval
            + 1/(kappaval*(1 - kappaval/ratio) + ratio*kappaval*kappaval));
    kappascale.add_to_scale_derivative(deriv,*accum);
  }
  return score;
}

/* Return all particles whose attributes are read by the restraints. To
   do this, ask the pair score what particles it uses.*/
ParticlesTemp vonMisesKappaJeffreysRestraint::get_input_particles() const
{
  return ParticlesTemp(1,kappa_);
}

/* The only container used is pc_. */
ContainersTemp vonMisesKappaJeffreysRestraint::get_input_containers() const
{
  return ContainersTemp();
}

void vonMisesKappaJeffreysRestraint::do_show(std::ostream& out) const
{
  out << "particle= " << kappa_->get_name() << std::endl;
}

IMPISD_END_NAMESPACE
