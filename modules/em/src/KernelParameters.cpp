/**
 *  \file KernelParameters.cpp
 *  \brief Calculates and stores gaussian kernel parameters.
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 *
 */

#include <IMP/em/KernelParameters.h>
#include <IMP/constants.h>

IMPEM_BEGIN_NAMESPACE

KernelParameters::Parameters::Parameters(float radii_, float rsigsq_,
                                         float timessig_, float sq2pi3_,
                                         float inv_rsigsq_, float rnormfac_,
                                         float rkdist_)
{
  if (radii_ > EPS) {
    // to prevent calculation for particles with the same radius ( atoms)
    //    vsig = 1./(sqrt(2.*log(2.))) * radii_; // volume sigma
    vsig = 1./(sqrt(2.)) * radii_; // volume sigma
    vsigsq = vsig * vsig;
    inv_sigsq = rsigsq_ + vsigsq;
    sig = sqrt(inv_sigsq);
    kdist = timessig_ * sig;
    inv_sigsq = 1./inv_sigsq *.5;
    normfac = sq2pi3_ * 1. / (sig * sig * sig);
  }
  else {
    inv_sigsq = inv_rsigsq_;
    normfac = rnormfac_;
    kdist = rkdist_;
  }
}


void KernelParameters::init(float resolution_)
{
  // the number of sigmas used - 3 means that 99% of density is considered.
  timessig=3.;
  // convert resolution to sigma squared. Full width at half maximum criterion
  // (Topf 2008)
  rsig = resolution_/(2*sqrt(2.*log(2.))); // sigma
  rsigsq = rsig * rsig; // sigma squared
  inv_rsigsq = 1./(2.*rsigsq); // term for the exponential
  // normalization factor for the gaussian
  sq2pi3 = 1. / sqrt(8. * PI * PI * PI);
  rnormfac = sq2pi3 * 1. / (rsig * rsig * rsig);
  rkdist   = timessig * rsig;
  lim = exp(-0.5 * (timessig - EPS) * (timessig - EPS));
}



void KernelParameters::set_params(float radius) {
  IMP_check(initialized,
            "The Kernel Parameters are not initialized",
            InvalidStateException);
  std::map<float ,const KernelParameters::Parameters *>::iterator iter =
                   radii2params.find(radius);
  IMP_check(iter == radii2params.end(),
            "The Kernel Parameters for the radius " << radius
            << " have already been calculated",
            InvalidStateException);
  radii2params[radius]=new Parameters(radius,rsigsq,timessig,sq2pi3,
                                          inv_rsigsq,rnormfac,rkdist);
}

IMPEM_END_NAMESPACE
