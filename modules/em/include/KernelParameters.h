/**
 *  \file KernelParameters.h
 *  \brief Calculates and stores gaussian kernel parameters.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPEM_KERNEL_PARAMETERS_H
#define IMPEM_KERNEL_PARAMETERS_H

#include "def.h"
#include <IMP/exception.h>
#include <IMP/log.h>
#include <map>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <vector>
#include <limits>

IMPEM_BEGIN_NAMESPACE

//! Calculates kernel parameters as a function of a specific radius.
class IMPEMEXPORT RadiusDependentKernelParameters {
  public:
    RadiusDependentKernelParameters(
       float radii,float rsigsq,float timessig,
       float sq2pi3,float inv_rsigsq, float rnormfac, float rkdist);
    //! Show
    void show(std::ostream& s=std::cout) const;
    //! Gets the value of the volume sigma
    inline float get_vsig() const { return vsig_;}
    //! Gets the value of volume sigma squared
    inline float get_vsigsq() const { return vsigsq_;}
    //! Gets the inverse sigma squared
    inline float get_inv_sigsq() const { return inv_sigsq_;}
    //! Gets the sigma value
    inline float get_sig() const { return sig_;}
    //! Gets the value of kdist parameter
    inline float get_kdist() const { return kdist_;}
    //! Gets the value of normfac parameter
    inline float get_normfac() const { return normfac_;}
    ~RadiusDependentKernelParameters() {
      vsig_= std::numeric_limits<float>::quiet_NaN();
      vsigsq_= std::numeric_limits<float>::quiet_NaN();
      inv_sigsq_= std::numeric_limits<float>::quiet_NaN();
      sig_= std::numeric_limits<float>::quiet_NaN();
      kdist_= std::numeric_limits<float>::quiet_NaN();
      normfac_= std::numeric_limits<float>::quiet_NaN();
    }
  protected:
    //! vsig
    float vsig_;
    //! square of vsig
    float vsigsq_;
    //! the inverse of sigma square
    float inv_sigsq_;
    //! the sigma
    float sig_;
    //! the kernel distance (= elements for summation)
    float kdist_;
    //! normalization factor
    float normfac_;
};

IMP_VALUES(RadiusDependentKernelParameters,
           RadiusDependentKernelParametersList);

//! Calculates and stores Gaussian kernel parameters as a function
//! of a specufuc radius.
class IMPEMEXPORT KernelParameters
{
public:
  KernelParameters() {
    initialized_ = false;
  }

  KernelParameters(float resolution) {
    init(resolution);
    initialized_ = true;
  }

  //! Sets the parameters that depend on the radius of a given particle.
  /** The other variables of the parameters
    (rsigsq,timessig,sq2pi3,inv_rsigsq,rnormfac,rkdist) must have been set.
    \param[in] radius the radius
    \return the radius based parameters
  */
  const RadiusDependentKernelParameters* set_params(float radius);

  //! Finds the precomputed parameters given a particle radius.
  /**
    \param[in] radius searching for parameters of this radius
    \param[in] eps used for numerical stability
    \note The parameters are indexes by the radius. To maintain
    numeratical stability, look for a radius within +-eps from the
    queried radius.
   \note the function return NULL and writes a warning if parameters
   for this radius were not found.
  */
  const RadiusDependentKernelParameters* get_params(
        float radius,float eps=0.001);
  bool are_params_set(float radius,float eps=0.001) {
    return get_params(radius, eps) != NULL;
  }

  //! Get sigma as a function of the resolution according to th
  //! Full width at half maximum criterion
  inline  float get_rsig() const  {return rsig_;}
  //! Get squared sigma as a function of the resolution accroding to the
  //! Full width at half maximum criterion
  inline float get_rsigsq() const {return rsigsq_;}
  //! Get the inverse of sigma sqaured
  inline float get_inv_rsigsq() const {return inv_rsigsq_;}
  //! Gets the number of sigma used.
  /**
    \note We use 3, which means that 99% of the density is considered
   */
  inline float get_timessig() const {return timessig_;}
  //! Get the non-sigma portion of the Gaussian normalization factor
  inline float get_sq2pi3() const {return sq2pi3_;}
  //! Get the Gaussian normalization factor
  inline float get_rnormfac() const {return rnormfac_;}
  //! Get the lenght of the Gaussian (sigma*number_of_sigmas_used)
  inline float get_rkdist() const {return rkdist_;}
  //! Gets the value of lim parameter
  inline float get_lim() const {return lim_;}
protected:
  float rsig_,rsigsq_,timessig_,sq2pi3_,inv_rsigsq_,rnormfac_,rkdist_,lim_;
  bool initialized_;
  std::map <float,const RadiusDependentKernelParameters *> radii2params_;
  void init(float resolution);
};

IMP_VALUES(KernelParameters, KernelParametersList);

IMPEM_END_NAMESPACE

#endif  /* IMPEM_KERNEL_PARAMETERS_H */
