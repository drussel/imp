#ifndef _COARSECC_H
#define _COARSECC_H

#include <vector>
#include "EM_config.h"
#include "exp.h"
#include "DensityMap.h"
#include "SampledDensityMap.h"
#include "ParticlesAccessPoint.h"
#include "def.h"
#include "ErrorHandling.h"
//! Responsible for performing coarse fitting between two density objects.
/** The pixels involved are derived from the positions of N particles.
 */
class EMDLLEXPORT CoarseCC
{

public:


  //! Evaluates the value of the cross correlation term.
  /** This function has a special behavior, as it does not return the true
      cross correlation coefficient ccc, but the value:
      scalefac*(1-ccc)
      The reason why is to use this term as part of an scoring function that
      is better the lower the term. If you want the cross correlation
      coefficient, use cross_correlation_coefficient() instead.
      \param[in] em_map DensityMap class containing the EM map. note:
             correct RMSD and mean MUST be in the header!
      \param[in] model_map SampledDensityMap class prepared to contain the
             simulated EM map for the model.
      \param[in] access_p ParticlesAccessPoint class with the particles data
             (location, radii, weight)
      \param[in] dvx vector to contain the xpartial derivatives
      \param[in] dvy vector to contain the y partial derivatives
      \param[in] dvz vector to contain the z partial derivatives
      \param[in] scalefactor scale factor to apply to the value of the cross
             correlation term
      \param[in] lderiv if true, the derivatives of the term are computed
      \return the value of the cross correlation term: scalefac*(1-ccc)
   */
  static float evaluate(DensityMap &data, SampledDensityMap &model_map,
                        const ParticlesAccessPoint &access_p,
                        std::vector<float> &dvx, std::vector<float>&dvy,
                        std::vector<float>&dvz, float scalefac, bool lderiv,
                         bool divide_by_rms=true);


/*!
 Computes the derivatives of the cross correlation term scalefac*(1-ccc) at each
 voxel of the map.
 \param[in] em_map DensityMap class containing the EM map. note: correct RMS and
            mean MUST be in the header!
 \param[in] model_map SampledDensityMap class prepared to contain the simulated
            EM map for the model.
 \param[in] access_p ParticlesAccessPoint class with the particles data
            (location, radii, weight)
 \param[in] scalefactor scale factor to apply to the value of the cross
                        correlation term
 \param[out] dvx vector to contain the x partial derivatives
 \param[out] dvy vector to contain the y partial derivatives
 \param[out] dvz vector to contain the z partial derivatives
 \return the function stores the values of the partial derivatives in
         the vectors
*/
/* comments: Javi to Frido:
I am pretty sure what causes the subtle difference:
the corr routine requires that the mean is subtracted from the em-density.
we did not do that, yet.
*/
  static void calc_derivatives(const DensityMap &em_map,
                              SampledDensityMap &model_map,
                              const ParticlesAccessPoint &access_p,
                              const float &scalefac,
                              std::vector<float> &dvx, std::vector<float>&dvy,
                              std::vector<float>&dvz);



  /** Cross correlation coefficient between the em density and the density of a
      model. moddens threshold can be specified that is checked in moddens to
      reduce elements of summation
      \note This is not the local CC function
      \param[in] em_map               the target map (experimentally determined)
      \param[in] model_map            the sampled density map of the model
      \param[in] voxel_data_threshold voxels with value lower than threshold
                 in model_map are not summed (avoid calculating correlation on
                 voxels below the threshold)
      \param[in] recalc_ccnormfac determines wheather the model_map should be
                 normalized prior to the correlation calculation. false is
                 faster, but potentially innacurate
      \return the cross correlation coefficient value between two density maps
      comments:
      Frido:
      I am pretty sure what causes the subtle difference:
      the corr routine requires that the mean is subtracted from the
      em-density. we did not do that, yet.
   */
  static float cross_correlation_coefficient(const DensityMap &em_map,
                                             DensityMap &model_map,
                                             float voxel_data_threshold,
                                             bool recalc_ccnormfac = true,
                                             bool divide_by_rms=true);
};


#endif //_COARSECC_H
