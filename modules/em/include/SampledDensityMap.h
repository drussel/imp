/**
 *  \file SampledDensityMap.h
 *  \brief Sampled density map.
 *
 *  Copyright 2007-2010 Sali Lab. All rights reserved.
 *
 */

#ifndef IMPEM_SAMPLED_DENSITY_MAP_H
#define IMPEM_SAMPLED_DENSITY_MAP_H

#include "config.h"
#include "exp.h"
#include "DensityMap.h"
#include "DensityHeader.h"
#include "def.h"
#include "ParticlesAccessPoint.h"
#include "KernelParameters.h"
#include <vector>

IMPEM_BEGIN_NAMESPACE

// an advantage of using const double over define is that it limits the use
// of pointers to affect the data:
//#define PI 3.141592653589793238462643383
//define could be manipulated with a const *int ptr declaration.

class IMPEMEXPORT SampledDensityMap: public DensityMap
{

public:


  //! Creates a new density map for sampled map.
  /** The header of the map is not determined and no data is being allocated
   */
  SampledDensityMap() {
  }

  //! The size of the map is determined by the header and the data is allocated.
  SampledDensityMap(const DensityHeader &header);

  //! Generatea a sampled density map from the particles.
  /** /param[in] access_p     access point to the particles (locations,
                              radius, weight)
      /param[in] resolution   half width the Gaussian
      /param[in] voxel_size
      /param[in] sig_cutoff   Choose what should be the sigma cutoff for
                 accurate sampling. It is used in two functions;
                 (i)  to determine the size of the grid dimensions
                 (ii) to determine the voxels around the coords participating
                      in the sampling procedure.
   */
  SampledDensityMap(const ParticlesAccessPoint &access_p, emreal resolution,
                    emreal voxel_size, int sig_cuttoff=3);


  //! Sampling beads on an EM grid
  /** /param[in] access_p   access point to the particles (locations,
                            radius, weight)
   */
  virtual void resample(const ParticlesAccessPoint &access_p);

  void calc_sampling_bounding_box(const emreal &x,const emreal &y,
                                  const emreal &z,
                                  const emreal &kdist,
                                  int &iminx,int &iminy, int &iminz,
                                  int &imaxx,int &imaxy, int &imaxz) const;

  KernelParameters *get_kernel_params()  { return &kernel_params_;}

  IMP_REF_COUNTED_DESTRUCTOR(SampledDensityMap)
protected:
  //! Calculate the parameters of the particles bounding box
  /** \param[in]  access_p     access point to the particles (locations,
                               radius, weight)
      \param[out] lower_bound  the left-bottom point of the bounding box
                               (lower_bound[0] - x coordinate
                                lower_bound[1] - y coordinate
                                lower_bound[2] - z coordinate)
      \param[out] upper_bound  the right-upper point of the bounding box
      \param[out] maxradius    the maximum radius of all the particles in
                               the model.
   */
  void calculate_particles_bounding_box(const ParticlesAccessPoint &access_p,
                                        std::vector<emreal> &lower_bound,
                                        std::vector<emreal> &upper_bound,
                                        emreal &maxradius);

  void set_header(const std::vector<emreal> &lower_bound,
                  const std::vector<emreal> &upper_bound,
                  emreal maxradius, emreal resolution, emreal voxel_size,
                  int sig_offset);

  // bookkeeping functions
  int lower_voxel_shift(const emreal &loc, const emreal &kdist,
                        const emreal &orig, int ndim) const {
    int imin;
    imin = (int)floor(((loc-kdist-orig) / header_.get_spacing()));
    //bookkeeping
    if (imin < 0)
      imin = 0;
    if (imin > ndim-1)
      imin = ndim-1;
    return imin;
  }


  int upper_voxel_shift(const emreal &loc, const emreal &kdist,
                        const emreal &orig, int ndim) const {
    int imax;
    imax = (int)floor(((loc+kdist-orig) / header_.get_spacing()));
    //bookkeeping
    if (imax < 0) imax = 0;
    if (imax > ndim-1) imax = ndim-1;
    return imax;
  }

protected:
  //! kernel handling
  KernelParameters kernel_params_;
};

IMPEM_END_NAMESPACE

#endif  /* IMPEM_SAMPLED_DENSITY_MAP_H */
