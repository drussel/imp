/**
 *  \file SampledDensityMap.h
 *  \brief Sampled density map.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPEM_SAMPLED_DENSITY_MAP_H
#define IMPEM_SAMPLED_DENSITY_MAP_H

#include "em_config.h"
#include "exp.h"
#include "DensityMap.h"
#include "DensityHeader.h"
#include "def.h"
#include "KernelParameters.h"
#include <vector>
#include "IMP/base_types.h"
#include "IMP/core/XYZR.h"
#include "IMP/atom/Mass.h"
#include "masking.h"
IMPEM_BEGIN_NAMESPACE

// an advantage of using const double over define is that it limits the use
// of pointers to affect the data:
//#define PI 3.141592653589793238462643383
//define could be manipulated with a const *int ptr declaration.


//! Class for sampling a density map from particles
class IMPEMEXPORT SampledDensityMap: public DensityMap
{

public:


  //! Creates a new density map for sampled map.
  /** The header of the map is not determined and no data is being allocated
   */
  SampledDensityMap(): DensityMap("SampledDensityMap%1%") {
  }

  //! The size of the map is determined by the header and the data is allocated.
  SampledDensityMap(const DensityHeader &header);

  //! Generatea a sampled density map from the particles.
  /** /param[in] ps     particles with XYZ, radius and weight attributes
      /param[in] resolution   half width the Gaussian
      /param[in] voxel_size
      /param[in] sig_cutoff   Choose what should be the sigma cutoff for
                 accurate sampling. It is used in two functions;
                 (i)  to determine the size of the grid dimensions
                 (ii) to determine the voxels around the coords participating
                      in the sampling procedure.
   */
  SampledDensityMap(const Particles &ps, emreal resolution,
   emreal voxel_size,
   IMP::FloatKey mass_key=IMP::atom::Mass::get_mass_key(),
   int sig_cuttoff=3);

  //! Resampling beads on an EM grid
  /**
  \note The density of a particle p centered at pl at position gl is:
  \f$\frac{{Z}e^{\frac{{-0.5}({p_l}-{g_l})}{\sigma}}}{\sqrt{{2}{\pi}\sigma}}\f$
    , such that \f${Z}\f$ is the weight of the particle and \f${\sigma}\f$
      is defined to be \f${0.425}\f$ times the resolution,
      to follow the 'full width at half maxima'
      criterion. For more details please refer to Topf et al, Structure, 2008.
   */
  virtual void resample();

//! Project particles on the grid by their mass value
/**
\param[in] x_margin sampling is restricted to [x_margin,nx-x_maring]
\param[in] y_margin sampling is restricted to [y_margin,ny-y_maring]
\param[in] z_margin sampling is restricted to [z_margin,nz-z_maring]
\param[in] shift the positions of all particles are shifted by
                 this value before projection
 */
void project (const Particles &ps,
       int x_margin,int y_margin,int z_margin,
              algebra::Vector3D shift=algebra::Vector3D(0.,0.,0.),
              FloatKey mass_key=atom::Mass::get_mass_key());

//!setting particles in case they were not set by the constructor
 void set_particles(const IMP::Particles &ps,
     IMP::FloatKey mass_key = IMP::atom::Mass::get_mass_key());

#if !defined(DOXYGEN) && !defined(SWIG)
  KernelParameters *get_kernel_params()  { return &kernel_params_;}
#endif

  inline const core::XYZRs & get_xyzr_particles() const {return xyzr_;}
 // would go away on�ce we have a XYZRW decorator and the next function as well
  inline const Particles & get_sampled_particles() const {return ps_;}
  inline FloatKey  get_weight_key() const {return weight_key_;}

  //! Get minimum density value between voxels that correspond to particles
  float get_minimum_resampled_value();

  IMP_REF_COUNTED_DESTRUCTOR(SampledDensityMap);
protected:
  //! Calculate the parameters of the particles bounding box
  /** \param[in] ps particles with XYZ, radius and weight attributes
      \return the particles bounding box
   */
  IMP::algebra::BoundingBoxD<3>
     calculate_particles_bounding_box(const Particles &ps);


  //! Determine the size of the grid as a function of the particles
  //! and the resolution.
  void determine_grid_size(emreal resolution, emreal voxel_size,int sig_cutoff);
  void set_header(const algebra::VectorD<3> &lower_bound,
                  const algebra::VectorD<3> &upper_bound,
                  emreal maxradius, emreal resolution, emreal voxel_size,
                  int sig_offset);
 protected:
  void set_neighbor_mask(float radius);
protected:
  //! kernel handling
  KernelParameters kernel_params_;
  //  DistanceMask distance_mask_;
  Particles ps_;
  core::XYZRs xyzr_; //each voxel decorator would contain X,Y,Z,R
  FloatKey weight_key_;
  FloatKey x_key_,y_key_,z_key_;
};
IMP_OBJECTS(SampledDensityMap, SampledDensityMaps);

IMPEM_END_NAMESPACE

#endif  /* IMPEM_SAMPLED_DENSITY_MAP_H */
