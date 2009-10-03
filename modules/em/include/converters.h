/**
 *  \file converters.h
 *  \brief Converters of density values
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 *
 */

#ifndef IMPEM_CONVERTERS_H
#define IMPEM_CONVERTERS_H

#include "config.h"
#include <IMP/base_types.h>
#include <IMP/macros.h>
#include "DensityMap.h"
#include "SampledDensityMap.h"
#include <IMP/em/IMPParticlesAccessPoint.h>
#include <IMP/Particle.h>
#include "IMP/core/XYZ.h"
#include "IMP/algebra/Vector3D.h"

IMPEM_BEGIN_NAMESPACE

//! Converts a density grid to a set of paritlces
/**
Each such particle will be have xyz attributes and a density_val attribute of
type Float.
/param[in] dmap the density map
/param[in] threshold only voxels with density above the given threshold will
           be converted to particles
/param[in] the new density based particles will be converted to particles.
 */
IMPEMEXPORT void density2particles(DensityMap &dmap, Float threshold,
                                   Particles &ps, Model *m);


//! Resample a set of particles into a density grid
/**
Each such particle should be have xyz radius and weight attributes
/param[in] ps         the particles to sample
/param[in] rad_key    the radius attribute key of the particles
/param[in] weight_key the weight attribute key of the particles
/param[in] ps  the particles to sample
/param[in] resolution the resolution of the new sampled map
/param[in] apix the voxel size of the sampled map
/return the sampled density grid
 */
inline SampledDensityMap * particles2density(Particles &ps,
               const FloatKey &rad_key, const FloatKey &weight_key,
               Float resolution, Float apix) {
  IMPParticlesAccessPoint access_p(ps,rad_key,weight_key);
  SampledDensityMap *dmap = new SampledDensityMap(access_p, resolution,
                                                  apix);
  return dmap;
}

IMPEM_END_NAMESPACE
#endif /* IMPEM_CONVERTERS_H */
