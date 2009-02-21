/**
 *  \file convertors.h
 *  \brief Convertors of density values
 *
 *  Copyright 2007-8 Sali Lab. All rights reserved.
 *
 */

#ifndef IMPEM_CONVERTORS_H
#define IMPEM_CONVERTORS_H

#include "config.h"
#include <IMP/base_types.h>
#include <IMP/macros.h>
#include "internal/version_info.h"
#include "DensityMap.h"
#include <IMP/Particle.h>
#include "IMP/core/XYZDecorator.h"
#include "IMP/algebra/Vector3D.h"
#define DENS_ATT_NAME "density_val"

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
static void density2particles(DensityMap &dmap, Float threshold,
                              Particles &ps, Model *m) {
  Float x,y,z;
  for (long i=0;i<dmap.get_number_of_voxels();i++) {
    x = dmap.voxel2loc(i,0);
    y = dmap.voxel2loc(i,1);
    z = dmap.voxel2loc(i,2);
    if (dmap.get_value(x,y,z) > threshold) {
      Particle * p = new Particle(m);
      IMP::core::XYZDecorator::create(p,IMP::algebra::Vector3D(x,y,z));
      p->add_attribute(DENS_ATT_NAME,dmap.get_value(x,y,z),false);
      ps.push_back(p);
    }
  }
}
IMPEM_END_NAMESPACE
#endif /* IMPEM_CONVERTORS_H */
