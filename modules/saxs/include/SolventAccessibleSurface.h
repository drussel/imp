/**
 * \file SolventAccessibleSurface.h \brief
 *
 * Copyright 2007-2011 IMP Inventors. All rights reserved.
 *
 */
#ifndef IMPSAXS_SOLVENT_ACCESSIBLE_SURFACE_H
#define IMPSAXS_SOLVENT_ACCESSIBLE_SURFACE_H

#include "saxs_config.h"
#include <IMP/core/XYZR.h>

IMPSAXS_BEGIN_NAMESPACE

/**
   Class for estimation of accessible surface area. The probe (sampled
   with dots) is rolled over the atoms. The area is estimated by the
   number of dots that do not collide with atoms. Note, this is a rough
   estimate of the area (enough for SAXS).
*/
class IMPSAXSEXPORT SolventAccessibleSurface {
public:
  //! estimate surface accessability of each atom.
  /**
     \param[in] points A set of points for which surface accessability is
     computed. Each point should have an XYZ coordinate and  a radius.
     \param[in] probe_radius Radius of the probe to roll over points.
     \param[in] density Sampling density per A^2 for area estimation
     \return a value between 0 to 1, for surface accessability,
     where 0 means buried and 1 means fully accessible to the water
  */
  IMP::Floats get_solvent_accessibility(const core::XYZRs& points,
                                        float probe_radius = 1.8,
                                        float density = 5.0);
 private:
  bool is_intersecting(const algebra::Vector3D& sphere_center1,
                       const algebra::Vector3D& sphere_center2,
                       const float radius1, const float radius2) {
    float squared_radius_sum = (radius1+radius2)*(radius1+radius2);
    float squared_dist =
      algebra::get_squared_distance(sphere_center1, sphere_center2);
    if(fabs(squared_radius_sum - squared_dist) < 0.0001)
      return false;
    if(squared_radius_sum > squared_dist)
      return true;
    return false;
  }

  algebra::Vector3Ds create_sphere_dots(float radius, float density);

};

IMPSAXS_END_NAMESPACE

#endif /* IMPSAXS_SOLVENT_ACCESSIBLE_SURFACE_H */
