/**
 *  \file SpherePatch3D.h   \brief Simple 3D sphere patch class.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 *
 */
#ifndef IMPALGEBRA_SPHERE_PATCH_3D_H
#define IMPALGEBRA_SPHERE_PATCH_3D_H

#include "Sphere3D.h"
#include "Plane3D.h"

IMPALGEBRA_BEGIN_NAMESPACE

/**
A patch of a sphere is defined as all points above the plane and on the sphere.
   \ingroup uninitialized_default
   \ingroup noncomparable
 */
class IMPALGEBRAEXPORT SpherePatch3D {
public:
  SpherePatch3D(){}
  //! Constructor
  /**
  /param[in] sph            the sphere to cut a patch from
  /param[in] crossing_plane the plane that crosses the sphere
  */
  SpherePatch3D(const SphereD<3> &sph, const Plane3D& crossing_plane);
  //! Return true if the point is contained in the patch
  /** Note that the point must be on the sphere (this is not necessarily
      checked).
   */
  bool get_contains(const VectorD<3> &p) const;
  Plane3D  get_plane() const {return crossing_plane_;}
  SphereD<3> get_sphere() const {return SphereD<3>(sph_);}
  IMP_SHOWABLE_INLINE(SpherePatch3D,{
      sph_.show(out);
      crossing_plane_.show(out);
    });
  //! Get a point which is on the boundary of the patch
  VectorD<3> get_boundary_point() const;
protected:
  SphereD<3> sph_;
  Plane3D crossing_plane_;
};

IMP_AREA_GEOMETRY_METHODS(SpherePatch3D, sphere_patch_3d,
                          if (0) std::cout << g; IMP_NOT_IMPLEMENTED;,
                          {
                            return get_bounding_box(g.get_sphere());
                          });


IMPALGEBRA_END_NAMESPACE

#endif  /* IMPALGEBRA_SPHERE_PATCH_3D_H */
