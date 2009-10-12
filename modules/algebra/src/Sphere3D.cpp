/**
 *  \file  Sphere3D.cpp
 *  \brief simple implementation of spheres in 3D
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 */
#include <IMP/base_types.h>
#include <IMP/algebra/Sphere3D.h>
#include <cmath>

#ifdef IMP_USE_CGAL
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Min_sphere_of_spheres_d.h>
#endif

IMPALGEBRA_BEGIN_NAMESPACE
Sphere3D::Sphere3D(const Vector3D& center,double radius):center_(center),
                                                 radius_(radius){
}
double Sphere3D::get_volume() const {
  return PI * (4.0 / 3.0) * std::pow(radius_, 3.0);
}

double Sphere3D::get_surface_area() const {
  return PI * 4.0 * square(radius_);
}

Sphere3D enclosing_sphere(const Sphere3Ds &ss) {
  IMP_USAGE_CHECK(!ss.empty(),
                  "Must pass some spheres to have a bounding sphere",
            ValueException);
#ifdef IMP_USE_CGAL
  typedef CGAL::Exact_predicates_inexact_constructions_kernel             K;
  typedef CGAL::Min_sphere_of_spheres_d_traits_3<K, K::FT> Traits;
  typedef CGAL::Min_sphere_of_spheres_d<Traits> Min_sphere;
  typedef K::Point_3                        Point;
  typedef Traits::Sphere                    Sphere;

  std::vector<Sphere> spheres;
  for (unsigned int i=0; i< ss.size(); ++i) {
    // need cast to resolve ambiguity
    spheres.push_back(Sphere(Point(ss[i].get_center()[0],
                                   ss[i].get_center()[1],
                                   ss[i].get_center()[2]),
                             ss[i].get_radius()));
  }
  Min_sphere ms(spheres.begin(), spheres.end());
  Sphere3D s(Vector3D(*ms.center_cartesian_begin(),
                      *(ms.center_cartesian_begin()+1),
                      *(ms.center_cartesian_begin()+2)),
              ms.radius());
   /*IMP_IF_LOG(VERBOSE) {
     IMP_LOG(VERBOSE, "Enclosing sphere is " << s << " for ");
     for (unsigned int i=0; i< ss.size(); ++i) {
       IMP_LOG(VERBOSE, ss[i] << "| ");
     }
     IMP_LOG(VERBOSE, std::endl);
     }*/
   return s;
#else
   BoundingBox3D bb= get_bounding_box(ss[0]);
   for (unsigned int i=1; i< ss.size(); ++i) {
     bb+= get_bounding_box(ss[i]);
   }
   Vector3D c= .5*(bb.get_corner(0)+ bb.get_corner(1));
   double r=0;
   for (unsigned int i=0; i< ss.size(); ++i) {
     double d= (c- ss[i].get_center()).get_magnitude();
     d+= ss[i].get_radius();
     r= std::max(r, d);
   }
   return Sphere3D(c, r);
#endif
}

IMPALGEBRA_END_NAMESPACE
