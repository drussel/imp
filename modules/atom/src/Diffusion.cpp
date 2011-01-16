/**
 *  \file Diffusion.cpp   \brief Simple xyzr decorator.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 *
 */

#include "IMP/atom/Diffusion.h"
#include <IMP/algebra/Vector3D.h>
#include <IMP/constants.h>

IMPATOM_BEGIN_NAMESPACE

typedef
unit::Shift<unit::Multiply<unit::Pascal,
                           unit::Second>::type,
            -3>::type MillipascalSecond;

static MillipascalSecond eta(unit::Kelvin T)
{
  const std::pair<unit::Kelvin, MillipascalSecond> points[]
    ={ std::make_pair(unit::Kelvin(273+10.0),
                      MillipascalSecond(1.308)),
       std::make_pair(unit::Kelvin(273+20.0),
                      MillipascalSecond(1.003)),
       std::make_pair(unit::Kelvin(273+30.0),
                      MillipascalSecond(0.7978)),
       std::make_pair(unit::Kelvin(273+40.0),
                      MillipascalSecond(0.6531)),
       std::make_pair(unit::Kelvin(273+50.0),
                      MillipascalSecond(0.5471)),
       std::make_pair(unit::Kelvin(273+60.0),
                      MillipascalSecond(0.4668)),
       std::make_pair(unit::Kelvin(273+70.0),
                      MillipascalSecond(0.4044)),
       std::make_pair(unit::Kelvin(273+80.0),
                      MillipascalSecond(0.3550)),
       std::make_pair(unit::Kelvin(273+90.0),
                      MillipascalSecond(0.3150)),
       std::make_pair(unit::Kelvin(273+100.0),
                      MillipascalSecond(0.2822)),
       std::make_pair(unit::Kelvin(std::numeric_limits<Float>::max()),
                      MillipascalSecond(0.2822))};

  //const unsigned int npoints= sizeof(points)/sizeof(std::pair<float,float>);
  if (T < points[0].first) {
    return points[0].second;
  } else {
    unsigned int i;
    for (i=1; points[i].first < T; ++i);
    double f= ((T - points[i-1].first)
              /(points[i].first - points[i-1].first));
    MillipascalSecond ret=
      (1.0-f) *points[i-1].second + f*points[i].second;
    return ret;
  }
}

namespace {
unit::Femtojoule kt(unit::Kelvin t) {
  return IMP::unit::Femtojoule(IMP::internal::KB*t);
}
}


FloatKey Diffusion::get_d_key() {
  static FloatKey k("D");
  return k;
}
void Diffusion::set_d_from_radius(Float ir) {
  return set_d_from_radius(ir,
                            IMP::internal::DEFAULT_TEMPERATURE.get_value());
}


unit::SquareCentimeterPerSecond Diffusion::D_from_r(unit::Angstrom radius,
                                                    unit::Kelvin t) {
  MillipascalSecond e=eta(t);
  unit::SquareCentimeterPerSecond ret(kt(t)/(6.0* PI*e*radius));
  return ret;
}



void Diffusion::set_d_from_radius(Float ir,
                                  Float it) {
  set_d(D_from_r(unit::Angstrom(ir),
                 unit::Kelvin(it)));
}

void Diffusion::show(std::ostream &out) const
{
  XYZ::show(out);
  out << "D= " << get_d_in_cm2_per_second() << "cm^2/sec";

}

IMPATOM_END_NAMESPACE
