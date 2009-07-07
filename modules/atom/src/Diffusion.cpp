/**
 *  \file Diffusion.cpp   \brief Simple xyzr decorator.
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
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

unit::Femtojoule kt(unit::Kelvin t) {
  return IMP::unit::Femtojoule(IMP::internal::KB*t);
}



FloatKey Diffusion::get_D_key() {
  static FloatKey k("D");
  return k;
}
void Diffusion::set_D_from_radius_in_angstroms(Float ir) {
  return set_D_from_radius_in_angstroms(ir,
                            IMP::internal::DEFAULT_TEMPERATURE.get_value());
}


unit::SquareCentimeterPerSecond Diffusion::D_from_r(unit::Angstrom radius,
                                                    unit::Kelvin t) {
  MillipascalSecond e=eta(t);
  //unit::MKSUnit<-13, 0, 1, 0, -1> etar( e*r);
  /*std::cout << e << " " << etar << " " << kt << std::endl;
  std::cout << "scalar etar " << (unit::Scalar(6*unit::PI)*etar)
            << std::endl;
  std::cout << "ret pre conv " << (kt/(unit::Scalar(6* unit::PI)*etar))
  << std::endl;*/
  unit::SquareCentimeterPerSecond ret(kt(t)/(6.0* PI*e*radius));
  //std::cout << "ret " << ret << std::endl;
  return ret;
}



void Diffusion::set_D_from_radius_in_angstroms(Float ir,
                                                        Float it) {
  set_D(D_from_r(unit::Angstrom(ir),
                 unit::Kelvin(it)));
}

void Diffusion::show(std::ostream &out, std::string prefix) const
{
  XYZ::show(out, prefix);
  out << "D= " << get_D_in_cm2_per_second() << "cm^2/sec";

}

IMPATOM_END_NAMESPACE
