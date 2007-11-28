/**
 *  \file XYZDecorator.cpp   \brief Simple xyz decorator.
 *
 *  Copyright 2007 Sali Lab. All rights reserved.
 *
 */

#include <IMP/decorators/XYZDecorator.h>
#include <sstream>
#include <cmath>

namespace IMP
{

FloatKey XYZDecorator::x_key_;
FloatKey XYZDecorator::y_key_;
FloatKey XYZDecorator::z_key_;




  void XYZDecorator::show(std::ostream &out, std::string prefix) const
{
  out << prefix << "(" << get_x()<< ", "
  << get_y() << ", " << get_z() <<")";

}




  IMP_DECORATOR_INITIALIZE(XYZDecorator, DecoratorBase,
                           {
                             x_key_= FloatKey("x");
                             y_key_= FloatKey("y");
                             z_key_= FloatKey("z");
                           })
  namespace {
    template <class T>
    T d(T a, T b){T d=a-b; return d*d;}
  }

  Float distance(XYZDecorator a, XYZDecorator b){
    double d2= d(a.get_x(), b.get_x()) + d(a.get_y(), b.get_y())
      + d(a.get_z(), b.get_z());
    return std::sqrt(d2);
  }
} // namespace IMP
