/**
 *  \file BoundingBox3DSingletonScore.cpp
 *  \brief XXXX.
 *
 *  Copyright 2007-8 IMP Inventors. All rights reserved.
 *
 */

#include "IMP/core/BoundingBox3DSingletonScore.h"
#include "IMP/core/XYZ.h"
#include <IMP/core/internal/evaluate_distance_pair_score.h>

#include <boost/lambda/lambda.hpp>

IMPCORE_BEGIN_NAMESPACE

BoundingBox3DSingletonScore
::BoundingBox3DSingletonScore(UnaryFunction *f,
                            const algebra::BoundingBox3D &bb ): f_(f), bb_(bb){
  IMP_USAGE_CHECK(std::abs(f_->evaluate(0)) <.1,
                  "The unary function should return "
            " 0 when passed a value of 0. Not " << f_->evaluate(0));
}

double BoundingBox3DSingletonScore::evaluate(Particle *p,
                                           DerivativeAccumulator *da) const {
  core::XYZ d(p);
  algebra::VectorD<3> cp;
  bool outside=false;
  for (unsigned int i=0; i< 3; ++i) {
    if (bb_.get_corner(0)[i] > d.get_coordinate(i)) {
      cp[i]=bb_.get_corner(0)[i];
      outside=true;
    } else if (bb_.get_corner(1)[i] < d.get_coordinate(i)) {
      cp[i]=bb_.get_corner(1)[i];
      outside=true;
    } else {
      cp[i]= d.get_coordinate(i);
    }
  }
  if (outside) {
    algebra::VectorD<3> deriv;
    double v= internal::compute_distance_pair_score(d.get_coordinates()-cp,
                                                    f_.get(),&deriv,
                                                    boost::lambda::_1);
    if (da) {
      d.add_to_derivatives(deriv, *da);
    }
    return v;
  } else {
    return 0;
  }
}

void BoundingBox3DSingletonScore::do_show(std::ostream &out) const {
  out << "box is " << bb_ << std::endl;
}

IMPCORE_END_NAMESPACE
