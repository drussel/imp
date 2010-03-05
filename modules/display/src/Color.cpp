/**
 *  \file Color.cpp
 *  \brief XXXX.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 *
 */

#include "IMP/display/Color.h"


IMPDISPLAY_BEGIN_NAMESPACE

Color::Color() {
  c_[0]=-1;
  c_[1]=-1;
  c_[2]=-1;
}

Color::Color( float r, float g, float b){
  IMP_USAGE_CHECK(r>= 0 && r <=1, "Red out of range: " << r);
  IMP_USAGE_CHECK(g>= 0 && g <=1, "Green out of range: " << g);
  IMP_USAGE_CHECK(b>= 0 && b <=1, "Blue out of range: " << b);
  c_[0]=r;
  c_[1]=g;
  c_[2]=b;
}

Color::~Color(){}

IMPDISPLAY_END_NAMESPACE
