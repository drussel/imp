/**
 *  \file filters.cpp
 *  \brief Classes to deal with filters
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
*/

#include "IMP/em/filters.h"

IMPEM_BEGIN_NAMESPACE

//! Function to set the paramters of of the filter
/**
  \param[in] threshold
  \param[in] mode: 0 - Filter all the values BELOW the threshold
                    1 - Filter all the values ABOVE the threshold
  \param[in] value Value to given to the filtered entries. By default
    the given value is the threshold
**/
void MapFilterByThreshold::set_parameters(float threshold,
                                              int mode,float value) {
  threshold_ = threshold;
  mode_ = mode;
  value_ = value;
}

void MapFilterByThreshold::set_parameters(float threshold,int mode) {
  threshold_ = threshold;
  mode_ = mode;
  value_ = threshold;
}

void MapFilterByThreshold::apply(DensityMap& m) {
  long  nvox = m.get_number_of_voxels();
  for (long ii=0;ii<nvox;ii++) {
    switch (mode_) {
    case 0: // Filter all the values BELOW the threshold
      if(m.get_value(ii)<threshold_) {
        m.set_value(ii,value_);
      }
      break;
    case 1: // Filter all the values ABOVE the threshold
      if(m.get_value(ii)>threshold_) {
        m.set_value(ii,value_);
      }
      break;
    }
  }
}

IMPEM_END_NAMESPACE
