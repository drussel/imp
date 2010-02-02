/**
 *  \file CoarseCCatIntervals.cpp
 *  \brief Cross correlation coefficient calculator.
 *
 *  Copyright 2007-2010 Sali Lab. All rights reserved.
 *
 */

#include <IMP/em/CoarseCCatIntervals.h>

IMPEM_BEGIN_NAMESPACE

CoarseCCatIntervals::CoarseCCatIntervals(const int &ncd)
{

  // Number of times the evaluation has been called. The evaluation is only
  // performed the first time and when calls_counter_ reaches eval_interval.
  // Otherwise the stored_cc_ value is returned
  calls_counter_ = 0;

  stored_cc_ = 0.0;
  dv_memory_allocated_ = false;
  allocate_derivatives_array(ncd);
}

CoarseCCatIntervals::CoarseCCatIntervals()
{
  calls_counter_ = 0;
  stored_cc_ = 0.0;
  dv_memory_allocated_ = false;
}


void CoarseCCatIntervals::allocate_derivatives_array(int ncd)
{
  if (dv_memory_allocated_)
    return;
  // Allocate memmory for the derivative terms if not done yet
  stored_dvx_=new float [ncd];
  stored_dvy_=new float [ncd];
  stored_dvz_=new float [ncd];

  for (int i=0;i<ncd;i++) {
     stored_dvx_[i] = 0.0;
     stored_dvy_[i] = 0.0;
     stored_dvz_[i] = 0.0;
   }
  dv_memory_allocated_ = true;
}

float CoarseCCatIntervals::evaluate(DensityMap &em_map,
                                    SampledDensityMap &model_map,
                                    std::vector<float> &dvx,
                                    std::vector<float> &dvy,
                                    std::vector<float> &dvz,
                                    float scalefac, bool lderiv,
                                    unsigned long eval_interval) {
// eval_interval is the interval size before recalculating the CC score

  unsigned int number_of_particles=model_map.get_xyzr_particles().size();
  // If the function requires to be evaluated
  if  (calls_counter_ % eval_interval == 0) {
    // The base evaluate function calculates the derivates of the EM term.
    stored_cc_ = CoarseCC::evaluate(em_map, model_map, dvx, dvy, dvz,
                                    scalefac, lderiv);

    calls_counter_ = 1;
    if (lderiv) {
      // sync the derivatives.
      allocate_derivatives_array(number_of_particles);
      for (unsigned int i=0;i < number_of_particles;i++) {
        stored_dvx_[i] = dvx[i];
        stored_dvy_[i] = dvy[i];
        stored_dvz_[i] = dvz[i];
      }
    }
  }
  // If the evaluation was not required, return the previously stored values
  else {
    for (unsigned int i=0;i<number_of_particles;i++) {
      if (lderiv) {
        dvx[i] = stored_dvx_[i];
        dvy[i] = stored_dvy_[i];
        dvz[i] = stored_dvz_[i];
      }
    }
    ++calls_counter_;
  }

  return stored_cc_;
}

IMPEM_END_NAMESPACE
