#include "CoarseCCatIntervals.h"

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
                                    const ParticlesAccessPoint &access_p,
                                    std::vector<float> &dvx,
                                    std::vector<float> &dvy,
                                    std::vector<float> &dvz,
                                    float scalefac, bool lderiv,
                         // the interval size before recalculating the CC score
                                    unsigned long eval_interval)
{


  // If the function requires to be evaluated
  if  (calls_counter_ % eval_interval == 0) {

    // we should sync the derivatives. store_dv holds the EM contribution of
    // the derivatives.  The base evaluate function adds the EM contribution
    // to the existing derivatives.

    stored_cc_ = CoarseCC::evaluate(em_map, model_map, access_p, dvx, dvy, dvz,
                                    scalefac, lderiv);

    calls_counter_ = 1;

    if (lderiv) {
      // sync the derivatives. Now remove the additional contributions
      // (currently stored in store_dv) from the new values
      allocate_derivatives_array(access_p.get_size());
      for (int i=0;i<access_p.get_size();i++) {
        stored_dvx_[i] = dvx[i];
        stored_dvy_[i] = dvy[i];
        stored_dvz_[i] = dvz[i];
      }
    }
  }
  // If the function does not require to evaluate, return the previously
  // stored CCC and add the derivatives to the general terms
  else {
    for (int i=0;i<access_p.get_size();i++) {
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
