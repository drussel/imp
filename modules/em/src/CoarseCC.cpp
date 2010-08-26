/**
 *  \file CoarseCC.cpp
 *  \brief Perform coarse fitting between two density objects.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 *
 */

#include <IMP/em/CoarseCC.h>
#include <math.h>
#include <IMP/core/utility.h>

IMPEM_BEGIN_NAMESPACE

float CoarseCC::calc_score(
         DensityMap &em_map,
         SampledDensityMap &model_map,
         float scalefac,
         bool recalc_rms,bool resample,
         FloatPair norm_factors) {
  //resample the map for the particles provided
  if (resample) {
     model_map.resample();
  }

  if (recalc_rms) {
    em_map.calcRMS();
    //determine a threshold for calculating the CC
    model_map.calcRMS();   // This function adequately computes the dmin value,
                          // the safest value for the threshold
  }
  emreal voxel_data_threshold=model_map.get_header()->dmin-EPS;
  //rmss have already been calculated
  float escore = cross_correlation_coefficient(
                     em_map, model_map,
                     voxel_data_threshold,false,norm_factors);
  IMP_LOG(VERBOSE, "CoarseCC::evaluate parameters:  threshold:"
          << voxel_data_threshold << std::endl);
  IMP_LOG(VERBOSE, "CoarseCC::evaluate: the score is:" << escore << std::endl);
  escore = scalefac * (1. - escore);

  return escore;
}



namespace{
double cross_correlation_coefficient_internal(
      const DensityMap &grid1,
      const DensityMap &grid2,
      float grid2_voxel_data_threshold,
      FloatPair norm_factors) {

  const DensityHeader *grid2_header = grid2.get_header();
  const DensityHeader *grid1_header = grid1.get_header();

  const emreal *grid1_data = grid1.get_data();
  const emreal *grid2_data = grid2.get_data();

  bool same_origin = grid1.same_origin(grid2);
  long  nvox = grid1_header->get_number_of_voxels();
  emreal ccc = 0.0;

  if(same_origin){ // Fastest version
    IMP_LOG(IMP::VERBOSE,"calc CC with the same origin"<<std::endl);
    for (long i=0;i<nvox;i++) {
      if (grid2_data[i] > grid2_voxel_data_threshold) {
        if (grid1_data[i] >EPS) {
          ccc += grid1_data[i]*grid2_data[i];}
      }
    }
  }

  else  { // Compute the CCC taking into account the different origins
    IMP_LOG(IMP::VERBOSE,"calc CC with different origins"<<std::endl);
    // Given the same size of the maps and the dimension order, the difference
    // between two positions in voxels is always the same

    // calculate the difference in voxels between the origin of the  model map
    // and the origin of the em map.
    float voxel_size = grid1_header->get_spacing();
    int ivoxx_shift = (int)floor((grid2_header->get_xorigin()
                                  - grid1_header->get_xorigin())
                                 / voxel_size);
    int ivoxy_shift = (int)floor((grid2_header->get_yorigin()
                                  - grid1_header->get_yorigin())
                                 / voxel_size);
    int ivoxz_shift = (int)floor((grid2_header->get_zorigin()
                                  - grid1_header->get_zorigin())
                                 / voxel_size);


    long j; // Index for em data
    long i; // Index for model data
    // calculate the shift in index of the origin of model_map in em_map
    // ( j can be negative)
    j = ivoxz_shift * grid1_header->get_nx() * grid1_header->get_ny()
      + ivoxy_shift * grid1_header->get_nx() + ivoxx_shift;
    int num_elements=0;//needed for checks
    for (i=0;i<nvox;i++) {
      // if the voxel of the model is above the threshold
      if (grid2_data[i] > grid2_voxel_data_threshold) {
        ++num_elements;
        // Check if the voxel belongs to the em map volume, and only then
        // compute the correlation
        if (j + i >= 0 && j + i < nvox)  {
          ccc += grid1_data[j+i] * grid2_data[i];
        }
      }
    }
    IMP_INTERNAL_CHECK(num_elements>0,
                       "No voxels participated in the calculation"<<
                       " may be that the voxel_data_threshold:" <<
                       grid2_voxel_data_threshold <<" is off"<<std::endl);
  }
  if ((norm_factors.first >0.) && (norm_factors.second>0.)){
    ccc = (ccc-norm_factors.first)/norm_factors.second;
  }
  else{
    ccc = (ccc-nvox*grid1_header->dmean*grid2_header->dmean)
      /(nvox*grid1_header->rms * grid2_header->rms);
  }

  IMP_LOG(VERBOSE, " ccc : " << ccc << " voxel# " << nvox
          << " norm factors (map,model) " << grid1_header->rms
          << "  " <<  grid2_header->rms << " means(grid1,grid2) "
          << grid1_header->dmean << " " << grid2_header->dmean << std::endl);
  return ccc;
}
}

double CoarseCC::cross_correlation_coefficient(
                        const DensityMap &grid1,
                        const DensityMap &grid2,
                        float grid2_voxel_data_threshold,
                        bool allow_padding,
                        FloatPair norm_factors) {
  IMP_LOG(VERBOSE,"Going to calcualte correlation score with values: "<<
          "grid2_voxel_data_threshold:"<<
          grid2_voxel_data_threshold<<
          " allow_padding:"<<allow_padding<<
          " norm factors:"<<norm_factors.first<<","<<norm_factors.second <<
          "\n");
  //run vaildation checks
  const DensityHeader *grid2_header = grid2.get_header();
  const DensityHeader *grid1_header = grid1.get_header();

  IMP_INTERNAL_CHECK(grid2_header->dmax>grid2_voxel_data_threshold,
                     "voxel_data_threshold: " << grid2_voxel_data_threshold <<
                     " is not within the map range: " <<
                     grid2_header->dmin<<"-"<<
                     grid2_header->dmax<<std::endl);

  IMP_INTERNAL_CHECK(grid1_header->is_top_calculated(),
                     "Top should be calculated for grid1\n");

  IMP_INTERNAL_CHECK(grid2_header->is_top_calculated(),
                     "Top should be calculated for grid2\n");

  IMP_INTERNAL_CHECK(grid1.same_voxel_size(grid2),
                     "Both grids should have the same spacing\n");
  if (!allow_padding) {
  //additional validity checks
  IMP_USAGE_CHECK(grid1.same_dimensions(grid2),
            "This function cannot handle density maps of different size. "
      << "First map dimensions : " << grid1_header->get_nx() << " x "
      << grid1_header->get_ny() << " x " << grid1_header->get_nz() << "; "
      << "Second map dimensions: " << grid2_header->get_nx() << " x "
      << grid2_header->get_ny() << " x " << grid2_header->get_nz());
  IMP_USAGE_CHECK(grid1.same_voxel_size(grid2),
            "This function cannot handle density maps of different pixelsize. "
            << "First grid pixelsize : " << grid1_header->get_spacing() << "; "
            << "Second grid pixelsize: " << grid2_header->get_spacing());
    return cross_correlation_coefficient_internal(
                                 grid1,grid2,
                                 grid2_voxel_data_threshold,norm_factors);
  }
  else {
    IMP_LOG(VERBOSE,"calculated correlation bewteen padded maps\n");
    //create a padded version of the grids
    //copy maps to contain the same extent
    if (!get_interiors_intersect(
             get_bounding_box(&grid1),
             get_bounding_box(&grid2,grid2_voxel_data_threshold))){
      return 0.;
    }
    algebra::BoundingBox3D merged_bb=
      get_bounding_box(&grid1)+get_bounding_box(&grid2);
    DensityMap* padded_grid1=
      create_density_map(merged_bb,grid1_header->get_spacing());
    padded_grid1->add(grid1);
    DensityMap* padded_grid2=
      create_density_map(merged_bb,grid2_header->get_spacing());
    padded_grid2->add(grid2);
    padded_grid1->calcRMS();
    padded_grid2->calcRMS();
    IMP_LOG(VERBOSE,"calcaulte correlation internal");
    double score=cross_correlation_coefficient_internal(
                                 *padded_grid1,*padded_grid2,
                                 grid2_voxel_data_threshold,norm_factors);
    //release the padded versions of the grids
    delete (padded_grid1);
    delete (padded_grid2);
    return score;
  }
}


float CoarseCC::local_cross_correlation_coefficient(const DensityMap &em_map,
                                              DensityMap &model_map,
                                              float voxel_data_threshold,
                                              bool recalc_ccnormfac,
                                              bool divide_by_rms)
{
  IMP_INTERNAL_CHECK(model_map.get_header()->dmax>voxel_data_threshold,
                     "voxel_data_threshold: " << voxel_data_threshold <<
                     " is not within the map range: " <<
                     model_map.get_header()->dmin<<"-"<<
                     model_map.get_header()->dmax<<std::endl);
  const DensityHeader *model_header = model_map.get_header();
  const DensityHeader *em_header = em_map.get_header();

  if (recalc_ccnormfac) {
    model_map.calcRMS();
  }

  const emreal *em_data = em_map.get_data();
  const emreal *model_data = model_map.get_data();

  //validity checks
  IMP_USAGE_CHECK(em_map.same_voxel_size(model_map),
            "This function cannot handle density maps of different pixelsize. "
            << "First map pixelsize : " << em_header->get_spacing() << "; "
            << "Second map pixelsize: " << model_header->get_spacing());

  // Check if the model map has zero RMS
  if ((fabs(model_map.get_header()->rms-0.0)<EPS) && divide_by_rms) {
    IMP_WARN("The model map rms is zero, and the user ask to divide"<<
             " by rms. returning 0!"<<std::endl);
    return 0.0;
  }

  long  nvox = em_header->get_number_of_voxels();;
  emreal ccc = 0.0;
  emreal model_mean=0.;
  emreal em_mean=0.;
  emreal model_rms=0.;
  emreal em_rms=0.;
  int num=0;
IMP_LOG(IMP::VERBOSE,"calc local CC with different origins"<<std::endl);
  model_map.get_header_writable()->compute_xyz_top();

  // Given the same size of the maps and the dimension order, the difference
  // between two positions in voxels is always the same

  // calculate the difference in voxels between the origin of the  model map
  // and the origin of the em map.
  float voxel_size = em_map.get_header()->get_spacing();
  int ivoxx_shift = (int)floor((model_header->get_xorigin()
                                  - em_header->get_xorigin())
                                 / voxel_size);
  int ivoxy_shift = (int)floor((model_header->get_yorigin()
                                  - em_header->get_yorigin())
                                 / voxel_size);
  int ivoxz_shift = (int)floor((model_header->get_zorigin()
                                  - em_header->get_zorigin())
                                 / voxel_size);


   long j; // Index for em data
    long i; // Index for model data
    // calculate the shift in index of the origin of model_map in em_map
    // ( j can be negative)
    j = ivoxz_shift * em_header->get_nx() * em_header->get_ny() + ivoxy_shift
      * em_header->get_nx() + ivoxx_shift;
    int num_elements=0;//needed for checks
    for (i=0;i<nvox;i++) {
      // if the voxel of the model is above the threshold
      if (model_data[i] > voxel_data_threshold) {
        ++num_elements;
        // Check if the voxel belongs to the em map volume, and only then
        // compute the correlation
        if (j + i >= 0 && j + i < nvox)  {
          ccc += em_data[j+i] * model_data[i];
          num++;
          em_mean += em_data[j+i] ;
          model_mean += model_data[i];
          em_rms += em_data[j+i]*em_data[j+i];
          model_rms = model_data[i]*model_data[i];
        }
      }
    }
    em_mean = em_mean / num;
    model_mean = model_mean / num;
    em_rms = std::sqrt(em_rms/num-em_mean*em_mean);
    model_rms = std::sqrt(model_rms/num-model_mean*model_mean);
    IMP_INTERNAL_CHECK(num_elements>0,
                       "No voxels participated in the calculation"<<
                       " may be that the voxel_data_threshold:" <<
                       voxel_data_threshold <<" is off"<<std::endl);
    if (divide_by_rms) {
      //      ccc = (ccc-nvox*em_header->dmean*model_header->dmean)
      //    /(nvox*em_header->rms * model_header->rms);
            ccc = (ccc-num*em_mean*model_mean)
          /(num*em_rms * model_rms);
    }
  IMP_LOG(VERBOSE, " ccc : " << ccc << " voxel# " << num
          << " norm factors (map,model) " << em_rms
          << "  " <<  model_rms << " means(map,model) "
          << em_mean << " " << model_mean << std::endl);
  return ccc;
}

void CoarseCC::calc_derivatives(
             const DensityMap &em_map,
             const DensityMap &model_map,
             const Particles &model_ps,const FloatKey &w_key,
             KernelParameters *kernel_params,DistanceMask *dist_mask,
             const float &scalefac,
             std::vector<float> &dvx, std::vector<float>&dvy,
             std::vector<float>&dvz) {

  float tdvx = 0., tdvy = 0., tdvz = 0., tmp,rsq;
  int iminx, iminy, iminz, imaxx, imaxy, imaxz;

  const DensityHeader *model_header = model_map.get_header();
  const DensityHeader *em_header = em_map.get_header();
  const float *x_loc = model_map.get_x_loc();
  const float *y_loc = model_map.get_y_loc();
  const float *z_loc = model_map.get_z_loc();
  IMP_INTERNAL_CHECK(model_ps.size()==dvx.size(),
    "input derivatives array size does not match "<<
    "the number of particles in the model map\n");
  core::XYZRsTemp model_xyzr = core::XYZRsTemp(model_ps);
  //this would go away once we have XYZRW decorator
  const emreal *em_data = em_map.get_data();
  float lim = kernel_params->get_lim();
  long nvox = em_header->get_number_of_voxels();
  long ivox;
  // validate that the model and em maps are not empty
  IMP_USAGE_CHECK(em_header->rms >= EPS,
            "EM map is empty ! em_header->rms = " << em_header->rms);
  //it may be that CG takes a too large step, which causes the particles
  //to go outside of the density
  // if (model_header->rms <= EPS){
  // IMP_WARN("Model map is empty ! model_header->rms = " << model_header->rms
  //           <<" derivatives are not calculated. the model centroid is : " <<
  //           core::get_centroid(core::XYZsTemp(model_ps))<<
  //           " the map centroid is " << em_map.get_centroid()<<
  //                 "number of particles in model:"<<model_ps.size()
  //<<std::endl);
  // return;
  // }
  // Compute the derivatives
  for (unsigned int ii=0; ii<model_ps.size(); ii++) {
    float x,y,z;
    x=model_xyzr[ii].get_x();y=model_xyzr[ii].get_y();
    z=model_xyzr[ii].get_z();
    const RadiusDependentKernelParameters *params =
      kernel_params->get_params(
            model_xyzr[ii].get_radius());
    calc_local_bounding_box(model_map,
                            x,y,z,
                            params->get_kdist(),
                            iminx, iminy, iminz,
                            imaxx, imaxy, imaxz);
    tdvx = .0;tdvy=.0; tdvz=.0;
    for (int ivoxz=iminz;ivoxz<=imaxz;ivoxz++) {
      for (int ivoxy=iminy;ivoxy<=imaxy;ivoxy++) {
        ivox = ivoxz * em_header->get_nx() * em_header->get_ny()
          + ivoxy * em_header->get_nx() + iminx;
        for (int ivoxx=iminx;ivoxx<=imaxx;ivoxx++) {
          float dx = x_loc[ivox] - x;
          float dy = y_loc[ivox] - y;
          float dz = z_loc[ivox] - z;
          rsq = dx * dx + dy * dy + dz * dz;
          rsq = EXP(-rsq * params->get_inv_sigsq());
          tmp = (x-x_loc[ivox]) * rsq;
          if (std::abs(tmp) > lim) {
            tdvx += tmp * em_data[ivox];
          }
          tmp = (y-y_loc[ivox]) * rsq;
          if (std::abs(tmp) > lim) {
            tdvy += tmp * em_data[ivox];
          }
          tmp = (z-z_loc[ivox]) * rsq;
          if (std::abs(tmp) > lim) {
            tdvz += tmp * em_data[ivox];
          }
          ivox++;
        }
      }
      }
    tmp =model_ps[ii]->get_value(w_key) * 2.*params->get_inv_sigsq()
          * scalefac
          * params->get_normfac() /
          (1.0*nvox * em_header->rms * model_header->rms);
    dvx[ii] =  tdvx * tmp;
    dvy[ii] =  tdvy * tmp;
    dvz[ii] =  tdvz * tmp;
  }//particles
}


IMPEM_END_NAMESPACE
