/**
 *  \file CoarseCC.cpp
 *  \brief Perform coarse fitting between two density objects.
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 *
 */

#include <IMP/em/CoarseCC.h>
#include <math.h>

IMPEM_BEGIN_NAMESPACE

/*Correlation function  */
float CoarseCC::evaluate(DensityMap &em_map,
                         SampledDensityMap &model_map,
                         const ParticlesAccessPoint &access_p,
                         std::vector<float> &dvx, std::vector<float>&dvy,
                         std::vector<float>&dvz, float scalefac, bool lderiv,
                         bool divide_by_rms)
{
  //resample the map for the particle provided
  // TODO(frido): rename: resample -> sample
  model_map.resample(access_p);

  if (divide_by_rms) {
    em_map.calcRMS();
    //determine a threshold for calculating the CC
    model_map.calcRMS();   // This function adequately computes the dmin value,
                          // the safest value for the threshold
  }
  emreal voxel_data_threshold=model_map.get_header()->dmin-EPS;
  // here we ask not to recalculate the rms ( already calculated)
  float escore = cross_correlation_coefficient(em_map, model_map,
                         voxel_data_threshold,false,divide_by_rms);
  //  std::cout <<" CoarseCC::evaluate >> cross_correlation_coefficient : "<<
  //escore << endl;
  escore = scalefac * (1. - escore);
  //compute the derivatives if required
  if (lderiv) {
    CoarseCC::calc_derivatives(em_map, model_map, access_p, scalefac,
                              dvx, dvy, dvz);
  }
  return escore;
}


float CoarseCC::cross_correlation_coefficient(const DensityMap &em_map,
                                              DensityMap &model_map,
                                              float voxel_data_threshold,
                                              bool recalc_ccnormfac,
                                              bool divide_by_rms)
{
  const DensityHeader *model_header = model_map.get_header();
  const DensityHeader *em_header = em_map.get_header();

  if (recalc_ccnormfac) {
    model_map.calcRMS();
  }

  const emreal *em_data = em_map.get_data();
  const emreal *model_data = model_map.get_data();

  //validity checks
  IMP_check(em_map.same_dimensions(model_map),
            "This function cannot handle density maps of different size. "
            << "First map dimensions : " << em_header->nx << " x "
            << em_header->ny << " x " << em_header->nz << "; "
            << "Second map dimensions: " << model_header->nx << " x "
            << model_header->ny << " x " << model_header->nz,
            InvalidStateException);
  IMP_check(em_map.same_voxel_size(model_map),
            "This function cannot handle density maps of different pixelsize. "
            << "First map pixelsize : " << em_header->Objectpixelsize << "; "
            << "Second map pixelsize: " << model_header->Objectpixelsize,
            InvalidStateException);

  // Take into account the possibility of a model map with zero rms
  if ((fabs(model_map.get_header()->rms-0.0)<EPS) && divide_by_rms)
    return 0.0;

  bool same_origin = em_map.same_origin(model_map);
  int  nvox = em_header->nx*em_header->ny*em_header->nz;
  emreal ccc = 0.0;

  if(same_origin){ // Fastest version
    for (int i=0;i<nvox;i++) {
      if (model_data[i] > voxel_data_threshold) {
        ccc += em_data[i]*model_data[i];
      }
    }
    // This formula does not assume normalization in the maps
    if (divide_by_rms) {
      ccc = (ccc-nvox*em_header->dmean*model_header->dmean)/
            (1.0* nvox*em_header->rms * model_header->rms);
    }
  }

  else  { // Compute the CCC taking into account the different origins

    model_map.get_header_writable()->compute_xyz_top();

    // Given the same size of the maps and the dimension order, the difference
    // between two positions in voxels is always the same

    // calculate the difference in voxels between the origin of the  model map
    // and the origin of the em map.
    float voxel_size = em_map.get_header()->Objectpixelsize;
    const DensityHeader *em_header = em_map.get_header();
    const DensityHeader *model_header = model_map.get_header();
    int ivoxx_shift = (int)floor((model_header->get_xorigin()
                                  - em_header->get_xorigin())
                                 / voxel_size);
    int ivoxy_shift = (int)floor((model_header->get_yorigin()
                                  - em_header->get_yorigin())
                                 / voxel_size);
    int ivoxz_shift = (int)floor((model_header->get_zorigin()
                                  - em_header->get_zorigin())
                                 / voxel_size);


    int j; // Index for em data
    int i; // Index for model data
    // calculate the shift in index of the origin of model_map in em_map
    // ( j can be negative)
    j = ivoxz_shift * em_header->nx * em_header->ny + ivoxy_shift
        * em_header->nx + ivoxx_shift;
    for (i=0;i<nvox;i++) {
      // if the voxel of the model is above the threshold
      if (model_data[i] > voxel_data_threshold) {
        // Check if the voxel belongs to the em map volume, and only then
        // compute the correlation
        if (j + i >= 0 && j + i < nvox)  {
          ccc += em_data[j+i] * model_data[i];
        }
      }
    }
    if (divide_by_rms) {
      ccc = (ccc-nvox*em_header->dmean*model_header->dmean)
            /(nvox*em_header->rms * model_header->rms);
    }
    //    std::cout << " ccc : " << ccc << " voxel# " << nvox
    //          << " norm factors (map,model) " << em_header->rms
    //          << "  " <<  model_header->rms << " means(map,model) "
    //          << em_header->dmean << " " << model_header->dmean << std::endl;
  }
  return ccc;
}

void CoarseCC::calc_derivatives(const DensityMap &em_map,
                               SampledDensityMap &model_map,
                               const ParticlesAccessPoint &access_p,
                               const float &scalefac,
                               std::vector<float> &dvx, std::vector<float>&dvy,
                               std::vector<float>&dvz)
{
  float tdvx = 0., tdvy = 0., tdvz = 0., tmp,rsq;
  int iminx, iminy, iminz, imaxx, imaxy, imaxz;


  const DensityHeader *model_header = model_map.get_header();
  const DensityHeader *em_header = em_map.get_header();
  const float *x_loc = model_map.get_x_loc();
  const float *y_loc = model_map.get_y_loc();
  const float *z_loc = model_map.get_z_loc();

  const emreal *em_data = em_map.get_data();

  float lim = (model_map.get_kernel_params())->get_lim();
  //lim = 0.00000001;

  int nvox = em_header->nx * em_header->ny * em_header->nz;
  int ivox;


  // validate that the model and em maps are not empty
  IMP_check(em_header->rms >= EPS,
            "EM map is empty ! em_header->rms = " << em_header->rms,
            InvalidStateException);
  IMP_check(model_header->rms >= EPS,
            "Model map is empty ! model_header->rms = " << em_header->rms,
            InvalidStateException);
  // Compute the derivatives
  for (int ii=0; ii<access_p.get_size(); ii++) {
    const KernelParameters::Parameters *params =
        model_map.get_kernel_params()->find_params(access_p.get_r(ii));
    model_map.calc_sampling_bounding_box(access_p.get_x(ii),
                                         access_p.get_y(ii),
                                         access_p.get_z(ii),
                                         params->get_kdist(),
                                         iminx, iminy, iminz,
                                         imaxx, imaxy, imaxz);
    tdvx = .0;tdvy=.0; tdvz=.0;
    for (int ivoxz=iminz;ivoxz<=imaxz;ivoxz++) {
      for (int ivoxy=iminy;ivoxy<=imaxy;ivoxy++) {
        ivox = ivoxz * em_header->nx * em_header->ny
               + ivoxy * em_header->nx + iminx;
        for (int ivoxx=iminx;ivoxx<=imaxx;ivoxx++) {
          float dx = x_loc[ivox] - access_p.get_x(ii);
          float dy = y_loc[ivox] - access_p.get_y(ii);
          float dz = z_loc[ivox] - access_p.get_z(ii);
          rsq = dx * dx + dy * dy + dz * dz;
          rsq = EXP(- rsq * params->get_inv_sigsq());
          tmp = (access_p.get_x(ii)-x_loc[ivox]) * rsq;
          if (std::abs(tmp) > lim) {
            tdvx += tmp * em_data[ivox];
          }
          tmp = (access_p.get_y(ii)-y_loc[ivox]) * rsq;
          if (std::abs(tmp) > lim) {
            tdvy += tmp * em_data[ivox];
          }
          tmp = (access_p.get_z(ii)-z_loc[ivox]) * rsq;
          if (std::abs(tmp) > lim) {
            tdvz += tmp * em_data[ivox];
          }
          ivox++;
        }
      }
    }
    tmp = access_p.get_w(ii) * 2.*params->get_inv_sigsq() * scalefac
          * params->get_normfac() /
          (1.0*nvox * em_header->rms * model_header->rms);
    dvx[ii] =  tdvx * tmp;
    dvy[ii] =  tdvy * tmp;
    dvz[ii] =  tdvz * tmp;
  }
}

IMPEM_END_NAMESPACE
