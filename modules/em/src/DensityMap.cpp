/**
 *  \file DensityMap.cpp
 *  \brief Class for handling density maps.
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 *
 */

#include <IMP/em/DensityMap.h>
#include <climits>

IMPEM_BEGIN_NAMESPACE

DensityMap::DensityMap()
{
  loc_calculated_ = false;
  normalized_ = false;
  rms_calculated_ = false;
  x_loc_ = NULL;
  y_loc_ = NULL;
  z_loc_ = NULL;
  data_ = NULL;
}



//TODO - update the copy cons
DensityMap::DensityMap(const DensityMap &other)
{
  header_ = other.header_;
  long size = get_number_of_voxels();
  data_ = new emreal[size];
  x_loc_ = new float[size];
  y_loc_ = new float[size];
  z_loc_ = new float[size];
  for (long i = 0; i < size; i++) {
    data_[i] = other.data_[i];
  }
  loc_calculated_ = other.loc_calculated_;
  if (loc_calculated_) {
    for (long i = 0; i < header_.nx * header_.ny * header_.nz; i++) {
      x_loc_[i] = other.x_loc_[i];
      y_loc_[i] = other.y_loc_[i];
      z_loc_[i] = other.z_loc_[i];
    }
  }

  data_allocated_ = other.data_allocated_;
  normalized_ = other.normalized_;
  rms_calculated_ = other.rms_calculated_;
}

DensityMap& DensityMap::operator=(const DensityMap& other)
{
  if (this == &other) { // protect against invalid self-assignment
    return *this;
  }

  DensityMap *a = new DensityMap(other);
  return *a;
}


DensityMap::~DensityMap()
{
  delete[] data_;
  delete[] x_loc_;
  delete[] y_loc_;
  delete[] z_loc_;
}

void DensityMap::CreateVoidMap(const int &nx, const int &ny, const int &nz)
{
  long nvox = nx*ny*nz;
  data_ = new emreal[nvox];
  for (long i=0;i<nvox;i++) {
    data_[i]=0.0;
  }
  header_.nx = nx;
  header_.ny = ny;
  header_.nz = nz;
}

#ifndef IMP_NO_DEPRECATED
void DensityMap::Read(const char *filename, MapReaderWriter &reader) {
  // TODO: we need to decide who does the allocation ( mapreaderwriter or
  // density)? if we keep the current implementation ( mapreaderwriter )
  // we need to pass a pointer to data_
  float *f_data;
  reader.Read(filename, &f_data, header_);
  float2real(f_data, &data_);
  delete[] f_data;
  normalized_ = false;
  calcRMS();
  calc_all_voxel2loc();
  header_.compute_xyz_top();
  if (header_.get_spacing() == 1.0) {
    IMP_WARN("The pixel size is set to the default value 1.0."<<
              "Please make sure that this is indeed the pixel size of the map");
  }
}
#endif

DensityMap* read_map(const char *filename, MapReaderWriter &reader)
{
  // TODO: we need to decide who does the allocation ( mapreaderwriter or
  // density)? if we keep the current implementation ( mapreaderwriter )
  // we need to pass a pointer to data_
  DensityMap *m= new DensityMap();
  float *f_data;
  reader.Read(filename, &f_data, m->header_);
  m->float2real(f_data, &m->data_);
  delete[] f_data;
  m->normalized_ = false;
  m->calcRMS();
  m->calc_all_voxel2loc();
  m->header_.compute_xyz_top();
  if (m->header_.get_spacing() == 1.0) {
    IMP_WARN("The pixel size is set to the default value 1.0."<<
              "Please make sure that this is indeed the pixel size of the map");
  }
  return m;
}

void DensityMap::float2real(float *f_data, emreal **r_data)
{
  long size = get_number_of_voxels();
  (*r_data)= new emreal[size];
  for (long i=0;i<size;i++){
    (*r_data)[i]=(emreal)(f_data)[i];
  }
}

void DensityMap::real2float(emreal *r_data, float **f_data)
{
  long size = get_number_of_voxels();
  (*f_data)= new float[size];
  for (long i=0;i<size;i++){
    (*f_data)[i]=(float)(r_data)[i];
  }
}

#ifndef IMP_DEPRECATED

void DensityMap::Write(const char *filename, MapReaderWriter &writer) {
  IMP::em::write_map(this, filename, writer);
}
#endif

void write_map(DensityMap *d, const char *filename, MapReaderWriter &writer)
{
  float *f_data;
  d->real2float(d->data_, &f_data);
  writer.Write(filename, f_data, d->header_);
  delete[] f_data;
}

long DensityMap::get_number_of_voxels() const {
  return header_.nx * header_.ny * header_.nz;
}

float DensityMap::voxel2loc(const int &index, int dim) const
{
  IMP_check(loc_calculated_,
            "locations should be calculated prior to calling this function",
            InvalidStateException);
//   if (!loc_calculated_)
//     calc_all_voxel2loc();
  IMP_check(dim >= 0 && dim <= 2,
            "the dim index should be 0-2 (x-z) dim value:" << dim,
            ValueException);
  if (dim==0) {
    return x_loc_[index];
    }
  else if (dim==1) {
    return y_loc_[index];
    }
  return z_loc_[index];
}

long DensityMap::xyz_ind2voxel(int voxx,int voxy,int voxz) const{
  return voxz * header_.nx * header_.ny + voxy * header_.nx + voxx;
}

long DensityMap::loc2voxel(float x,float y,float z) const
{
  IMP_check(is_part_of_volume(x,y,z),
            "The point is not part of the grid", IndexException);
  int ivoxx=(int)floor((x-header_.get_xorigin())/header_.Objectpixelsize);
  int ivoxy=(int)floor((y-header_.get_yorigin())/header_.Objectpixelsize);
  int ivoxz=(int)floor((z-header_.get_zorigin())/header_.Objectpixelsize);
  return xyz_ind2voxel(ivoxx,ivoxy,ivoxz);
}

bool DensityMap::is_xyz_ind_part_of_volume(int ix,int iy,int iz) const
{
  if( ix>=0 && ix<header_.nx &&
      iy>=0 && iy<header_.ny &&
      iz>=0 && iz<header_.nz )
    return true;
  return false;
}


bool DensityMap::is_part_of_volume(float x,float y,float z) const
{
  if( x>=header_.get_xorigin() && x<=header_.get_top(0) &&
      y>=header_.get_yorigin() && y<=header_.get_top(1) &&
      z>=header_.get_zorigin() && z<=header_.get_top(2) ) {
    return true;

  }
  else {
    return false;

  }
}

emreal DensityMap::get_value(float x, float y, float z) const {
  return data_[loc2voxel(x,y,z)];
}

emreal DensityMap::get_value(long index) const {
  IMP_check(index >= 0 && index < get_number_of_voxels(),
            "The index " << index << " is not part of the grid",
            IndexException);
  return data_[index];
}

void DensityMap::set_value(long index,emreal value) {
  IMP_check(index >= 0 && index < get_number_of_voxels(),
            "The index " << index << " is not part of the grid",
            IndexException);
  data_[index]=value;
  normalized_ = false;
  rms_calculated_ = false;
}

void DensityMap::set_value(float x, float y, float z,emreal value) {
  data_[loc2voxel(x,y,z)]=value;
  normalized_ = false;
  rms_calculated_ = false;
}


void DensityMap::reset_voxel2loc() {
  loc_calculated_=false;
  delete [] x_loc_;
  delete [] y_loc_;
  delete [] z_loc_;
  x_loc_=NULL;y_loc_=NULL;z_loc_=NULL;
}
void DensityMap::calc_all_voxel2loc()
{
  if (loc_calculated_)
    return;

  long nvox = get_number_of_voxels();
  x_loc_ = new float[nvox];
  y_loc_ = new float[nvox];
  z_loc_ = new float[nvox];

  int ix=0,iy=0,iz=0;
  for (long ii=0;ii<nvox;ii++) {
    x_loc_[ii] =  ix * header_.Objectpixelsize + header_.get_xorigin();
    y_loc_[ii] =  iy * header_.Objectpixelsize + header_.get_yorigin();
    z_loc_[ii] =  iz * header_.Objectpixelsize + header_.get_zorigin();

    // bookkeeping
    ix++;
    if (ix == header_.nx) {
      ix = 0;
      ++iy;
      if (iy == header_.ny) {
        iy = 0;
        ++iz;
      }
    }
  }
  loc_calculated_ = true;
}

void DensityMap::std_normalize()
{
  if (normalized_)
    return;

  float max_value=-1e40, min_value=1e40;
  float inv_std = 1.0/calcRMS();
  float mean = header_.dmean;
  long nvox = get_number_of_voxels();

  for (long ii=0;ii<nvox;ii++) {
    data_[ii] = (data_[ii] - mean) * inv_std;
    if(data_[ii] > max_value) max_value = data_[ii];
    if(data_[ii] < min_value) min_value = data_[ii];
  }
  normalized_ = true;
  rms_calculated_ = true;
  header_.rms = 1.;
  header_.dmean = 0.0;
  header_.dmin = min_value;
  header_.dmax = max_value;
}



emreal DensityMap::calcRMS()
{

  if (rms_calculated_) {
    return header_.rms;
  }

  emreal max_value=-1e40, min_value=1e40;
  long  nvox = get_number_of_voxels();
  emreal meanval = .0;
  emreal stdval = .0;

  for (long ii=0;ii<nvox;ii++) {
    meanval += data_[ii];
    stdval += data_[ii] * data_[ii];
    if(data_[ii] > max_value) max_value = data_[ii];
    if(data_[ii] < min_value) min_value = data_[ii];

  }

  header_.dmin=min_value;
  header_.dmax=max_value;


  meanval /=  nvox;
  header_.dmean = meanval;

  stdval = sqrt(stdval/nvox-meanval*meanval);
  header_.rms = stdval;

  return stdval;
}

//!  Set the density voxels to zero and reset the managment flags.
void DensityMap::reset_data(float val)
{
  for (long i = 0; i < get_number_of_voxels(); i++) {
    data_[i] = val;
  }
  normalized_ = false;
  rms_calculated_ = false;
}



void DensityMap::set_origin(float x, float y, float z)
{
  header_.set_xorigin(x); header_.set_yorigin(y); header_.set_zorigin(z);
  // We have to compute the xmin,xmax, ... values again after
  // changing the origin
  header_.compute_xyz_top();
  reset_voxel2loc();
  calc_all_voxel2loc();
}




bool DensityMap::same_origin(const DensityMap &other) const
{
  if( fabs(get_header()->get_xorigin()-other.get_header()->get_xorigin())<EPS &&
      fabs(get_header()->get_yorigin()-other.get_header()->get_yorigin())<EPS &&
      fabs(get_header()->get_zorigin()-other.get_header()->get_zorigin())<EPS)
    return true;
  return false;
}

bool DensityMap::same_dimensions(const DensityMap &other) const
{
  if (get_header()->nx==other.get_header()->nx &&
      get_header()->ny==other.get_header()->ny &&
      get_header()->nz==other.get_header()->nz)
    return true;
  return false;
}

bool DensityMap::same_voxel_size(const DensityMap &other) const
{
  if(fabs(get_header()->Objectpixelsize
          - other.get_header()->Objectpixelsize) < EPS)
    return true;
  return false;
}

algebra::Vector3D DensityMap::get_centroid(emreal threshold)  const{
  emreal max_val = get_max_value();
  IMP_check(threshold < max_val,
            "The input threshold with value " << threshold
            << " is higher than the maximum density in the map " << max_val,
            ValueException);
  float x_centroid = 0.0;
  float y_centroid = 0.0;
  float z_centroid = 0.0;
  int counter = 0;
  long nvox = get_number_of_voxels();
  for (long i=0;i<nvox;i++) {
    if (data_[i] <= threshold) {
      continue;
    }
    x_centroid += voxel2loc(i,0);
    y_centroid += voxel2loc(i,1);
    z_centroid += voxel2loc(i,2);
    counter += 1;
  }
  //counter will not be 0 since we checked that threshold is within
  //the map densities.
  x_centroid /= counter;
  y_centroid /= counter;
  z_centroid /= counter;
  return algebra::Vector3D(x_centroid,y_centroid,z_centroid);
}
emreal DensityMap::get_max_value() const{
  emreal max_val = -1.0 * INT_MAX;
  long nvox = get_number_of_voxels();
  for (long i=0;i<nvox;i++) {
    if (data_[i] > max_val) {
      max_val = data_[i];
    }
  }
  return max_val;
}

emreal DensityMap::get_min_value() const{
  emreal min_val = INT_MAX;
  long nvox = get_number_of_voxels();
  for (long i=0;i<nvox;i++) {
    if (data_[i] < min_val) {
      min_val = data_[i];
    }
  }
  return min_val;
}

std::string DensityMap::get_locations_string(float t)  {
  std::stringstream out;
  long nvox = get_number_of_voxels();
  float x,y,z;
  for (long i=0;i<nvox;i++) {
    if (data_[i] > t) {
      x=voxel2loc(i,0);
      y=voxel2loc(i,1);
      z=voxel2loc(i,2);
      out<<x << " " << y << " " << z << std::endl;
    }
  }
  return out.str();
}

void DensityMap::multiply(float factor) {
  long size = header_.nx * header_.ny * header_.nz;
  for (long i=0;i<size;i++){
    data_[i]= factor*data_[i];
  }
}

void DensityMap::add(const DensityMap &other) {
  //check that the two maps have the same dimensions
  IMP_check(same_dimensions(other),
            "The dimensions of the two maps differ ( " << header_.nx
            << "," << header_.ny << "," << header_.nz << ") != ("
            << other.header_.nx << "," << other.header_.ny << ","
            << other.header_.nz << " ) ", ValueException);
  // check that the two maps have the same voxel size
  IMP_check(same_voxel_size(other),
            "The voxel sizes of the two maps differ ( "
            << header_.Objectpixelsize << " != "
            << other.header_.Objectpixelsize, ValueException);
  long size = header_.nx * header_.ny * header_.nz;
  for (long i=0;i<size;i++){
    data_[i]= data_[i] + other.data_[i];
  }
}


void DensityMap::pad(int nx, int ny, int nz,float val) {

  IMP_check(nx >= header_.nx && ny >= header_.ny && nz >= header_.nz,
            "The requested volume is smaller than the existing one",
            InvalidStateException);

  long new_size = nx*ny*nz;
  long cur_size = get_number_of_voxels();
  reset_voxel2loc();
  calc_all_voxel2loc();
  emreal * new_data = new emreal[new_size];
  for (long i = 0; i < new_size; i++) {
    new_data[i] = val;
  }
  int new_vox_x,new_vox_y,new_vox_z;
  long new_vox;
  for (long i = 0; i <  cur_size; i++) {
    float x = voxel2loc(i,0);
    float y = voxel2loc(i,1);
    float z = voxel2loc(i,2);
    new_vox_x=(int)floor((x-header_.get_xorigin())/header_.Objectpixelsize);
    new_vox_y=(int)floor((y-header_.get_yorigin())/header_.Objectpixelsize);
    new_vox_z=(int)floor((z-header_.get_zorigin())/header_.Objectpixelsize);
    new_vox =  new_vox_z * nx * ny + new_vox_y * nx + new_vox_x;
    new_data[new_vox] = data_[i];
  }
  header_.update_map_dimensions(nx,ny,nz);
  delete [] data_;
  data_ = new_data;
  calc_all_voxel2loc();
}


void DensityMap::update_voxel_size(float new_apix) {
  header_.Objectpixelsize = new_apix;
  header_.compute_xyz_top(true);
  reset_voxel2loc();
  calc_all_voxel2loc();
}

IMPEM_END_NAMESPACE
