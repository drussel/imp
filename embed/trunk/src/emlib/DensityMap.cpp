#include "DensityMap.h"
#include <climits>

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
  int size = header_.nx * header_.ny * header_.nz;
  data_ = new emreal[size];
  x_loc_ = new float[size];
  y_loc_ = new float[size];
  z_loc_ = new float[size];
  for (int i = 0; i < header_.nx * header_.ny * header_.nz; i++) {
    data_[i] = other.data_[i];
  }
  loc_calculated_ = other.loc_calculated_;
  if (loc_calculated_) {
    for (int i = 0; i < header_.nx * header_.ny * header_.nz; i++) {
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
  int nvox = nx*ny*nz;
  data_ = new emreal[nvox];
  for (int i=0;i<nvox;i++) {
    data_[i]=0.0;
  }
  header_.nx = nx;
  header_.ny = ny;
  header_.nz = nz;
}


void DensityMap::Read(const char *filename, MapReaderWriter &reader)
{
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
}

void DensityMap::float2real(float *f_data, emreal **r_data)
{
  int size = header_.nx * header_.ny * header_.nz;
  (*r_data)= new emreal[size]; 
  for (int i=0;i<size;i++){
    (*r_data)[i]=(emreal)(f_data)[i];
  }
}

void DensityMap::real2float(emreal *r_data, float **f_data)
{
  int size = header_.nx * header_.ny * header_.nz;
  (*f_data)= new float[size]; 
  for (int i=0;i<size;i++){
    (*f_data)[i]=(float)(r_data)[i];
  }
}



void DensityMap::Write(const char *filename, MapReaderWriter &writer)
{
  float *f_data;
  real2float(data_, &f_data);
  writer.Write(filename, f_data, header_);
  delete[] f_data;
}


float DensityMap::voxel2loc(const int &index, int dim) 
{
  if (!loc_calculated_) 
    calc_all_voxel2loc();
  if (dim==0) {
    return x_loc_[index];
    }
  else if (dim==1) {
    return y_loc_[index];
    }
  //TODO - add error handling, dim is not 0,1,2
  return z_loc_[index];
}

int DensityMap::loc2voxel(float x,float y,float z) const
{
  if (!part_of_volume(x,y,z)) {
    std::ostringstream msg;
    msg << " DensityMap::loc2voxel >> The point is not part of the grid \n";
    throw EMBED_OutOfRange(msg.str().c_str());
  }
  int ivoxx=(int)floor((x-header_.get_xorigin())/header_.Objectpixelsize);
  int ivoxy=(int)floor((y-header_.get_yorigin())/header_.Objectpixelsize);
  int ivoxz=(int)floor((z-header_.get_zorigin())/header_.Objectpixelsize);
  return ivoxz * header_.nx * header_.ny + ivoxy * header_.nx + ivoxx;
}

bool DensityMap::part_of_volume(float x,float y,float z) const
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

emreal DensityMap::get_value(float x, float y, float z) const
{
  return data_[loc2voxel(x,y,z)];
}

void DensityMap::calc_all_voxel2loc()
{
  if (loc_calculated_)
    return;

  int nvox = header_.nx * header_.ny * header_.nz;
  x_loc_ = new float[nvox];
  y_loc_ = new float[nvox];
  z_loc_ = new float[nvox];

  int ix=0,iy=0,iz=0;
  for (int ii=0;ii<nvox;ii++) {
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
  int nvox = header_.nx * header_.ny * header_.nz;

  for (int ii=0;ii<nvox;ii++) {
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
  int  nvox = header_.nx * header_.ny * header_.nz;
  emreal meanval = .0;
  emreal stdval = .0;

  for (int ii=0;ii<nvox;ii++) {
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
void DensityMap::reset_data()
{
  for (int i = 0; i < header_.nx * header_.ny * header_.nz; i++) {
    data_[i] = 0.0;
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
  loc_calculated_ = false;
  delete[] x_loc_;
  delete[] y_loc_;
  delete[] z_loc_;
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

Vector3 DensityMap::get_centroid(emreal threshold)  {
  emreal max_val = get_max_value(); 
  if (threshold >= max_val) {
    std::ostringstream msg;
    msg << "DensityMap::get_centroid >> The input threshold with value " << 
    threshold << " is higher than the maximum density in the map " <<  
      max_val << std::endl;
    throw EMBED_WrongValue(msg.str().c_str());
  }
  float x_centroid = 0.0;
  float y_centroid = 0.0;
  float z_centroid = 0.0;
  int counter = 0;
  int nvox = header_.nx*header_.ny*header_.nz;
  for (int i=0;i<nvox;i++) {
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
  return Vector3(x_centroid,y_centroid,z_centroid);
}
emreal DensityMap::get_max_value() const{
  float max_val = -1.0 * INT_MAX;
  int nvox = header_.nx*header_.ny*header_.nz;
  for (int i=0;i<nvox;i++) {
    if (data_[i] > max_val) {
      max_val = data_[i];
    }
  }
  return max_val;
}
