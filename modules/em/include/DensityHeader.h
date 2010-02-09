/**
 *  \file DensityHeader.h
 *  \brief Metadata for a density file.
 *
 *  Copyright 2007-2010 Sali Lab. All rights reserved.
 *
 */

#ifndef IMPEM_DENSITY_HEADER_H
#define IMPEM_DENSITY_HEADER_H

#include "config.h"
#include "def.h"
#include <IMP/exception.h>
#include <iostream>
#include <fstream>
#include <cstring>

IMPEM_BEGIN_NAMESPACE

/** \todo change so that the att will not be encoded but loaded from
    a conf file (Keren) */
class IMPEMEXPORT DensityHeader
{

public:
  DensityHeader() {
    is_resolution_set_= false;
    top_calculated_ = false;
    // Initialize some of the parameters of the header
    nx=0;ny=0;nz=0;
    data_type=5;
    nxstart=1; nystart=1; nzstart=1;

    mx = nxstart + nx - 1; my = nystart + ny - 1; mz = nzstart + nz - 1;
    xorigin_ = yorigin_ = zorigin_ = 0.0;
    alpha=90. ; beta=90. ; gamma=90.;
    mapc =1; mapr=2; maps=3;
    ispg=0;
    nsymbt=0;
    Objectpixelsize_=1.0;
    machinestamp = 0;
    nlabl=0;
    strcpy(map,"MAP \0");
    magic=6;
  }

  // If a copy constructor is not defined in a class, the compiler itself
  // defines one. This will ensure a shallow copy.
  // If the class does not have pointer variables with dynamically allocated
  // memory, then one need not worry about defining a
  // copy constructor. It can be left to the compiler's discretion.
  // But if the class has pointer variables and has some dynamic memory
  // allocations, then it is a must to have a copy constructor.

  //! gets the upper bound of the grid
  /** \param[in] ind The dimension index x:=0,y:=1,z:=2
      \return the coordinate value in angstroms
   */
  emreal get_top(int ind) const
  {
    IMP_USAGE_CHECK(top_calculated_,
               " DensityHeader::get_top  the top coordinates of the map "
               << "have not been setup yet " << std::endl,ValueException);
    if (ind==0) return xtop_;
    if (ind==1) return ytop_;
    return ztop_;

  }
  //! Compute the farthest point of the grid.
  /**
    /param[in] force if true then the top point is calculated
               even if it has already been calcualted.
   */
  void compute_xyz_top(bool force=false);


  //! Update the dimensions of the map to be (nnx,nny,nnz)
  //!The origin of the map does not change
  /**
     /param[in] nnx the new number of voxels on the X axis
     /param[in] nny the new number of voxels on the Y axis
     /param[in] nnz the new number of voxels on the Z axis
   */
  void update_map_dimensions(int nnx,int nny,int nnz);

  //! Update the cell dimensions of the map multiplying the number of voxels
  //! along each direction by the Objectpixelsize
  void update_cell_dimensions();

  void show(std::ostream& out=std::cout) const {
    out<< "nx: " << nx << " ny: " << ny << " nz: " << nz << std::endl;
    out<<"data_type: " << data_type << std::endl;
    out<<"nxstart: " << nxstart << " nystart: " << nystart <<" nzstart: "
      << nzstart << std::endl;
    out<<"mx: "<< mx <<" my:" << my << " mz: " << mz << std::endl;
    out<< "xlen: " << xlen <<" ylen: " << ylen <<" zlen: " << zlen
      << std::endl;
    out<<"alpha : " << alpha << " beta: " << beta <<" gamma: "<< gamma
      << std::endl;
    out<< "mapc : " << mapc << " mapr: " << mapr <<" maps: " << maps
      << std::endl;
    out<< "dmin: " << dmin << " dmax: " << dmax << " dmean: " << dmean
      << std::endl;
    out<<"ispg: " << ispg << std::endl;
    out<<"nsymbt: " << nsymbt << std::endl;
    out<< "user: " << user << std::endl;
    out<<"xorigin: " << xorigin_ << " yorigin: "<< yorigin_ <<" zorigin: "
       << zorigin_ << std::endl;
    out<<"map: " << map << std::endl;
    out<< "Objectpixelsize: " << Objectpixelsize_ << std::endl;
    out<< "Resolution: " << resolution_ << std::endl;
    out<< "machinestamp: " << machinestamp << std::endl;
    out<<"rms: " << rms << std::endl;
    out<<"nlabl: " << nlabl <<std::endl;
    for(int i=0;i<nlabl;i++)
      out<< "comments[" << i << "] = ->" <<  comments[i] << "<-" << std::endl;
  }
  friend std::ostream& operator<<(std::ostream& s, const DensityHeader &v) {
    v.show(s);
    return s;
  }


  static const unsigned short MAP_FIELD_SIZE   =  4;
  static const unsigned short USER_FIELD_SIZE     =  25;
  static const unsigned short COMMENT_FIELD_NUM_OF     =  10;
  static const unsigned short COMMENT_FIELD_SINGLE_SIZE    =  80;
  //! map size (voxels) x-dimension
  int nx;
  //! map size (voxels) y-dimension
  int ny;
  //! map size (voxels) z-dimension
  int nz;
  //! How many bits are used to store the density of a single voxel
  //! (used in MRC format)
  int data_type;
  //! number of first columns in map (x-dimension)
  int nxstart;
  //! number of first columns in map (y-dimension)
  int nystart;
  //! number of first columns in map (z-dimension)
  int nzstart;

  int mx, my, mz; // Number of intervals along each dimension
  float xlen,ylen,zlen; //Cell dimensions (angstroms)
  float alpha, beta, gamma; //Cell angles (degrees)
  //! Axes corresponding to columns (mapc), rows (mapr) and sections (maps)
  //! (1,2,3 for x,y,z)
  int mapc, mapr, maps;
  float dmin,dmax,dmean; //Minimum, maximum and mean density value
  int ispg; //space group number 0 or 1
  int nsymbt; //Number of bytes used for symmetry data
  int user[USER_FIELD_SIZE];//extra space used for anything
  char map[MAP_FIELD_SIZE]; //character string 'MAP ' to identify file type
  int machinestamp; //machine stamp (0x11110000 bigendian, 0x44440000 little)
  float rms; //RMS deviation of map from mean density
  int nlabl; //Number of labels being used
  //! text comments \todo MRC: labels[10][80] - should it be a different field?
  char comments[COMMENT_FIELD_NUM_OF][COMMENT_FIELD_SINGLE_SIZE];
  //! magic byte for machine platform (~endian), OS-9=0, VAX=1, Convex=2,
  //! SGI=3, Sun=4, Mac(Motorola)=5, PC,IntelMac=6
  int magic;
  float voltage; //Voltage of electron microscope
  float Cs;  //Cs of microscope
  float Aperture;  //Aperture used
  float Magnification;  //Magnification
  float Postmagnification; //Postmagnification (of energy filter)
  float Exposuretime; //Exposuretime
  float Microscope;  //Microscope
  float Pixelsize; //Pixelsize - used for the microscope CCD camera
  float CCDArea;  //CCDArea
  float Defocus;  //Defocus
  float Astigmatism;//Astigmatism
  float AstigmatismAngle; //Astigmatism Angle
  float FocusIncrement;//Focus-Increment
  float CountsPerElectron;//Counts/Electron
  float Intensity;//Intensity
  float EnergySlitwidth;//Energy slitwidth of energy filter
  float EnergyOffset; //Energy offset of Energy filter
  float Tiltangle;//Tiltangle of stage
  float Tiltaxis;//Tiltaxis
  float MarkerX;//Marker_X coordinate
  float MarkerY;//Marker_Y coordinate
  int lswap;

  //! Returns the resolution of the map
  inline float get_resolution() const {
    IMP_INTERNAL_CHECK(is_resolution_set_,
                       "The resolution was not set"<<std::endl);
   return resolution_;
  }
  //! Return if the resolution has been set
  inline bool get_has_resolution() const {
    return is_resolution_set_;
  }
  //! Sets the resolution of the map
  inline void set_resolution(float resolution) {
    is_resolution_set_=true;
    resolution_=resolution;}
  /** \note Use DensityMap::update_voxel_size() to set the spacing value.
  */
  inline float get_spacing() const {return Objectpixelsize_;}
  //! Returns the origin on the map (x-coordinate)
  inline float get_xorigin() const {return xorigin_;}
  //! Returns the origin on the map (y-coordinate)
  inline float get_yorigin() const {return yorigin_;}
  //! Returns the origin on the map (z-coordinate)
  inline float get_zorigin() const {return zorigin_;}
  //! Returns the origin on the map
  /**
  \param[in] i the relevant coordinate (0:x, 1:y, 2:z)
  \exception ValueException if the value of i is out of range.
  */
  inline float get_origin(int i) const {
    IMP_USAGE_CHECK(i >= 0 && i <= 2,
              "The origin coordinate should be between 0 and 2",
              ValueException);
    switch (i) {
      case 0: return get_xorigin();
      case 1: return get_yorigin();
      default: return get_zorigin();
    }
  }
  //! Sets the origin on the map (x-coordinate).
  /**
   \note This is an absolute position in space.
   */
  inline void set_xorigin(float x)  {xorigin_=x; top_calculated_=false;}
  //! Sets the origin on the map (y-coordinate)
  /**
    \note This is an absolute position in space.
  */
  inline void set_yorigin(float y)  {yorigin_=y; top_calculated_=false;}
  //! Sets the origin on the map (z-coordinate)
  /**
   \note This is an absolute position in space.
  */
  inline void set_zorigin(float z)  {zorigin_=z; top_calculated_=false;}
  //! True if the top coodinates (bounding-box) are calculated
  inline bool is_top_calculated() const { return top_calculated_;}
  float Objectpixelsize_; //this is the actual pixelsize
protected:
  float xtop_, ytop_, ztop_; // The upper bound for the x,y and z grid.
  float xorigin_, yorigin_, zorigin_; //Origin used for transforms
  bool top_calculated_;
  float resolution_;
  bool is_resolution_set_;
};

IMPEM_END_NAMESPACE

#endif  /* IMPEM_DENSITY_HEADER_H */
