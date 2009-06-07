/**
 *  \file ParticlesAccessPoint.h
 *  \brief Implements particle iterator
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 *
 */

#ifndef IMPEM_PARTICLES_ACCESS_POINT_H
#define IMPEM_PARTICLES_ACCESS_POINT_H

#include "config.h"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <IMP/algebra/Vector3D.h>

IMPEM_BEGIN_NAMESPACE

class IMPEMEXPORT ParticlesAccessPoint
{
public:
  // implements particle iterator
  virtual ~ParticlesAccessPoint(){}
  virtual int get_size() const =0;
  virtual float get_x(unsigned int index)const=0;
  virtual float get_y(unsigned int index)const=0;
  virtual float get_z(unsigned int index)const=0;
  virtual float get_r(unsigned int index)const=0;
  virtual float get_w(unsigned int index)const=0;
  virtual void set_x(unsigned int index, float xval)=0;
  virtual void set_y(unsigned int index, float yval)=0;
  virtual void set_z(unsigned int index, float zval)=0;
  virtual algebra::Vector3D get_centroid() const=0;
};




class IMPEMEXPORT ParticlesProvider : public ParticlesAccessPoint
{
public:
  ~ParticlesProvider(){}
  ParticlesProvider() {}
  // The constructor reads the file
  ParticlesProvider(const char *filename) {
    read(filename);
  }

  // Translate function
  void translate(float xo,float yo,float zo) {
    int n = get_size();
    for (int i=0;i<n;i++) {
      x_[i] += xo; y_[i] += yo; z_[i] += zo;
    }
  }

  void read(const char *filename) {
    std::fstream fs;
    int i,ncd;
    float x, y, z, r, w;

    fs.open(filename, std::fstream::in );
    if(!fs.fail()) {
      fs >> ncd; // Read number of coordinates
      // Get values
      for (i=0;i<ncd;i++) {
        fs >> x >> y >> z >> r >> w;
        append(x, y, z, r, w);
      }
    }
    fs.close();
  }

  void append(float x, float y, float z, float r, float w) {
    x_.push_back(x);
    y_.push_back(y);
    z_.push_back(z);
    radii_.push_back(r);
    weights_.push_back(w);
  }

  int get_size() const {return x_.size();}
  float get_x(unsigned int index)const {return x_[index];}
  float get_y(unsigned int index)const {return y_[index];}
  float get_z(unsigned int index)const {return z_[index];}
  float get_r(unsigned int index)const {return radii_[index];}
  float get_w(unsigned int index)const {return weights_[index];}
  void set_x(unsigned int index, float xval) {
     if (index < x_.size() && index >= 0) x_[index] = xval;
  }
  void set_y(unsigned int index, float yval) {
    if (index < y_.size() && index >= 0) y_[index] = yval;
  }
  void set_z(unsigned int index, float zval) {
    if (index < z_.size() && index >= 0) z_[index] = zval;
  }
  algebra::Vector3D get_centroid() const{
    float x_mean=0.;
    float y_mean=0.;
    float z_mean=0.;
    for(unsigned int i=0;i<x_.size();i++) {
      x_mean += x_[i];
      y_mean += y_[i];
      z_mean += z_[i];
    }
    x_mean /= x_.size();
    y_mean /= x_.size();
    z_mean /= x_.size();
    return algebra::Vector3D(x_mean,y_mean,z_mean);
  }
protected:
  std::vector<float> x_, y_, z_, radii_, weights_;
};

IMPEM_END_NAMESPACE

#endif  /* IMPEM_PARTICLES_ACCESS_POINT_H */
