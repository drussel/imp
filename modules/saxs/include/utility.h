/**
 *  \file saxs/utility.h
 *  \brief Functions to deal with very common saxs operations
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
*/

#ifndef IMPSAXS_UTILITY_H
#define IMPSAXS_UTILITY_H

#include "saxs_config.h"
#include "FormFactorTable.h"
#include <IMP/exception.h>
#include <IMP/core/XYZ.h>

IMPSAXS_BEGIN_NAMESPACE

inline void get_coordinates(const Particles& particles,
                            std::vector<algebra::Vector3D>& coordinates) {
  // copy everything in advance for fast access
  coordinates.resize(particles.size());
  for (unsigned int i=0; i<particles.size(); i++) {
    coordinates[i] = core::XYZ(particles[i]).get_coordinates();
  }
}

inline void get_form_factors(const Particles& particles,
                             FormFactorTable* ff_table,
                             Floats& form_factors,
                             FormFactorType ff_type) {
  form_factors.resize(particles.size());
  for (unsigned int i=0; i<particles.size(); i++) {
    form_factors[i] = ff_table->get_form_factor(particles[i], ff_type);
  }
}

//! compute max distance
inline Float compute_max_distance(const Particles& particles) {
  Float max_dist2 = 0;
  std::vector<algebra::Vector3D> coordinates(particles.size());
  get_coordinates(particles, coordinates);
  for (unsigned int i = 0; i < coordinates.size(); i++) {
    for (unsigned int j = i + 1; j < coordinates.size(); j++) {
      Float dist2 = get_squared_distance(coordinates[i], coordinates[j]);
      if(dist2 > max_dist2)
        max_dist2 = dist2;
    }
  }
  return sqrt(max_dist2);
}

//! compute max distance between pairs of particles one from particles1
//! and the other from particles2
inline Float compute_max_distance(const Particles& particles1,
                                  const Particles& particles2) {
  Float max_dist2 = 0;
  std::vector<algebra::Vector3D> coordinates1, coordinates2;
  get_coordinates(particles1, coordinates1);
  get_coordinates(particles2, coordinates2);

  for (unsigned int i = 0; i < coordinates1.size(); i++) {
    for (unsigned int j = i + 1; j < coordinates2.size(); j++) {
      Float dist2 = get_squared_distance(coordinates1[i], coordinates2[j]);
      if(dist2 > max_dist2)
        max_dist2 = dist2;
    }
  }
  return sqrt(max_dist2);
}

//! compute radius_of_gyration
inline Float radius_of_gyration(const Particles& particles) {
  algebra::Vector3D centroid(0.0, 0.0, 0.0);
  std::vector<algebra::Vector3D> coordinates(particles.size());
  get_coordinates(particles, coordinates);
  for (unsigned int i = 0; i < particles.size(); i++) {
    centroid += coordinates[i];
  }
  centroid /= particles.size();
  Float rg = 0;
  for (unsigned int i = 0; i < particles.size(); i++) {
    rg += get_squared_distance(coordinates[i], centroid);
  }
  rg /= particles.size();
  return sqrt(rg);
}

IMPSAXS_END_NAMESPACE

#endif  /* IMPSAXS_UTILITY_H */
