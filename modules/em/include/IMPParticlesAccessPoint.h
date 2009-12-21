/**
 *  \file IMPParticlesAccessPoint.h
 *  \brief Provision of EMBED structural data using the IMP framework.
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 *
 */

#ifndef IMPEM_IMP_PARTICLES_ACCESS_POINT_H
#define IMPEM_IMP_PARTICLES_ACCESS_POINT_H

#include "config.h"
#include "ParticlesAccessPoint.h"
#include <IMP/core/XYZR.h>
#include <IMP/atom/Mass.h>
#include <IMP/Model.h>
#include <IMP/Particle.h>

#include <vector>
#include <map>

IMPEM_BEGIN_NAMESPACE

//!
class IMPEMEXPORT IMPParticlesAccessPoint : public ParticlesAccessPoint
{

public:
  IMPParticlesAccessPoint() {}
  //! Constructor
  /** \param[in] particles the particles
      \param[in] radius_key the attribute name of the radius
      \param[in] weight_key the attribute name of the weight
   */
  IMPParticlesAccessPoint(const Particles &particles,
                          FloatKey radius_key,
                          FloatKey weight_key);
  //! Add more particles to the access point.
  /**
  \note Notice that the index of ps[0] will be the current number of particles
        stored in the access point.
   */
  void add(const Particles& ps);
  void reselect(const Particles& ps);
  //! Get the number of particles
  int get_size() const {
    return particles_.size();
  }
  //! Get the value of the x coordinate of a particle
  /** \param[in] ind the position of a particle in the stored
                     vector of particles
      \return the value of the x coordinate of particle particles_[ind]
   */
  float get_x(unsigned int ind) const {
    return particles_[ind]->get_value(get_x_key());
  }
  //! Get the value of the y coordinate of a particle
  /** \param[in] ind the position of a particle in the stored
                     vector of particles
      \return the value of the y coordinate of particle particles_[ind]
   */
  float get_y(unsigned int ind) const {
    return particles_[ind]->get_value(get_y_key());
  }
  //! Get the value of the z coordinate of a particle
  /** \param[in] ind the position of a particle in the stored
                     vector of particles
      \return the value of the z coordinate of particle particles_[ind]
   */
  float get_z(unsigned int ind) const {
    return particles_[ind]->get_value(get_z_key());
  }

  //! Get the value of the coordinates of a particle
  /** \param[in] ind the position of a particle in the stored
                     vector of particles
   */
  algebra::Vector3D get_coordinates(unsigned int ind) const {
    return algebra::Vector3D(particles_[ind]->get_value(get_x_key()),
                             particles_[ind]->get_value(get_y_key()),
                             particles_[ind]->get_value(get_z_key()));
  }

  //! Get the value of the radius attribute of a particle
  /** \param[in] ind the position of a particle in the stored
                     vector of particles
      \return the value of the radius attribute of particle particles_[ind]
   */
  float get_r(unsigned int ind) const {
    return particles_[ind]->get_value(radius_key_);
  }
  //! Get the value of the weight attribute of a particle
  /** \param[in] ind the position of a particle in the stored
                     vector of particles
      \return the value of the weight attribute of particle particles_[ind]
   */
  float get_w(unsigned int ind) const {
    return particles_[ind]->get_value(weight_key_);
  }
  //! Set the value of the x coordinate of a particle
  /** \param[in] ind the position of a particle in the stored
                     vector of particles
      \param[in] xval the value of the x coordinate
   */
  void set_x(unsigned int ind, float xval) {
    particles_[ind]->set_value(get_x_key(), xval);
  }
  //! Set the value of the y coordinate of a particle
  /** \param[in] ind the position of a particle in the stored
                     vector of particles
      \param[in] yval the value of the y coordinate
   */
  void set_y(unsigned int ind, float yval) {
    particles_[ind]->set_value(get_y_key(), yval);
  }
  //! Set the value of the x coordinate of a particle
  /** \param[in] ind the position of a particle in the stored
                     vector of particles
      \param[in] zval the value of the z coordinate
   */
  void set_z(unsigned int ind, float zval) {
    particles_[ind]->set_value(get_z_key(), zval);
  }

  FloatKey get_x_key() const {
    return IMP::core::XYZ::get_xyz_keys()[0];
  }
  FloatKey get_y_key() const {
    return IMP::core::XYZ::get_xyz_keys()[1];
  }
  FloatKey get_z_key() const {
    return IMP::core::XYZ::get_xyz_keys()[2];
  }
  algebra::Vector3D get_centroid() const;

  void show(std::ostream &out=std::cout) const{
    for (unsigned i=0; i<particles_.size(); ++i) {
      out<<"("<<get_x(i)<<","<<get_y(i)<<","<<get_z(i)<<
           ") "<<get_r(i)<<" "<< get_w(i)<<std::endl;
    }
  }

private:
  Particles particles_;
  FloatKey radius_key_, weight_key_;
};

IMPEM_END_NAMESPACE

#endif  /* IMPEM_IMP_PARTICLES_ACCESS_POINT_H */
