/*
 *  \file Distribution.cpp \brief computes
 *
 *  distribution classes implementation
 *
 *  Copyright 2009 Sali Lab. All rights reserved.
 *
 */
#include <IMP/saxs/Distribution.h>
#include <IMP/saxs/utility.h>
#include <IMP/algebra/Vector3D.h>
#include <IMP/core/XYZ.h>

IMPSAXS_BEGIN_NAMESPACE

RadialDistributionFunction::
RadialDistributionFunction(FormFactorTable * ff_table, Float bin_size)
  : Distribution<Float>(bin_size), ff_table_(ff_table)
{
}

void RadialDistributionFunction::
calculate_distribution(const Particles& particles)
{
  IMP_LOG(VERBOSE, "start distribution calculation for "
          << particles.size() << " particles" << std::endl);

  // copy coordinates and form factors in advance, to avoid n^2 copy operations
  std::vector < algebra::Vector3D > coordinates;
  Floats form_factors;
  for (unsigned int i = 0; i < particles.size(); i++) {
    coordinates.push_back(core::XYZ::cast(particles[i]).
                          get_coordinates());
    form_factors.push_back(ff_table_->get_form_factor(particles[i]));
  }

  // iterate over pairs of atoms
  for (unsigned int i = 0; i < coordinates.size(); i++) {
    for (unsigned int j = i + 1; j < coordinates.size(); j++) {
      Float dist = distance(coordinates[i], coordinates[j]);
      add_to_distribution(dist, 2.0 * form_factors[i] * form_factors[j]);
    }
    // add autocorrelation part
    add_to_distribution(0.0, square(form_factors[i]));
  }
}

void RadialDistributionFunction::
calculate_distribution(const Particles& particles1,
                       const Particles& particles2)
{
  IMP_LOG(VERBOSE, "start distribution calculation for "
          << particles1.size() << " + " << particles2.size()
          << " particles" << std::endl);

  // copy coordinates and form factors in advance, to avoid n^2 copy operations
  std::vector < algebra::Vector3D > coordinates1, coordinates2;
  Floats form_factors1, form_factors2;
  for (unsigned int i = 0; i < particles1.size(); i++) {
    coordinates1.push_back(core::XYZ::cast(particles1[i]).
                           get_coordinates());
    form_factors1.push_back(ff_table_->get_form_factor(particles1[i]));
  }
  for (unsigned int i = 0; i < particles2.size(); i++) {
    coordinates2.push_back(core::XYZ::cast(particles2[i]).
                           get_coordinates());
    form_factors2.push_back(ff_table_->get_form_factor(particles2[i]));
  }

  // iterate over pairs of atoms
  for (unsigned int i = 0; i < coordinates1.size(); i++) {
    for (unsigned int j = 0; j < coordinates2.size(); j++) {
      Float dist = distance(coordinates1[i], coordinates2[j]);
      add_to_distribution(dist, 2 * form_factors1[i] * form_factors2[j]);
    }
  }
}

void RadialDistributionFunction::scale(Float c) {
  for (unsigned int i = 0; i < distribution_.size(); i++) {
    distribution_[i]*=c;
  }
}

void RadialDistributionFunction::add(const RadialDistributionFunction& other_rd)
{
  for (unsigned int i = 0; i < other_rd.distribution_.size(); i++) {
    add_to_distribution(other_rd.index2dist(i), other_rd.distribution_[i]);
  }
}

void RadialDistributionFunction::
show(std::ostream& out, std::string prefix) const
{
  for (unsigned int i = 0; i < distribution_.size(); i++) {
    out << prefix << " dist " << index2dist(i) << " " << distribution_[i]
    << std::endl;
  }
}


DeltaDistributionFunction::
DeltaDistributionFunction(FormFactorTable* ff_table,
                          const Particles& particles,
                          Float max_distance, Float bin_size)
  : Distribution<algebra::Vector3D>(bin_size), ff_table_(ff_table)
{
  // copy coordinates and form factors in advance, to avoid n^2 copy operations
  coordinates_.resize(particles.size());
  form_factors_.resize(particles.size());
  for (unsigned int i=0; i<particles.size(); i++) {
    coordinates_[i] = core::XYZ::cast(particles[i]).get_coordinates();
    form_factors_[i] = ff_table_->get_form_factor(particles[i]);
  }
  // compute max distance if not given
  max_distance_ = max_distance;
  if (max_distance_ <= 0.0) max_distance_ = compute_max_distance(particles);
}

void DeltaDistributionFunction::
calculate_derivative_distribution(Particle* particle)
{
  init();

  algebra::Vector3D particle_coordinate =
    core::XYZ::cast(particle).get_coordinates();
  Float particle_form_factor = ff_table_->get_form_factor(particle);

  for (unsigned int i=0; i<coordinates_.size(); i++) {
    Float dist = distance(coordinates_[i], particle_coordinate);
    algebra::Vector3D diff_vector = particle_coordinate - coordinates_[i];
    diff_vector *= particle_form_factor * form_factors_[i];
    add_to_distribution(dist, diff_vector);
  }
}


void DeltaDistributionFunction::
show(std::ostream & out, std::string prefix) const
{
  out << "DeltaDistributionFunction::show" << std::endl;
  for (unsigned int i = 0; i < distribution_.size(); i++) {
    out << prefix << " dist " << index2dist(i) << " value " << distribution_[i]
        << std::endl;
  }
}

IMPSAXS_END_NAMESPACE
