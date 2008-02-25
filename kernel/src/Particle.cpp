/**
 *  \file Particle.cpp   \brief Classes to handle individual model particles.
 *
 *  Copyright 2007-8 Sali Lab. All rights reserved.
 *
 */

#include "IMP/Particle.h"
#include "IMP/log.h"
#include "IMP/Model.h"


namespace IMP
{

Particle::Particle()
{
  is_active_ = true;
}

Particle::~Particle()
{
}


void Particle::set_model(Model *md, ParticleIndex pi)
{
  model_ = md;
  pi_ = pi;
}

void Particle::set_is_active(const bool is_active)
{
  is_active_ = is_active;
}


void Particle::add_attribute(FloatKey name, const Float value,
                             bool is_optimized)
{
  IMP_assert(model_ ,
             "Particle must be added to Model before an attributes are added");
  float_indexes_.insert(name, model_->get_model_data()->add_float(value));

  model_->get_model_data()->set_is_optimized(float_indexes_.get_value(name),
                                             is_optimized);
}


void Particle::add_attribute(IntKey name, const Int value)
{
  IMP_assert(model_,
             "Particle must be added to Model before an attributes are added");
  int_indexes_.insert(name, model_->get_model_data()->add_int(value));
}


void Particle::add_attribute(StringKey name, const String value)
{
  IMP_assert(model_,
             "Particle must be added to Model before an attributes are added");
  string_indexes_.insert(name, model_->get_model_data()->add_string(value));
}



void Particle::show(std::ostream& out) const
{
  char* inset = "  ";
  out << std::endl;
  out << "--" << get_index() << "--" << std::endl;
  if (is_active_) {
    out << inset << inset << "active";
  } else {
    out << inset << inset << "dead";
  }
  out << std::endl;

  if (get_model() != NULL) {
    out << inset << inset << "float attributes:" << std::endl;
    float_indexes_.show(out, "    ", get_model()->get_model_data());

    out << inset << inset << "int attributes:" << std::endl;
    int_indexes_.show(out, "    ", get_model()->get_model_data());

    out << inset << inset << "string attributes:" << std::endl;
    string_indexes_.show(out, "    ", get_model()->get_model_data());
  }
}


}  // namespace IMP
