/**
 *  \file Configuration.cpp
 *  \brief Storage of a model, its restraints,
 *                         constraints and particles.
 *
 *  Copyright 2007-2013 IMP Inventors. All rights reserved.
 *
 */

#include "IMP/Configuration.h"
#include "IMP/internal/utility.h"
#include "IMP/dependency_graph.h"
#include "IMP/compatibility/set.h"

IMP_BEGIN_NAMESPACE



Configuration::Configuration(Model *m, std::string name): Object(name),
  model_(m){
  floats_= *m;
  strings_=*m;
  ints_=*m;
  objects_=*m;
  ints_lists_=*m;
  objects_lists_=*m;
  particles_=*m;
  particles_lists_=*m;
}




void Configuration::load_configuration() const {
  IMP_OBJECT_LOG;
  set_was_used(true);
  // workaround for weird mac os and boost 1.48 bug
  Configuration *ncthis= const_cast<Configuration*>(this);
  static_cast<internal::FloatAttributeTable&>(*model_)= ncthis->floats_;
  static_cast<internal::StringAttributeTable&>(*model_)= ncthis->strings_;
  static_cast<internal::IntAttributeTable&>(*model_)= ncthis->objects_;
  static_cast<internal::ObjectAttributeTable&>(*model_)= ncthis->ints_lists_;
  static_cast<internal::WeakObjectAttributeTable&>(*model_)
      = ncthis->wints_lists_;
  static_cast<internal::IntsAttributeTable&>(*model_)= ncthis->objects_lists_;
  static_cast<internal::ObjectsAttributeTable&>(*model_)= ncthis->particles_;
  static_cast<internal::ParticleAttributeTable&>(*model_)
      = ncthis->particles_lists_;
  static_cast<internal::ParticlesAttributeTable&>(*model_)= ncthis->ints_;
}

void Configuration::swap_configuration() {
  IMP_OBJECT_LOG;
  set_was_used(true);
  using std::swap;
  swap(static_cast<internal::FloatAttributeTable&>(*model_), floats_);
  swap(static_cast<internal::StringAttributeTable&>(*model_), strings_);
  swap(static_cast<internal::IntAttributeTable&>(*model_), objects_);
  swap(static_cast<internal::ObjectAttributeTable&>(*model_), ints_lists_);
  swap(static_cast<internal::WeakObjectAttributeTable&>(*model_), wints_lists_);
  swap(static_cast<internal::IntsAttributeTable&>(*model_), objects_lists_);
  swap(static_cast<internal::ObjectsAttributeTable&>(*model_), particles_);
  swap(static_cast<internal::ParticleAttributeTable&>(*model_),
       particles_lists_);
  swap(static_cast<internal::ParticlesAttributeTable&>(*model_), ints_);
}


void Configuration::do_show(std::ostream &out) const {
  out <<  "configuration" << std::endl;
}

IMP_END_NAMESPACE
