/**
 *  \file Restraint.cpp   \brief Abstract base class for all restraints.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#include "IMP/container_base.h"
#include "IMP/internal/utility.h"
#include "IMP/Particle.h"
#include "IMP/Model.h"
#include "IMP/internal/graph_utility.h"
#include "IMP/dependency_graph.h"

IMP_BEGIN_NAMESPACE

Container::Container(Model *m, std::string name):
  Constraint(m, name) {
  IMP_USAGE_CHECK(m, "Must pass model to container constructor.");
  changed_=false;
#if IMP_BUILD < IMP_FAST
  writeable_=true;
  readable_=true;
#endif
}

void Container::set_is_changed(bool tr) {
  validate_writable();
  changed_=tr;
}


bool Container::get_is_changed() const {
  validate_readable();
  return changed_;
}

void Container::do_after_evaluate(DerivativeAccumulator *) {
  changed_=false;
}

void Container::validate_readable() const {
  #if IMP_BUILD < IMP_FAST
  if (!readable_) {
    throw internal::InputOutputException(get_name(),
                                         internal::InputOutputException::GET);
  }
  #endif
}
void Container::validate_writable() const {
  #if IMP_BUILD < IMP_FAST
  if (!writeable_) {
    throw internal::InputOutputException(get_name(),
                                         internal::InputOutputException::GET);
  }
  #endif
}
void Container::set_is_readable(bool tf) {
  #if IMP_BUILD < IMP_FAST
    readable_=tf;
  #else
    IMP_UNUSED(tf);
  #endif
}
void Container::set_is_writable(bool tf) {
  #if IMP_BUILD < IMP_FAST
    writeable_=tf;
  #else
    IMP_UNUSED(tf);
  #endif
}

IMP_END_NAMESPACE
