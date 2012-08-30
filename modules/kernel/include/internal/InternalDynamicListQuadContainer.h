/**
 *  \file ListQuadContainer.h
 *  \brief Store a list of ParticleQuadsTemp
 *
 *  WARNING This file was generated from InternalDynamicListNAMEContainer.hpp
 *  in tools/maintenance/container_templates/kernel/internal
 *  by tools/maintenance/make-container.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */

#ifndef IMPKERNEL_INTERNAL_INTERNAL_DYNAMIC_LIST_QUAD_CONTAINER_H
#define IMPKERNEL_INTERNAL_INTERNAL_DYNAMIC_LIST_QUAD_CONTAINER_H

#include "../kernel_config.h"
#include "container_helpers.h"
#include "ListLikeQuadContainer.h"
#include <IMP/base/Pointer.h>

IMP_BEGIN_INTERNAL_NAMESPACE


class IMPEXPORT InternalDynamicListQuadContainer:
  public ListLikeQuadContainer
{
  typedef ListLikeQuadContainer P;
  // use this to define the set of all possible particles when it is dynamic
  base::Pointer<Container> scope_;
  bool check_list(const ParticleIndexes& cp) const;
 public:
  InternalDynamicListQuadContainer(Container *m, std::string name);
  InternalDynamicListQuadContainer(Container *m, const char *name);
  void add_particle_quad(const ParticleQuad& vt) {
    IMP_USAGE_CHECK(IMP::internal::is_valid(vt),
                    "Passed Quad cannot be nullptr (or None)");

    add_to_list(IMP::internal::get_index(vt));
    IMP_USAGE_CHECK(check_list(IMP::internal::flatten
                               (IMP::internal::get_index(vt))),
                    "Invalid entries added to list " << vt);
  }
  void add_particle_quad(const ParticleIndexQuad& vt) {
    add_to_list(vt);
    IMP_USAGE_CHECK(check_list(IMP::internal::flatten(vt)),
                    "Invalid entries added to list " << vt);
  }
  void add_particle_quads(const ParticleQuadsTemp &c) {
    if (c.empty()) return;
    ParticleIndexQuads cp= IMP::internal::get_index(c);
    add_to_list(cp);
    IMP_USAGE_CHECK(check_list(IMP::internal::flatten
                               (cp)),
                    "Invalid entries added to list " << cp);
  }
  void remove_particle_quads(const ParticleQuadsTemp &c);
  void set_particle_quads(ParticleQuadsTemp c) {
    ParticleIndexQuads cp= IMP::internal::get_index(c);
    update_list(cp);
    IMP_USAGE_CHECK(check_list(IMP::internal::flatten
                               (cp)),
                    "Invalid entries added to list " << c);
  }
  void set_particle_quads(ParticleIndexQuads cp) {
    update_list(cp);
    IMP_USAGE_CHECK(check_list(IMP::internal::flatten(cp)),
                    "Invalid entries added to list " << cp);
  }
  void clear_particle_quads() {
    ParticleIndexQuads t;
    update_list(t);
  }
  IMP_LISTLIKE_QUAD_CONTAINER(InternalDynamicListQuadContainer);
};

IMP_END_INTERNAL_NAMESPACE

#endif  /* IMPKERNEL_INTERNAL_INTERNAL_DYNAMIC_LIST_QUAD_CONTAINER_H */
