/**
 *  \file RMF/Category.h
 *  \brief Handle read/write of Model data from/to files.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#include <RMF/decorator_utility.h>
#include <RMF/decorators.h>
#include <utility>
#include <limits>

namespace RMF {

  namespace {
    NodeConstHandles
    get_particles_by_resolution_internal(const ParticleConstFactory& f,
                                         NodeConstHandle h,
                                         double resolution,
                                         int frame) {
      NodeConstHandles children=h.get_children();
      NodeConstHandles ret;
      for (unsigned int i=0; i< children.size(); ++i) {
        NodeConstHandles cur
          = get_particles_by_resolution_internal(f, children[i],
                                                 resolution, frame);
        ret.insert(ret.end(), cur.begin(), cur.end());
      }

      if (f.get_is(h, frame)) {
        ParticleConst p= f.get(h, frame);
        if (p.get_radius() < resolution || ret.empty()) {
          return NodeConstHandles(1, h);
        }
      }
      return ret;
    }
  }

NodeConstHandles get_particles_by_resolution(NodeConstHandle h,
                                             double resolution,
                                             int frame) {
  ParticleConstFactory f(h.get_file());
  return get_particles_by_resolution_internal(f, h, resolution,
                                              frame);
}


} /* namespace RMF */