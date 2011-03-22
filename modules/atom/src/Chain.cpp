/**
 *  \file Chain.cpp
 *  \brief XXXX.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 *
 */

#include "IMP/atom/Chain.h"


IMPATOM_BEGIN_NAMESPACE


IntKey Chain::get_id_key() {
  static IntKey k("chain");
  return k;
}

void Chain::show(std::ostream &out) const {
  out << "Chain " << get_id();
}


IMPATOMEXPORT
Chain get_chain(Hierarchy h) {
do {
    if (Chain::particle_is_instance(h)) {
      return Chain(h);
    }
  } while ((h=h.get_parent()));
 return Chain();
}

IMPATOM_END_NAMESPACE
