/**
 *  \file Key.cpp   \brief Internal workings of keys.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 *
 */

#include "IMP/Key.h"
#include "IMP/exception.h"
#include "IMP/base_types.h"
#include "IMP/internal/map.h"
#include "IMP/internal/AttributeTable.h"

IMP_BEGIN_INTERNAL_NAMESPACE

static double heuristic_value=238471628;

KeyData::KeyData(): heuristic_(heuristic_value){}

void KeyData::assert_is_initialized() const
{
  IMP_INTERNAL_CHECK(static_cast<int>(heuristic_)
                     == static_cast<int>(heuristic_value),
             "Uninitialized KeyData. Do not initialize Keys statically.");
}

void KeyData::show(std::ostream &out) const
{
  for (unsigned int i=0; i< rmap_.size(); ++i) {
    out << "\"" << rmap_[i] << "\" ";
  }
}

namespace {
  struct KeyTable: public Map<unsigned int, KeyData> {
    KeyTable() {
      unsigned int fk= FloatKey::get_ID();
      operator[](fk).add_key("x");
      operator[](fk).add_key("y");
      operator[](fk).add_key("z");
      operator[](fk).add_key("radius");
    }
  };
}

IMPEXPORT KeyData& get_key_data(unsigned int index) {
  static KeyTable key_data;
  return key_data[index];
}

IMP_END_INTERNAL_NAMESPACE
