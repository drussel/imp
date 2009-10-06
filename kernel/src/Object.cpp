/**
 *  \file Object.cpp
 *  \brief A shared base class to help in debugging and things.
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 *
 */

#include "IMP/Object.h"
#include "IMP/RefCounted.h"

IMP_BEGIN_NAMESPACE

unsigned int RefCounted::live_objects_=0;

RefCounted::~ RefCounted() {
  IMP_assert(get_ref_count()== 0, "Deleting object which still has references");
    IMP_LOG(MEMORY, "Deleting ref counted object " << this << std::endl);
    --live_objects_;
  }

Object::Object()
{
#ifndef IMP_NDEBUG
  log_level_=DEFAULT;
  check_value_=111111111;
#endif
  IMP_LOG(MEMORY, "Creating object " << this << std::endl);
}

Object::~Object()
{
  IMP_OBJECT_LOG;
  IMP_assert(get_is_valid(), "Object " << this << " previously freed.");
#ifndef IMP_NDEBUG
  check_value_=666666666;
#endif
  IMP_LOG(MEMORY, "Destroying object " << this << std::endl);
}

IMP_END_NAMESPACE
