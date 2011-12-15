/**
 *  \file Log.cpp   \brief Logging and error reporting support.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 *
 */

#include <IMP/base/SetLogState.h>
#include <IMP/base/Object.h>

IMPBASE_BEGIN_NAMESPACE

void SetLogState::do_set(Object*o, LogLevel l) {
  if (l != DEFAULT) {
    obj_=o;
    level_= obj_->get_log_level();
    obj_->set_log_level(l);
  } else {
    obj_=nullptr;
    level_=DEFAULT;
  }
}

void SetLogState::do_reset() {
  if (level_ != DEFAULT) {
    if (obj_) {
      obj_->set_log_level(level_);
    } else {
      set_log_level(level_);
    }
    obj_=nullptr;
    level_=DEFAULT;
  }
}

void SetLogState::do_show(std::ostream &out) const{
 out << "Setting from " << level_
                      << " to "
     << (obj_? obj_->get_log_level() : IMP::base::get_log_level())
     << std::endl;
}

IMPBASE_END_NAMESPACE
