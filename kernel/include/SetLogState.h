/**
 *  \file SetLogState.h     \brief Logging and error reporting support.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMP_SET_LOG_STATE_H
#define IMP_SET_LOG_STATE_H

#include "log.h"
#include "base_types.h"
#include <vector>

IMP_BEGIN_NAMESPACE
class Object;

//! A class to change and restore log state
/**
   To use, create an instance of this class with the log level you
   want. When it goes out of scope, it will restore the old level.
   \ingroup logging
 */
class IMPEXPORT SetLogState
{
  LogLevel level_;
  Object* obj_;
  void do_set(Object *o, LogLevel l);
  void do_reset();
  void do_show(std::ostream &out) const;
public:
  IMP_RAII(SetLogState, (Object *o, LogLevel l),
           {level_= DEFAULT; obj_=NULL;},
           {
             do_set(o, l);
           },
           {
             do_reset();
           }, do_show(out););

  //! Construct it with the desired level and target
  SetLogState(LogLevel l){
    obj_=NULL;
    level_= DEFAULT;
    set(l);
  }
  void set(LogLevel l) {
    reset();
    if (l != DEFAULT) {
      level_= get_log_level();
      set_log_level(l);
    } else {
      level_=DEFAULT;
    }
  }
};


IMP_VALUES(SetLogState, SetLogStates);

IMP_END_NAMESPACE

#endif  /* IMP_SET_LOG_STATE_H */
