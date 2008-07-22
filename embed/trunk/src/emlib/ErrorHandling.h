#ifndef _ERRORHANDLING_H
#define _ERRORHANDLING_H

#include "EM_config.h"
#include <string>
#include <iostream>
#include <sstream>
#include <cstring>

#define LEN_MSG 1024 // Length allowed for text in the error messages

//!  Base EMBED Exception
class EMDLLEXPORT EMBED_Exception
{
public:
  const char *what() const throw() {
    return str_? str_->message_: NULL;
  }

  EMBED_Exception(const char *message) {
    str_= new (std::nothrow) refstring();
    if (str_ != NULL) {
     str_->ct_=0;
     std::strncpy(str_->message_, message, LEN_MSG-1);
     str_->message_[LEN_MSG-1]='\0';
    }
  }

  ~EMBED_Exception() throw() {
    destroy();
  }

  EMBED_Exception(const EMBED_Exception &o) {copy(o);}
  EMBED_Exception &operator=(const EMBED_Exception &o) {
    destroy();
    copy(o);
    return *this;
  }
 protected:
  void destroy() {
    if (str_ != NULL) {
      --str_->ct_;
      if (str_->ct_==0) delete str_;
    }
  }
  void copy(const EMBED_Exception &o) {
    str_=o.str_;
    if (str_!= NULL) ++str_->ct_;
  }

 private:
  struct refstring {
    char message_[LEN_MSG];
    int ct_;
  };
  refstring *str_;
};

//! Exception to throw when a variable has a wrong value
class EMDLLEXPORT EMBED_WrongValue: public EMBED_Exception {
// Using EMDLLEXPORT is necessary to make the object public when compiled in a
// dynamic share object in SWIG
public:
  EMBED_WrongValue(const char *msg): EMBED_Exception(msg){}
};

 //! Exception to throw when there are I/O problems
class EMDLLEXPORT EMBED_IOException: public EMBED_Exception {
// Using EMDLLEXPORT is necessary to make the object public when compiled in a
// dynamic share object in SWIG
 public:
  virtual ~EMBED_IOException() throw() {};
  EMBED_IOException(const char *msg): EMBED_Exception(msg){}
};
//! Exception to throw when there is an error in the logic of the program
class EMDLLEXPORT EMBED_LogicError: public EMBED_Exception {
// Using EMDLLEXPORT is necessary to make the object public when compiled in a
// dynamic share object in SWIG
public:
  EMBED_LogicError(const char *msg): EMBED_Exception(msg){}
};
//! Exception to throw when there are values out of range
class EMDLLEXPORT EMBED_OutOfRange: public EMBED_Exception {
// Using EMDLLEXPORT is necessary to make the object public when compiled in a
// dynamic share object in SWIG
public:
  EMBED_OutOfRange(const char *msg): EMBED_Exception(msg){}
};

#endif //_ERRORHANDLING_H
