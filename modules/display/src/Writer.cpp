/**
 *  \file Writer.cpp
 *  \brief XXXX.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 *
 */

#include "IMP/display/Writer.h"
#include <boost/algorithm/string/predicate.hpp>


IMPDISPLAY_BEGIN_NAMESPACE

Writer::Writer(TextOutput fn, std::string name): Object(name), out_(fn){
  on_open_called_=false;
  set_was_used(true);
}
Writer::Writer(std::string name): Object(name){
}

Writer::~Writer(){
}


void Writer::add_geometry(Geometry *g) {
  handle_geometry(g);
}


Writer *create_writer(std::string name, bool append) {
  for (std::map<std::string, internal::WriterFactory *>::iterator
         it= internal::get_writer_factory_table().begin();
       it != internal::get_writer_factory_table().end(); ++it) {
    if (boost::algorithm::ends_with(name, it->first)) {
      return it->second->create(name, append);
    }
  }
  IMP_THROW("No writer found for file " << name,
            ValueException);
}

IMPDISPLAY_END_NAMESPACE
