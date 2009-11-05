/**
 *  \file writers.cpp
 *  \brief XXXX.
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 *
 */

#include <IMP/display/Writer.h>
#include <IMP/display/internal/writers.h>

IMPDISPLAY_BEGIN_INTERNAL_NAMESPACE
std::map<std::string, WriterFactory *>& get_writer_factory_table() {
  static std::map<std::string, WriterFactory *> table;
  return table;
}

WriterFactory::WriterFactory(){}
WriterFactory::~WriterFactory(){}
IMPDISPLAY_END_INTERNAL_NAMESPACE
