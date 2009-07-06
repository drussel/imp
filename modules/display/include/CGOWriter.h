/**
 *  \file CGOWriter.h
 *  \brief XXXXXXXXXXXXXX
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 */

#ifndef IMPDISPLAY_CGO_WRITER_H
#define IMPDISPLAY_CGO_WRITER_H

#include "config.h"
#include "macros.h"

#include "internal/version_info.h"
#include <IMP/PairContainer.h>
#include <IMP/SingletonContainer.h>
#include <IMP/display/Writer.h>

IMPDISPLAY_BEGIN_NAMESPACE

//! Write a CGO file with the geometry
/** The cgo file format is a simple format for displaying geometry in Pymol.
    The cgo writer supports points, spheres, cyliners, and segments.
    The file name should end in ".py".
 */
class IMPDISPLAYEXPORT CGOWriter: public Writer
{
  friend class CGOAnimationWriter;
  std::string name_;
  unsigned int count_;

  static void write_geometry(Geometry *g, std::ostream &out);
public:
  //! write to a file using the name to  name the files
  CGOWriter(std::string file_name=std::string());

  //! Set the name used to identify the model in pymol
  void set_name(std::string name) {
    name_=name;
  }

  IMP_WRITER(CGOWriter, internal::version_info)

  IMP_WRITER_ADD_GEOMETRY
};



IMPDISPLAY_END_NAMESPACE

#endif  /* IMPDISPLAY_CGO_WRITER_H */
