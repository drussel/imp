/**
 *  \file BildWriter.h
 *  \brief XXXXXXXXXXXXXX
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 */

#ifndef IMPDISPLAY_BILD_WRITER_H
#define IMPDISPLAY_BILD_WRITER_H

#include "config.h"
#include "macros.h"

#include <IMP/PairContainer.h>
#include <IMP/SingletonContainer.h>
#include <IMP/display/Writer.h>

IMPDISPLAY_BEGIN_NAMESPACE

//! Write a Bild file with the geometry
/** The bild file format is a simple format for displaying geometry in Chimera.
    The bild writer supports points, spheres, cyliners, and segments.

    You are probably better off using the IMP::display::ChimeraWriter unless
    you want a human-readable file. ChimeraWriter supports better
    interactivity and more types of geometry.
 */
class IMPDISPLAYEXPORT BildWriter: public Writer
{
public:
  //! write to a file with the given name
  BildWriter(std::string file_name=std::string());

  IMP_WRITER(BildWriter, get_module_version_info())

  IMP_WRITER_ADD_GEOMETRY
};



IMPDISPLAY_END_NAMESPACE

#endif  /* IMPDISPLAY_BILD_WRITER_H */
