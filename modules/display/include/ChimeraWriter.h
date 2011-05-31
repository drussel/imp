/**
 *  \file ChimeraWriter.h
 *  \brief XXXXXXXXXXXXXX
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 */

#ifndef IMPDISPLAY_CHIMERA_WRITER_H
#define IMPDISPLAY_CHIMERA_WRITER_H

#include "display_config.h"

#include <IMP/display/Writer.h>

IMPDISPLAY_BEGIN_NAMESPACE

//! Write geometry to a python file for Chimera to read
/** The writer writes a python file which can handle markers, edges
    and surfaces.  Since these are native chimera objects, they are
    handled a bit better than vrml ones.
 */
class IMPDISPLAYEXPORT ChimeraWriter: public TextWriter
{
  void cleanup(std::string name,
               bool need_ms, bool need_surf=false);
  bool handle(SphereGeometry *g,
               Color color, std::string name);
  bool handle(CylinderGeometry *g,
               Color color, std::string name);
  bool handle(PointGeometry *g,
               Color color, std::string name);
  bool handle(SegmentGeometry *g,
               Color color, std::string name);
  bool handle(PolygonGeometry *g,
               Color color, std::string name);
  bool handle(TriangleGeometry *g,
               Color color, std::string name);
  bool handle(EllipsoidGeometry *g,
               Color color, std::string name);
public:
  IMP_TEXT_WRITER(ChimeraWriter);
};


IMPDISPLAY_END_NAMESPACE

#endif  /* IMPDISPLAY_CHIMERA_WRITER_H */
