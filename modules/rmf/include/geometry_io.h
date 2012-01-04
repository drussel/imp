/**
 *  \file IMP/rmf/geometry_io.h
 *  \brief Handle read/write of Model data from/to files.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPRMF_GEOMETRY_IO_H
#define IMPRMF_GEOMETRY_IO_H

#include "rmf_config.h"
#include <RMF/NodeHandle.h>
#include <RMF/FileHandle.h>
#include <IMP/display/Writer.h>
#include <IMP/display/display_macros.h>

IMPRMF_BEGIN_NAMESPACE

/** \name Geometry I/O

    The geometry I/O support currently handles geometry composed of
    - IMP::display::SegmentGeometry
    - IMP::display::CylinderGeometry
    - IMP::display::SphereGeometry
    - IMP::display::SurfaceMeshGeometry

    Other types can be supported when requested. Be aware, many
    more complex geometry types are automatically decomposed into
    the above types and so, more or less, supported.
    @{
 */
/** Add the geometry to the file */
IMPRMFEXPORT void add_geometry(RMF::FileHandle parent, display::Geometry *r);
/** Save the geometry for the specified frame. */
IMPRMFEXPORT void save_frame(RMF::FileHandle parent, int frame,
                              display::Geometry *r);


IMPRMFEXPORT display::Geometries create_geometries(RMF::FileConstHandle parent,
                                                   int frame);
/** @} */

class IMPRMFEXPORT RMFWriter: public display::Writer {
  RMF::FileHandle rh_;
  void on_set_frame();
  bool handle(display::SphereGeometry *g,
               display::Color color, std::string name);
  bool handle(display::CylinderGeometry *g,
               display::Color color, std::string name);
  bool handle(display::SegmentGeometry *g,
               display::Color color, std::string name);
  bool handle(display::SurfaceMeshGeometry *g,
               display::Color color, std::string name);
 public:
  RMFWriter(RMF::FileHandle rh);
  IMP_WRITER(RMFWriter);
};
IMPRMF_END_NAMESPACE

#endif /* IMPRMF_GEOMETRY_IO_H */
