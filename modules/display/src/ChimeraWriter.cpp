/**
 *  \file ChimeraWriter.cpp
 *  \brief Write geometry as input to chimera via python.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 *
 */

#include "IMP/display/ChimeraWriter.h"
#include "IMP/display/internal/utility.h"

IMPDISPLAY_BEGIN_NAMESPACE

namespace {
  void write_marker(std::ostream &out,
                    const algebra::VectorD<3> &p,
                    Color c,
                    double r) {
    out << "mark=s.place_marker(("
                 << commas_io(p) << "), ("
                 << commas_io(c) << ")";
    if (r > 0) {
      out << ", " << r;
    } else {
      out << ", " << .1;
    }
    out << ")\n";
  }
}

void ChimeraWriter::cleanup(std::string name,
                            bool need_ms,
                            bool need_surf) {
  if (need_surf) {
    get_stream() << "if \""<< name << "\" not in surf_sets:\n";
    get_stream() << "  m=_surface.SurfaceModel()\n";
    get_stream() << "  m.name= \"" << name << "\"\n";
    get_stream() << "  surf_sets[\"" << name << "\"]=m\n";
    get_stream() << "m= surf_sets[\"" << name << "\"]\n";
  }
  if (need_ms) {
    get_stream() << "if \""<< name << "\" not in marker_sets:\n";
    get_stream() << "  s=new_marker_set('" << name
                 << "')\n";
    get_stream() << "  marker_sets[\"" << name << "\"]=s\n";
    get_stream() << "s= marker_sets[\"" << name << "\"]\n";
  }
}
bool ChimeraWriter::handle(SphereGeometry *g,
                            Color color, std::string name) {
  cleanup(name, true);
  write_marker(get_stream(), g->get_center(),
               color, g->get_radius());
  return true;
}
bool ChimeraWriter::handle(CylinderGeometry *g,
                            Color color, std::string name) {
  cleanup(name, true);
  write_marker(get_stream(), g->get_segment().get_point(0),
               color, g->get_radius());
  get_stream() << "ml=mark\n";
  write_marker(get_stream(), g->get_segment().get_point(1),
               color, g->get_radius());
  get_stream() << "Link(ml, mark, (" << commas_io(color) << ")";
  get_stream() << ", " << g->get_radius();
  get_stream() << ")\n";
  get_stream() << "ml=mark\n";
  return true;
}
bool ChimeraWriter::handle(PointGeometry *g,
                            Color color, std::string name) {
  cleanup(name, true);
  write_marker(get_stream(), *g,
               color, 0);
  return true;
}
bool ChimeraWriter::handle(SegmentGeometry *g,
                            Color color, std::string name) {
  cleanup(name, true);
  write_marker(get_stream(), g->get_point(0),
               color, 0);
  get_stream() << "ml=mark\n";
  write_marker(get_stream(), g->get_point(1),
               color, 0);
  get_stream() << "Link(ml, mark, (" << commas_io(color) << ")";
  get_stream() << ", .1)\n";
  get_stream() << "ml=mark\n";
  return true;
}
bool ChimeraWriter::handle(PolygonGeometry *g,
                            Color color, std::string name) {
  cleanup(name, false, true);
  std::pair<std::vector<algebra::Vector3Ds>, algebra::Vector3D> polys
    = internal::get_convex_polygons(*g);
  for (unsigned int i=0; i< polys.first.size(); ++i) {
    get_stream() << "v=[";
    for (unsigned int j=0; j< polys.first[i].size(); ++j) {
      get_stream() << "(" << commas_io(polys.first[i][j]) << "), ";
    }
    get_stream() << "]\n";
    get_stream() << "vi=[";
    for (unsigned int j=2; j< polys.first[i].size(); ++j) {
      get_stream() << "(";
      get_stream() << "0" << ", " << j-1 << ", " << j;
      get_stream() << "), ";
    }
    get_stream() << "]\n";
    get_stream() << "m.addPiece(v, vi, (" << commas_io(color)
                 << ", 1))\n";
  }
  return true;
}
bool ChimeraWriter::handle(TriangleGeometry *g,
                            Color color, std::string name) {
  cleanup(name, false, true);
  get_stream() << "v=[";
  for (unsigned int i=0; i< 3; ++i) {
    get_stream() << "(" << commas_io(g->at(i)) << "), ";
  }
  get_stream() << "]\n";
  get_stream() << "vi=[";
  for (unsigned int i=2; i< 3; ++i) {
    get_stream() << "(";
    get_stream() << "0" << ", " << i-1 << ", " << i;
    get_stream() << "), ";
  }
  get_stream() << "]\n";
  get_stream() << "m.addPiece(v, vi, (" << commas_io(color)
               << ", 1))\n";
  return true;
}
bool ChimeraWriter::handle(EllipsoidGeometry *g,
                            Color , std::string name) {
  cleanup(name, false, false);
  get_stream() << "try:\n";
  algebra::VectorD<4> q=g->get_rotation().get_quaternion();
  get_stream() << "  chimera.runCommand(\"shape ellipsoid radius "
               << g->get_radius(0) << "," << g->get_radius(1)
               << "," << g->get_radius(2) << " qrotation "
               << q[0] << "," << q[1] << "," << q[2] << "," << q[3]
               << "\")\n";
  get_stream() << "except:\n";
  get_stream() << "  print \"Need Chimera > 1.4 to draw ellipsoids\"\n";
  return true;
}

void ChimeraWriter::handle_open() {
  get_stream() << "import _surface\n";
  get_stream() << "import chimera\n";
  get_stream() << "try:\n";
  get_stream() << "  import chimera.runCommand\n";
  get_stream() << "except:\n";
  get_stream() << "  pass\n";
  get_stream() << "from VolumePath import markerset as ms\n";
  get_stream() << "try:\n";
  get_stream() << "  from VolumePath import Marker_Set, Link\n";
  get_stream() << "  new_marker_set=Marker_Set\n";
  get_stream() << "except:\n";
  get_stream() << "  from VolumePath import volume_path_dialog\n";
  get_stream() << "  d= volume_path_dialog(True)\n";
  get_stream() << "  new_marker_set= d.new_marker_set\n";
  get_stream() << "marker_sets={}\n";
  get_stream() << "surf_sets={}\n";
}
void ChimeraWriter::handle_close() {
  get_stream() << "for k in surf_sets.keys():\n";
  get_stream() << "  chimera.openModels.add([surf_sets[k]])\n";
}

IMP_REGISTER_WRITER(ChimeraWriter, ".py")

IMPDISPLAY_END_NAMESPACE
