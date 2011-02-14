/**
 *  \file CMMWriter.cpp
 *  \brief XXXX.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 *
 */

#include "IMP/display/CMMWriter.h"


IMPDISPLAY_BEGIN_NAMESPACE

void CMMWriter::handle_open() {
  get_stream() << "<marker_set name=\"" <<get_name() << "\">"<<std::endl;
  marker_index_=0;
}

void CMMWriter::handle_close() {
  get_stream() << "</marker_set>" << std::endl;
}

bool CMMWriter::handle(SphereGeometry *g, Color color,
                        std::string name) {
  get_stream() << "<marker id=\"" << ++marker_index_ << "\""
               << " x=\"" << g->get_geometry().get_center()[0] << "\""
               << " y=\"" << g->get_geometry().get_center()[1] << "\""
               << " z=\"" << g->get_geometry().get_center()[2] << "\""
               << " radius=\"" << g->get_geometry().get_radius() << "\""
               << " r=\"" << color.get_red() << "\""
               << " g=\"" << color.get_green() << "\""
               << " b=\"" << color.get_blue() <<  "\""
               << " note=\"" << name <<  "\"/>" << std::endl;
  return true;
}

bool CMMWriter::handle(PointGeometry *g, Color color,
                        std::string name) {
  get_stream() << "<marker id=\"" << ++marker_index_ << "\""
               << " x=\"" << g->get_geometry().operator[](0) << "\""
               << " y=\"" << g->get_geometry().operator[](1) << "\""
               << " z=\"" << g->get_geometry().operator[](2) << "\""
               << " radius=\"" << 1 << "\""
               << " r=\"" << color.get_red() << "\""
               << " g=\"" << color.get_green() << "\""
               << " b=\"" << color.get_blue() <<  "\""
               << " note=\"" << name <<  "\"/>" << std::endl;
  return true;
}

IMP_REGISTER_WRITER(CMMWriter, ".cmm")

IMPDISPLAY_END_NAMESPACE
