/**
 *  \file CMMWriter.cpp
 *  \brief XXXX.
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 *
 */

#include "IMP/display/CMMWriter.h"


IMPDISPLAY_BEGIN_NAMESPACE

CMMWriter::CMMWriter(std::string name): Writer(name){
}

void CMMWriter::show(std::ostream &out) const {
  out << "CMMWriter" << std::endl;
}

void CMMWriter::on_open() {
  get_stream() << "<marker_set name=\"" <<get_file_name() << "\">"<<std::endl;
  marker_index_=0;
}

void CMMWriter::on_close() {
  get_stream() << "</marker_set>" << std::endl;
}

void CMMWriter::add_geometry(Geometry *g) {
  IMP_CHECK_OBJECT(g);
  if (g->get_dimension() != 0) return;
  std::string name=g->get_name();
  Float radius = g->get_size();
  g->set_was_owned(true);

  for (unsigned int i=0; i< g->get_number_of_vertices(); ++i) {
    algebra::Vector3D v= g->get_vertex(i);
    get_stream() << "<marker id=\"" << ++marker_index_ << "\""
                 << " x=\"" << v[0] << "\""
                 << " y=\"" << v[1] << "\""
                 << " z=\"" << v[2] << "\""
                 << " radius=\"" << radius << "\""
                 << " r=\"" << g->get_color().get_red() << "\""
                 << " g=\"" << g->get_color().get_green() << "\""
                 << " b=\"" << g->get_color().get_blue() <<  "\""
                 << " note=\"" << name <<  "\"/>" << std::endl;
  }
}

IMP_REGISTER_WRITER(CMMWriter, ".cmm")

IMPDISPLAY_END_NAMESPACE
