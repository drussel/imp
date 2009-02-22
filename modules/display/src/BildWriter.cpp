/**
 *  \file BildWriter.cpp
 *  \brief XXXX.
 *
 *  Copyright 2007-8 Sali Lab. All rights reserved.
 *
 */

#include "IMP/display/BildWriter.h"


IMPDISPLAY_BEGIN_NAMESPACE

BildWriter::BildWriter(){
}

BildWriter::~BildWriter(){
  if (get_stream_is_open()) {
    on_close();
  }
}

void BildWriter::show(std::ostream &out) const {
  out << "BildWriter" << std::endl;
}

void BildWriter::on_open(std::string) {
}

void BildWriter::on_close() {
}

void BildWriter::add_geometry(Geometry *g) {
  IMP_CHECK_OBJECT(g);
  get_stream() << ".color " << algebra::spaces_io(g->get_color())
                   << "\n";
  if (g->get_dimension() ==0) {
    algebra::Vector3D v=g->get_vertex(0);
    if (g->get_size() ==0) {
      get_stream() << ".dotat " << algebra::spaces_io(v)
                   << "\n";
    } else {
      std::cout << "Vertex is " <<  algebra::spaces_io(v)
                << std::endl;
      get_stream() << ".sphere " << algebra::spaces_io(v)
                   << " "
                   << g->get_size() << "\n";
    }
  } else if (g->get_dimension() ==1) {
    if (g->get_size() ==0) {
      get_stream() << ".segment "
                   << algebra::spaces_io(g->get_vertex(0)) << " "
                   << algebra::spaces_io(g->get_vertex(1))
                   << "\n";
    } else {
      get_stream() << ".cylinder "
                   << algebra::spaces_io(g->get_vertex(0)) << " "
                   << algebra::spaces_io(g->get_vertex(1)) << " "
                   << g->get_size() << "\n";
    }

  }
}


IMPDISPLAY_END_NAMESPACE
