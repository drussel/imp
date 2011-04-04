/**
 * Copyright 2007-2011 IMP Inventors. All rights reserved.
 */
#include <IMP/hdf5/atom_io.h>
#include <IMP/hdf5/RootHandle.h>
#include <IMP/hdf5/particle_io.h>
#include <IMP/display/geometry.h>
#include <IMP/display/particle_geometry.h>
#include <IMP/display/Writer.h>
#include <IMP/hdf5/geometry_io.h>
#include <IMP/hdf5/restraint_io.h>
#include <boost/format.hpp>
#include <boost/program_options.hpp>
namespace po = boost::program_options;

std::string input, output;
po::options_description desc("Usage: input_hdf5 output_graphics");
double restraint_max=-1;

int frame=0;
void print_help() {
  std::cerr << desc << std::endl;
}

IMP::core::XYZRs get_xyzr_particles(IMP::hdf5::NodeHandle nh,
                                    IMP::hdf5::NodeIDKeys &niks,
                                    int frame) {
  IMP::ParticlesTemp ps= IMP::hdf5::get_restraint_particles(nh, niks, frame);
  IMP::core::XYZRs ret;
  for (unsigned int i=0; i< ps.size(); ++i) {
    if (IMP::core::XYZR::particle_is_instance(ps[i])) {
      ret.push_back(IMP::core::XYZR(ps[i]));
    }
  }
  return ret;
}

void set_color(IMP::hdf5::NodeHandle nh,
               int frame, IMP::display::Geometry *g) {
  if (restraint_max==-1) {
    return;
  } else {
    double score= IMP::hdf5::get_restraint_score(nh, frame);
    if (score <= -std::numeric_limits<double>::max()) return;
    double nscore= score/restraint_max;
    if (nscore<0) nscore=0;
    if (nscore>1) nscore=1;
    g->set_color( IMP::display::get_jet_color(nscore));
  }
}

IMP::display::Geometry *create_restraint_geometry(IMP::hdf5::NodeHandle nh,
                                                  IMP::hdf5::NodeIDKeys &niks,
                                                  IMP::hdf5::FloatKey &rsk,
                                                  int frame) {
  double score=IMP::hdf5::get_restraint_score(nh, rsk, frame);
  if (score < -std::numeric_limits<double>::max()) return NULL;
  IMP::hdf5::NodeHandles children=nh.get_children();
  IMP::display::Geometries gs;
  for (unsigned int i=0; i< children.size(); ++i) {
    IMP::display::Geometry* g
      = create_restraint_geometry(children[i], niks, rsk, frame);
    if (g) {
      IMP::Pointer<IMP::display::Geometry> gp(g);
      gs.push_back(g);
    }
  }
  IMP::core::XYZRs ds= get_xyzr_particles(nh, niks, frame);
  if (ds.size()==2) {
    IMP::algebra::Segment3D s(ds[0].get_coordinates(), ds[1].get_coordinates());
    gs.push_back(new IMP::display::SegmentGeometry(s));
    gs.back()->set_name(nh.get_name());
  } else {
    for (unsigned int i=0; i< ds.size(); ++i) {
      gs.push_back(new IMP::display::SphereGeometry(ds[i].get_sphere()));
      gs.back()->set_name(nh.get_name());
    }
  }
  if (gs.empty()) {
    return NULL;
  } else if (gs.size()==1) {
    set_color(nh, frame, gs[0]);
    IMP::Pointer<IMP::display::Geometry> ret(gs[0]);
    gs.clear();
    return ret.release();
  } else {
    IMP_NEW(IMP::display::CompoundGeometry, ret, (gs, nh.get_name()));
    set_color(nh, frame, ret);
    return ret.release();
  }
}

void add_restraints(IMP::hdf5::RootHandle rh,
                    int frame,
                    IMP::display::Writer *w) {
  IMP::hdf5::NodeIDKeys niks;
  IMP::hdf5::FloatKey rsk;
  IMP::hdf5::NodeHandles children = rh.get_children();
  for (unsigned int i=0; i< children.size(); ++i) {
    IMP::display::Geometry* g= create_restraint_geometry(children[i],
                                                         niks, rsk,
                                                         frame);
    if (g) {
      IMP::Pointer<IMP::display::Geometry> gp(g);
      g->set_color(IMP::display::get_display_color(i));
      w->add_geometry(g);
    }
  }
}

int main(int argc, char **argv) {
  desc.add_options()
    ("help,h", "Translate an hdf5 file to graphics.")
    ("recolor,c", "Recolor the hierarchies using the display colors.")
    ("frame,f", po::value< int >(&frame),
     "Frame to use. Do '-#' for every #th frame (eg -1 is every frame).")
    ("score,s", po::value< double >(&restraint_max),
     "The upper bound for the restraints scores to color the "\
     "restraints by score.")
    ("input-file,i", po::value< std::string >(&input),
     "input hdf5 file")
    ("output-file,o", po::value< std::string >(&output),
     "output graphics file");
  po::positional_options_description p;
  p.add("input-file", 1);
  p.add("output-file", 1);
  po::variables_map vm;
  po::store(
      po::command_line_parser(argc,argv).options(desc).positional(p).run(), vm);
  po::notify(vm);
  if (vm.count("help") || input.empty() || output.empty()) {
    print_help();
    return 1;
  }
  IMP::hdf5::RootHandle rh(input, false);
  IMP_NEW(IMP::Model, m, ());
  IMP::atom::Hierarchies hs= IMP::hdf5::read_all_hierarchies(rh, m);
  IMP::ParticlesTemp ps= IMP::hdf5::read_all_particles(rh, m);
  int minframe, maxframe;
  if (frame>=0) {
    minframe=frame;
    maxframe=minframe+1;
  } else {
    minframe=0;
    IMP::hdf5::FloatKey xk
      =rh.get_key<IMP::hdf5::FloatTraits>(IMP::hdf5::Physics, "cartesian x");
    std::cout << xk << std::endl;
    maxframe= rh.get_number_of_frames(xk)+1;
  }
  int step=1;
  if (frame<0) step=std::abs(frame);
  std::cout << "Reading frames [" << minframe << ", "
            << maxframe << ": " << step << ")" <<std::endl;
  for (int cur_frame=minframe; cur_frame < maxframe; cur_frame+=step) {
    if (cur_frame%10==0) {
      std::cout << cur_frame << " ";
    }
    std::string name=output;
    bool append=false;
    if (frame<0 && name.find("%1%")==std::string::npos) {
      append=cur_frame >minframe;
    } else if (frame<0) {
      std::ostringstream oss;
      oss << boost::format(output)%cur_frame;
      name=oss.str();
    }
    IMP::Pointer<IMP::display::Writer> w
      = IMP::display::create_writer(name, append);
    for (unsigned int i=0; i< hs.size(); ++i) {
      IMP::hdf5::load_configuration(rh, hs[i], cur_frame);
      IMP_NEW(IMP::display::HierarchyGeometry, g, (hs[i]));
      if (vm.count("recolor")) {
        g->set_color(IMP::display::get_display_color(i));
      }
      w->add_geometry(g);
    }
    for (unsigned int i=0; i< ps.size(); ++i) {
      /*if (frame!= 0) {
        IMP::hdf5::load_configuration(rh, hs[i], frame);
        }*/
      if (IMP::core::XYZR::particle_is_instance(ps[i])) {
        IMP::core::XYZR d(ps[i]);
        IMP_NEW(IMP::display::XYZRGeometry, g, (ps[i]));
        if (vm.count("recolor")) {
          g->set_color(IMP::display::get_display_color(i));
        }
        w->add_geometry(g);
      }
    }
    IMP::display::Geometries gs=
      IMP::hdf5::read_all_geometries(rh, cur_frame);
    for (unsigned int i=0; i< gs.size(); ++i) {
      w->add_geometry(gs[i]);
    }
    add_restraints(rh, cur_frame, w);
  }
  return 0;
}
