/**
 *  \file atom/hierarchy_tools.cpp
 *  \brief A decorator for a point particle that has an electrostatic charge.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 *
 */

#include "IMP/atom/hierarchy_tools.h"
#include <IMP/SingletonContainer.h>
#include <IMP/algebra/vector_generators.h>
#include <IMP/atom/Atom.h>
#include <IMP/atom/Chain.h>
#include <IMP/atom/Domain.h>
#include <IMP/atom/Fragment.h>
#include <IMP/atom/Mass.h>
#include <IMP/atom/Residue.h>
#include <IMP/atom/bond_decorators.h>
#include <IMP/atom/estimates.h>
#include <IMP/core/ConnectivityRestraint.h>
#include <IMP/core/DistancePairScore.h>
#include <IMP/core/Harmonic.h>
#include <algorithm>

IMPATOM_BEGIN_NAMESPACE

namespace {
  std::pair<int, double> compute_n(double V, double r, double f) {
    double n=.5*(3*V+2*PI*cube(r*f)-6*PI*cube(r)*square(f))
      /((-3*square(f)+cube(f)+2)*cube(r)*PI);
    int in= static_cast<int>(std::ceil(n));
    double rr= std::pow(V/(.666*(2*in-3*square(f)*n+cube(f)*n
                                 +3*square(f)-cube(f))*PI), .333333);
    return std::make_pair(in, rr);
  }
}

Hierarchy create_protein(Model *m,
                         double resolution,
                         int number_of_residues,
                         int first_residue_index,
                         double volume,
                         double /*spring strength*/) {
  double mass= atom::get_mass_from_number_of_residues(number_of_residues)/1000;
  if (volume < 0) {
    volume= atom::get_volume_from_mass(mass*1000);
  }
  // assume a 20% overlap in the beads to make the protein not too bumpy
  double overlap_frac=.2;
  std::pair<int, double> nr= compute_n(volume, resolution, overlap_frac);
  Hierarchy pd=Hierarchy::setup_particle(new Particle(m));
  Particles ps;
  for (int i=0; i< nr.first; ++i) {
    Particle *pc= new Particle(m);
    atom::Fragment pcd
      =atom::Fragment::setup_particle(pc);
    pd.add_child(pcd);
    core::XYZR xyzd=core::XYZR::setup_particle(pc);
    xyzd.set_radius(nr.second);
    xyzd.set_coordinates_are_optimized(true);
    ps.push_back(pc);
    Ints indexes;
    for (int j= i*(number_of_residues/nr.first)
           + first_residue_index; j <(i+1)*(number_of_residues/nr.first)
           + first_residue_index; ++j) {
      indexes.push_back(j);
    }
    pcd.set_residue_indexes(indexes);
    atom::Mass::setup_particle(pc, mass/nr.first);
  }
  IMP_INTERNAL_CHECK(pd.get_is_valid(true),
                     "Invalid hierarchy produced " << pd);
  return pd;
}


namespace {

  double get_mass(Hierarchy h) {
    if (Mass::particle_is_instance(h)) {
      return Mass(h).get_mass();
    } else {
      double mass=0;
      for (unsigned int i=0; i< h.get_number_of_children(); ++i) {
        mass+= get_mass(h.get_child(i));
      }
      return mass;
    }
  }
  // ignores overlap
  double get_volume(Hierarchy h) {
    if (core::XYZR::particle_is_instance(h)) {
      return algebra::get_volume(core::XYZR(h).get_sphere());
    } else {
      double volume=0;
      for (unsigned int i=0; i< h.get_number_of_children(); ++i) {
        volume+= get_volume(h.get_child(i));
      }
      return volume;
    }
  }
  void gather_residue_indices(Hierarchy h, Ints &inds) {
    if (Residue::particle_is_instance(h)) {
      int i=Residue(h).get_index();
      inds.push_back(i);
    } else if (Fragment::particle_is_instance(h)
               && h.get_number_of_children() != 0) {
      Ints v= Fragment(h).get_residue_indexes();
      inds.insert(inds.end(), v.begin(), v.end());
    } else if (Domain::particle_is_instance(h)
               && h.get_number_of_children() == 0) {
      Domain d(h);
      for ( int i=d.get_begin_index();
           i != d.get_end_index(); ++i) {
        inds.push_back(i);
      }
    } else {
      for (unsigned int i=0; i< h.get_number_of_children(); ++i) {
        gather_residue_indices(h.get_child(i), inds);
      }
    }
  }

  // create a particle which approximates the input set
  Hierarchy create_approximation(const HierarchiesTemp &t) {
    if (t.empty()) {
      return Hierarchy();
    }
    double m=0, v=0;
    Ints inds;
    algebra::VectorD<3> vv(0,0,0);
    unsigned int n=0;
    for (unsigned int i=0; i< t.size(); ++i) {
      m+= get_mass(t[i]);
      v+= get_volume(t[i]);
      gather_residue_indices(t[i], inds);
      core::XYZsTemp ls(get_leaves(t[i]));
      for (unsigned int i=0; i< ls.size(); ++i) {
        vv+= ls[i].get_coordinates();
      }
      n+= ls.size();
    }
    Model *mm= t[0]->get_model();
    Particle *p= new Particle(mm);
    Fragment f= Fragment::setup_particle(p);
    f.set_residue_indexes(inds);
    Mass::setup_particle(p, m);
    core::XYZR::setup_particle(p,
                         algebra::SphereD<3>(vv/n,
                         algebra::get_ball_radius_from_volume_3d(v)));
    return f;
  }
}


IMPATOMEXPORT Hierarchy create_simplified_along_backbone(Chain in,
                                                   int num_res) {
  if (in.get_number_of_children() ==0) {
    return Hierarchy();
  }
  IntRanges rs;
  int ib= Residue(in.get_child(0)).get_index();
  int ie= Residue(in.get_child(in.get_number_of_children()-1)).get_index()+1;
  int i;
  for (i=ib; i< ie; i+= num_res) {
    rs.push_back(IntRange(i, std::min(i+num_res, ie)));
  }
  return create_simplified_along_backbone(in, rs);
}

IMPATOMEXPORT Hierarchy
create_simplified_along_backbone(Chain in,
                            const IntRanges& residue_segments) {
  if (in.get_number_of_children() ==0 || residue_segments.empty()) {
    return Hierarchy();
  }
  for (unsigned int i=0; i< residue_segments.size(); ++i) {
    IMP_USAGE_CHECK(residue_segments[i].first < residue_segments[i].second,
                    "Residue intervals must be non-empty");
  }
  unsigned int cur_segment=0;
  HierarchiesTemp cur;
  Hierarchy root=create_clone_one(in);
  for (unsigned int i=0; i< in.get_number_of_children(); ++i) {
    int index= Residue(in.get_child(i)).get_index();
    if (index >= residue_segments[cur_segment].first
        && index < residue_segments[cur_segment].second) {
      cur.push_back(in.get_child(i));
    } else if (!cur.empty()) {
      IMP_LOG(TERSE, "Added particle for "
              << residue_segments[cur_segment].first
              << "..." << residue_segments[cur_segment].second
              << std::endl);
      root.add_child(create_approximation(cur));
      cur.clear();
      ++cur_segment;
    }
  }
  if (!cur.empty()) {
    root.add_child(create_approximation(cur));
  }
  IMP_INTERNAL_CHECK(root.get_is_valid(true),
                     "Invalid hierarchy produced " << root);
  return root;
}



IMPATOM_END_NAMESPACE
