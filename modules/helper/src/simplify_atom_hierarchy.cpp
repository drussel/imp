/**
 *  \file covers.cpp
 *  \brief Support for rigid bodies.
 *
 *  Copyright 2007-2010 Sali Lab. All rights reserved.
 *
 */

#include "IMP/helper/simplify_atom_hierarchy.h"
#include <IMP/SingletonContainer.h>
#include <IMP/algebra/vector_generators.h>
#include <IMP/atom.h>
#include <IMP/core.h>
#include <IMP/display/geometry.h>
#include <IMP/display/particle_geometry.h>
#include <IMP/display/LogOptimizerState.h>
#include <IMP/display/ChimeraWriter.h>
#include <IMP/display/particle_geometry.h>
#include <IMP/display/PymolWriter.h>
#include <IMP/em/FitRestraint.h>
#include <IMP/em/MRCReaderWriter.h>
#include <IMP/em/SampledDensityMap.h>
#include <algorithm>

IMPHELPER_BEGIN_NAMESPACE



namespace {


  /*************************************************************

   Below this is for the _2 version

  *************************************************************/

  void setup_fragments(const atom::Fragments& fragments,
                       const atom::Hierarchies &leaves) {
    std::vector<Ints> residue_indexes(fragments.size());
    std::vector<double> masses(fragments.size(), 0);

    for (unsigned int i=0; i< leaves.size(); ++i) {
      algebra::Vector3D v= core::XYZ(leaves[i]).get_coordinates();
      double md=std::numeric_limits<double>::max();
      int mj=-1;
      for (unsigned int j=0; j< fragments.size(); ++j) {
        algebra::Sphere3D s= core::XYZR(fragments[j]).get_sphere();
        double d= distance(s.get_center(), v)-s.get_radius();
        if (d < md) {
          md=d;
          mj=j;
        }
      }
      masses[mj]+= atom::Mass(leaves[i]).get_mass();
      if (atom::Atom::particle_is_instance(leaves[i])) {
        residue_indexes[mj]
          .push_back(get_residue(atom::Atom(leaves[i])).get_index());
      } else if (atom::Residue::particle_is_instance(leaves[i])) {
        residue_indexes[mj]
          .push_back(atom::Residue(leaves[i]).get_index());
      } else if (atom::Fragment::particle_is_instance(leaves[i])) {
        Ints cis= atom::Fragment(leaves[i]).get_residue_indexes();
        residue_indexes[mj].insert(residue_indexes[mj].end(),
                                   cis.begin(), cis.end());
      }
    }

    for (unsigned int i=0; i< residue_indexes.size(); ++i) {
      std::sort(residue_indexes[i].begin(), residue_indexes[i].end());
      residue_indexes[i].erase(std::unique(residue_indexes[i].begin(),
                                           residue_indexes[i].end()),
                            residue_indexes[i].end());
      fragments[i].set_residue_indexes(residue_indexes[i]);
      atom::Mass(fragments[i]).set_mass(masses[i]);
    }
  }





  class IncrementalRadiusMassMover :public core::Mover
  {
  public:
    IncrementalRadiusMassMover(SingletonContainer *sc,
                               unsigned int n,
                               Float delta_radius,
                               Float min, Float max);
    IMP_MOVER(IncrementalRadiusMassMover, get_module_version_info())
    private:
    IMP::internal::OwnerPointer<SingletonContainer> sc_;
    unsigned int n_;
    Float radius_delta_;
    ParticlesTemp moved_;
    double min_, max_;
  };



  IncrementalRadiusMassMover
  ::IncrementalRadiusMassMover(SingletonContainer *sc,
                               unsigned int n,
                               Float radius,
                               Float min, Float max): sc_(sc),
                                                      n_(n),
                                                      radius_delta_(radius),
                                                      moved_(n_),
                                                      min_(min), max_(max)
  {
  }

  void IncrementalRadiusMassMover::propose_move(Float size) {
    // damnit, why didn't these functions make it into the standard
    /*std::random_sample(sc_->particles_begin(), sc_->particles_end(),
      moved_.begin(), moved_.end());*/
    ParticlesTemp pts(sc_->get_particles());
    std::random_shuffle(pts.begin(), pts.end());
    for (unsigned int i=0; i< n_; ++i) {
      moved_[i]=pts[i];
      core::XYZR d(moved_[i]);
      atom::Mass m(moved_[i]);
      double ro= d.get_radius();
      double nv=algebra::random_vector_in_sphere<1>
        (algebra::VectorD<1>(d.get_radius()),
                    radius_delta_)[0];
      d.set_radius(std::min(std::max(min_, nv), max_));
      double mn= m.get_mass()/std::pow(ro,3) * std::pow(d.get_radius(),3);
      m.set_mass(mn);
    }
  }


  void IncrementalRadiusMassMover::accept_move() {
  }

  void IncrementalRadiusMassMover::reject_move() {
    for (unsigned int i=0; i< n_; ++i) {
      core::XYZR odr(moved_[i]->get_prechange_particle());
      core::XYZR cdr(moved_[i]);
      atom::Mass odm(moved_[i]->get_prechange_particle());
      atom::Mass cdm(moved_[i]);
      cdr.set_radius(odr.get_radius());
      cdm.set_mass(odm.get_mass());
    }
  }


  void IncrementalRadiusMassMover::show(std::ostream &out) const {
    out << "IncrementalRadiusMassMover" << std::endl;
  }


  class WriteMap:public FailureHandlerBase {
    IMP::internal::OwnerPointer<em::DensityMap> dm_;
    std::string file_name_;
  public:
    WriteMap(em::DensityMap *dm, std::string fn): dm_(dm), file_name_(fn){}
    void handle_failure() {
      em::MRCReaderWriter wr;
      em::write_map(dm_, file_name_.c_str(), wr);
    }
  };

}

/*
  r= resolution/2.0
  n*4/3 pi *r^3=3/2*v
  n= 9/4*v/(pi r^3)
*/
atom::Hierarchy create_simplified(atom::Hierarchy in,
                                  double resolution) {
  atom::Hierarchies leaves(atom::get_leaves(in));

  // rasterize protein to grid with size resolution/2.0
  IMP_NEW(em::SampledDensityMap, sdm,
          (leaves,resolution, resolution/3.0,
           core::XYZR::get_default_radius_key(),
           atom::Mass::get_mass_key(),1));
  {
    em::MRCReaderWriter mrc; // for some reason it wants a non-const ref
    em::write_map(sdm, "map.mrc", mrc);

    IMP_NEW(display::ChimeraWriter, cw, ("mapdata.py"));
    IMP_NEW(display::BoundingBoxGeometry,
            bbg, (em::get_bounding_box(sdm)));
    bbg->set_name("map_box");
    cw->add_geometry(bbg);
    IMP_NEW(display::BoundingBoxGeometry, bbp,
            (atom::get_bounding_box(in)));
    bbp->set_name("protein_box");
    cw->add_geometry(bbp);
    IMP_NEW(display::BoundingBoxGeometry, bbv,
            (algebra::BoundingBox3D(sdm->get_origin(),
                                    sdm->get_origin()
                                    +algebra::Vector3D(resolution/3.0,
                                                       resolution/3.0,
                                                       resolution/3.0))));
    bbv->set_name("voxel");
    cw->add_geometry(bbv);
  }

  // add up mass of leaves
  double total_mass=0;
  for (unsigned int i=0; i< leaves.size(); ++i) {
    total_mass+= atom::Mass(leaves[i]).get_mass();
  }

  // estimate volume using grid
  double v=atom::volume_from_mass(total_mass);
  IMP_LOG(TERSE, "Estimated volume is " << v << std::endl);


  // compute parameters
  unsigned int n=std::ceil(9*v/(PI *std::pow(resolution, 3)));
  std::cout << "Guess n of "
            << n
            << std::endl;
  algebra::BoundingBox3D bb=get_bounding_box(in);
  double r0=std::pow((3.0/4)*v/n, .33333);
  double m0= 2.0*total_mass/n;
  IMP_LOG(TERSE, "Using " << n << " spheres." << std::endl);

    // create new model and particles to optimize
    // TODO use qmm or something to pick starting points
    IMP_NEW(Model, m, ());
    m->set_is_incremental(true);
    core::XYZRs xyzrs(n);
    for (unsigned int i=0; i< n; ++i) {
      xyzrs.set(i,
                core::XYZR::setup_particle(new Particle(m),
                    algebra::Sphere3D(algebra::random_vector_in_box(bb),
                                                             r0)));
      xyzrs[i].set_coordinates_are_optimized(true);
      std::ostringstream oss;
      oss << "simplified fragment " << i;
      xyzrs[i].get_particle()->set_name(oss.str());
      atom::Mass::setup_particle(xyzrs[i], m0);
    }





    IMP_NEW(core::ListSingletonContainer, lsc, (xyzrs));
    IMP_NEW(core::BoundingBox3DSingletonScore, bss,
            (new core::HarmonicUpperBound(0,1), bb));
    IMP_NEW(core::SingletonsRestraint, sr, (bss, lsc));
    IMP_NEW(em::FitRestraint, ccr, (xyzrs, sdm,
                                    core::XYZR::get_default_radius_key(),
                                    atom::Mass::get_mass_key(),
                                    1.0));
    m->add_restraint(sr);
    m->add_restraint(ccr);


    IMP_NEW(core::AllPairContainer, appc, (lsc));
    IMP_NEW(core::NormalizedSphereDistancePairScore, nsdp,
            (new core::HarmonicLowerBound(-1,1)));
    IMP_NEW(core::PairsRestraint, pr, (nsdp, appc));
    m->add_restraint(pr);
    // create MC
    IMP_NEW(core::MonteCarlo, mc, ());
    mc->set_score_threshold(.2);
    mc->set_temperature(.01);
    mc->set_return_best(true);
    ScopedFailureHandler fh0, fh1, fh2;
    IMP_NEW(display::LogOptimizerState, los,
            (new display::ChimeraWriter(), "frame.%04d.py"));
    IMP_NEW(display::XYZRsGeometry, xyzrg, (lsc,
                             core::XYZR::get_default_radius_key()));
    los->add_geometry(xyzrg);
    if (0) {
      IMP_NEW(display::LogOptimizerState, plos,
              (new display::PymolWriter(), "frame.%04d.pymol.pym"));
      mc->add_optimizer_state(los);
      mc->add_optimizer_state(plos);

      plos->add_geometry(xyzrg);
      los->write("initial.py");
      los->set_skip_steps(0);
      IMP_NEW(core::DumpModelOnFailure, dmf, (m, "error.imp"));
      fh0.set(dmf);
      IMP_NEW(display::DisplayModelOnFailure, lof,
              (los, "error.py"));
      fh1.set(lof);
      fh2.set(new WriteMap(ccr->get_model_dens_map(), "error.mrc"));
    }
    mc->set_model(m);
    IMP_NEW(core::ConjugateGradients, cg, ());
    /*IMP_NEW(display::LogOptimizerState, los2,
      (new display::PymolWriter(), "local_frame.%04d.pym"));
      los2->add_geometry(xyzrg);
      //cg->add_optimizer_state(los2);*/
    mc->set_local_optimizer(cg);
    mc->set_local_steps(10);
    mc->add_mover(new core::IncrementalBallMover(lsc, 1,
                                                 resolution/2.0));
    mc->add_mover(new IncrementalRadiusMassMover(lsc, 1, resolution/10.0,
                                                 resolution/2.0, 4*resolution));
    // set up fit restraint

    // run MC
    set_log_level(SILENT);
    mc->set_log_level(TERSE);
    m->set_log_level(SILENT);
    cg->set_log_level(SILENT);
    // try several times if optimization fails
    unsigned int trial_count=0;
    do {
      for (unsigned int i=0; i< n; ++i) {
        xyzrs[i].set_coordinates( algebra::random_vector_in_box(bb));
      }
      try {
        mc->optimize(100);
      } catch (const ModelException &e) {
        ++trial_count;
        if (trial_count==10) {
          IMP_THROW("Unable to simplify protein for some reason. "
                    << " Please send us your input.",
                    ValueException);
        }
        continue;
      }
      break;
    } while (true);
  em::MRCReaderWriter rw;
  em::write_map(ccr->get_model_dens_map(), "final.mrc", rw);
  los->write("final.py");
  std::cout << "energy is " << m->evaluate(false) << std::endl;
  /*remove_failure_handler(dmf);
    remove_failure_handler(lof);*/

  // copy centers back
  atom::Fragments frags(n);
  for (unsigned int i=0; i< n; ++i) {
    frags.set(i, atom::Fragment::setup_particle(new Particle(in.get_model())));
    core::XYZR::setup_particle(frags[i], xyzrs[i].get_sphere());
    atom::Mass::setup_particle(frags[i], 0);
  }
  // X assign residues and masses
  setup_fragments(frags, leaves);
  // create hierarchy
  atom::Hierarchy ret
    = atom::Hierarchy::setup_particle(new Particle(in.get_model()));
  for (unsigned int i=0; i< frags.size(); ++i) {
    ret.add_child(frags[i]);
  }
  return ret;
}




IMPHELPER_END_NAMESPACE
