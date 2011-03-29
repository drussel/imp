/**
 *  \file rigid_fitting.cpp
 *  \brief Rigid fitting functionalities
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 *
 */

#include <IMP/em/rigid_fitting.h>
#include <IMP/em/CoarseCC.h>
#include <IMP/em/SampledDensityMap.h>
#include <IMP/core/RigidBodyMover.h>
#include <IMP/algebra/vector_generators.h>
#include <IMP/SingletonModifier.h>
#include <IMP/core/Transform.h>
#include <IMP/core/SteepestDescent.h>
#include <IMP/atom/pdb.h>
#include <IMP/algebra/geometric_alignment.h>
#include <IMP/em/converters.h>
#include <IMP/algebra/eigen_analysis.h>
#include <IMP/core/LeavesRefiner.h>
#include <IMP/em/MRCReaderWriter.h> //remove this!!


IMPEM_BEGIN_NAMESPACE

void FittingSolutions::add_solution(
  const algebra::Transformation3D &t,Float score) {
    fs_.push_back(FittingSolution(t,score));
}
void FittingSolutions::sort(bool reverse) {
  std::sort(fs_.begin(),fs_.end(),sort_by_cc());
  if (reverse){
    std::reverse(fs_.begin(),fs_.end());}
}

namespace {
RestraintSet * add_restraints(Model *model, DensityMap *dmap,
                              core::RigidBody rb,Refiner *leaves_ref,
                const FloatKey &wei_key,
                bool fast=false) {
  RestraintSet *rsrs = new RestraintSet();
   model->add_restraint(rsrs);
   //add fitting restraint
   Pointer<FitRestraint> fit_rs;
   FloatPair no_norm_factors(0.,0.);
   if (fast) {
     fit_rs = new FitRestraint(rb.get_particle(),
                               dmap,leaves_ref,no_norm_factors,
                               wei_key,1.0);
   }
   else {
     fit_rs = new FitRestraint(leaves_ref->get_refined(rb),
                               dmap,leaves_ref,no_norm_factors,
                               wei_key,1.0);
   }
   rsrs->add_restraint(fit_rs);
   return rsrs;
}

core::MonteCarlo* set_optimizer(Model *model, OptimizerStates display_log,
   core::RigidBody rb,
   Int number_of_cg_steps, Float max_translation, Float max_rotation) {
  //create a rigid body mover
  core::RigidBodyMover *rb_mover =
     new core::RigidBodyMover(rb,max_translation,max_rotation);
  //preform mc search
  Pointer<core::MonteCarlo> opt(new core::MonteCarlo());
  opt->set_model(model);
  opt->add_mover(rb_mover);
  opt->set_return_best(true);//return the lowest energy state visited
  IMP::set_print_exceptions(true);

  //  core::SteepestDescent *lopt = new core::SteepestDescent();
  core::ConjugateGradients *lopt = new core::ConjugateGradients();
  lopt->set_threshold(0.001);
  //  lopt->set_step_size(0.05);
  opt->set_local_optimizer(lopt);
  opt->set_local_steps(number_of_cg_steps);

  //set the logging if provided
  for(int i=0;i<(int)display_log.size();i++){
    lopt->add_optimizer_state(display_log[i]);
    display_log[i]->update();
  }
  return opt.release();
}

void optimize(Int number_of_optimization_runs, Int number_of_mc_steps,
              const algebra::VectorD<3> &anchor_centroid,
              core::RigidBody *rb, Refiner *refiner, core::MonteCarlo *opt,
              FittingSolutions &fr, Model *) {
  Float e;
  core::XYZsTemp xyz_t(refiner->get_refined(*rb));
    algebra::VectorD<3> ps_centroid = IMP::core::get_centroid(xyz_t);
  //save starting configuration
    algebra::Transformation3D starting_trans = rb->get_reference_frame()
      .get_transformation_to();
  algebra::Transformation3D move2centroid(algebra::get_identity_rotation_3d(),
                                          anchor_centroid-ps_centroid);

  for(int i=0;i<number_of_optimization_runs;i++) {
    IMP_LOG(VERBOSE, "number of optimization run is : "<< i << std::endl);
    //TODO - should we return this line?
    rb->set_reference_frame(algebra::ReferenceFrame3D(starting_trans));
    //set the centroid of the rigid body to be on the anchor centroid
    //make sure that all of the members are in the correct transformation
    //TODO - should we keep this?
    //    core::transform(*rb,move2centroid);
    //optimize
    try {
      e = opt->optimize(number_of_mc_steps);
      fr.add_solution(rb->get_reference_frame()
                      .get_transformation_to()/starting_trans,e);
    } catch (ModelException err) {
      IMP_WARN("Optimization run " << i << " failed to converge."
               << std::endl);
    }
  }
  //return the rigid body to the original position
  rb->set_reference_frame(algebra::ReferenceFrame3D(starting_trans));
}
}

FittingSolutions local_rigid_fitting_around_point(
   core::RigidBody rb,Refiner *refiner,
   const FloatKey &wei_key,
   DensityMap *dmap, const algebra::VectorD<3> &anchor_centroid,
   OptimizerStates display_log,
   Int number_of_optimization_runs, Int number_of_mc_steps,
   Int number_of_cg_steps, Float max_translation, Float max_rotation,
   bool fast) {
  FittingSolutions fr;
   IMP_LOG(TERSE,
          "rigid fitting with " << number_of_optimization_runs <<
          " MC optimization runs, each with " << number_of_mc_steps <<
          " Monte Carlo steps , each with " << number_of_cg_steps <<
          " Conjugate Gradients rounds. " << std::endl
           <<" The anchor point is : " << anchor_centroid << std::endl
           <<" The map center is : " << dmap->get_centroid() << std::endl);
   if(!dmap->is_part_of_volume(anchor_centroid)) {
     IMP_WARN("starting local refinement with a protein mostly outside "
              <<"of the density"<<std::endl);
   }
   //add restraints
   Model *model = rb.get_members()[0].get_particle()->get_model();
   RestraintSet *rsrs = add_restraints(model, dmap, rb,refiner,
                                       wei_key,fast);
   //create a rigid body mover and set the optimizer
   core::MonteCarlo *opt = set_optimizer(model, display_log, rb,
                           number_of_cg_steps, max_translation, max_rotation);

   //optimize

   IMP_LOG(VERBOSE,"before optimizer"<<std::endl);
   optimize(number_of_optimization_runs, number_of_mc_steps,
            anchor_centroid, &rb, refiner,opt, fr, model);

   fr.sort();
   IMP_IF_LOG(TERSE) {
     IMP_LOG(TERSE, "Solutions are: ");
     for (int i=0; i < fr.get_number_of_solutions(); ++i) {
       IMP_LOG(TERSE, fr.get_score(i) << " " << fr.get_transformation(i)
               << " -- ");
     }
     IMP_LOG(TERSE, std::endl);
   }
    //remove restraints
    model->remove_restraint(rsrs);
    IMP_LOG(TERSE,"end rigid fitting " <<std::endl);
    opt=NULL;
    return fr;
}

FittingSolutions local_rigid_fitting_around_points(
   core::RigidBody rb,Refiner *refiner,
   const FloatKey &wei_key,
   DensityMap *dmap, const std::vector<algebra::VectorD<3> > &anchor_centroids,
   OptimizerStates display_log,
   Int number_of_optimization_runs, Int number_of_mc_steps,
   Int number_of_cg_steps, Float max_translation, Float max_rotation) {
  FittingSolutions fr;
   IMP_LOG(VERBOSE,
           "rigid fitting around " << anchor_centroids.size() <<" with "
           << number_of_optimization_runs <<
           " MC optimization runs, each with " << number_of_mc_steps <<
           " Monte Carlo steps , each with " << number_of_cg_steps <<
           " Conjugate Gradients rounds. " << std::endl);
   Model *model = rb.get_members()[0].get_particle()->get_model();

   RestraintSet *rsrs = add_restraints(model, dmap, rb,refiner,
                                       wei_key);
   core::MonteCarlo *opt = set_optimizer(model, display_log, rb,
                           number_of_cg_steps,max_translation, max_rotation);

   for(std::vector<algebra::VectorD<3> >::const_iterator it
         = anchor_centroids.begin();
       it != anchor_centroids.end(); it++) {
     IMP_INTERNAL_CHECK(dmap->is_part_of_volume(*it),
                "The centroid is not part of the map");
     IMP_LOG(VERBOSE, "optimizing around anchor point " << *it << std::endl);
     optimize(number_of_optimization_runs,
              number_of_mc_steps,*it,&rb,refiner,opt,fr,model);
   }
   fr.sort();
   model->remove_restraint(rsrs);
   IMP_LOG(TERSE,"end rigid fitting " <<std::endl);
   return fr;
}

FittingSolutions local_rigid_fitting_grid_search(
   const Particles &ps,
   const FloatKey &wei_key,
   DensityMap *dmap,
   Int max_voxels_translation,
   Int translation_step,
   Float max_angle_in_radians,
   Int number_of_rotations) {
  FittingSolutions fr;
   IMP_USAGE_CHECK(ps.size()>0,
      "no particles given as input for local_rigid_fitting_grid_search"
       <<std::endl);
   Float max_t = dmap->get_spacing()*max_voxels_translation;
   Float step_t = dmap->get_spacing()*translation_step;

   IMP_LOG(TERSE,
      "going to preform local rigid fitting using a grid search method"
      << " on " << ps.size() << " particles. "<<std::endl
      <<"The grid search parameters are: " <<" translation between "
      << -max_t << " to " << max_t <<" with step : " << step_t
      << " number of rotations : " << number_of_rotations <<std::endl);

   //init the sampled density map
   IMP::em::SampledDensityMap *model_dens_map =
       new IMP::em::SampledDensityMap(*dmap->get_header());
   IMP_INTERNAL_CHECK(
      model_dens_map->same_dimensions(dmap),
      "sampled density map is of wrong dimensions"<<std::endl);
   model_dens_map->set_particles(ps,wei_key);
   model_dens_map->resample();
   model_dens_map->calcRMS();
   algebra::Rotation3Ds rots;
   //algebra::get_uniform_cover_rotations_3d(number_of_rotations);
   for(int i=0;i<number_of_rotations;i++) {
     algebra::VectorD<3> axis =
       algebra::get_random_vector_on(
          algebra::Sphere3D(algebra::VectorD<3>(0.0,0.0,0.0),1.));
     ::boost::uniform_real<> rand(-max_angle_in_radians,max_angle_in_radians);
     Float angle =rand(random_number_generator);
     algebra::Rotation3D r =
       algebra::get_rotation_about_axis(axis, angle);
     rots.push_back(r);
   }
   unsigned int rot_ind=static_cast<unsigned int>(-1);
   for(algebra::Rotation3Ds::iterator it = rots.begin();
                                      it != rots.end();it++) {
     ++rot_ind;
     IMP_LOG(IMP::TERSE,"working on rotation "<<
         rot_ind<<" out of "<< rots.size()<<std::endl);
     algebra::Transformation3D t1 =algebra::get_rotation_about_point(
                                 core::get_centroid(core::XYZsTemp(ps)),*it);
     Pointer<DensityMap> rotated_sampled_map =
       get_transformed(model_dens_map,t1);
     rotated_sampled_map->calcRMS();
     algebra::VectorD<3>
          origin(model_dens_map->get_header()->get_xorigin(),
                 model_dens_map->get_header()->get_yorigin(),
                 model_dens_map->get_header()->get_zorigin());
     Float score;
     for(Float x=-max_t; x<=max_t;x += step_t){
       for(Float y=-max_t; y<=max_t;y += step_t){
         for(Float z=-max_t; z<=max_t;z += step_t){
           algebra::Transformation3D t =
              algebra::Transformation3D(algebra::get_identity_rotation_3d(),
                                        algebra::VectorD<3>(x,y,z));
              rotated_sampled_map->set_origin(t.get_transformed(origin));
              float threshold = rotated_sampled_map->get_header()->dmin;
              score = 1.-IMP::em::CoarseCC::cross_correlation_coefficient(
                       dmap,rotated_sampled_map,threshold,true);
           fr.add_solution(IMP::algebra::compose(t,t1),score);
           model_dens_map->set_origin(origin);
         }//z
       }//y
     }//x
     rotated_sampled_map=NULL;
   }//end rotations
   fr.sort();
   return fr;
}

FittingSolutions compute_fitting_scores(const Particles &ps,
  DensityMap *em_map,
  const algebra::Transformation3Ds &transformations,
  bool fast_version,
  bool local,
  const FloatKey &wei_key) {
  FittingSolutions fr;
  //move the particles to the center of the map
  algebra::Transformation3D move_ps_to_map_center=
    algebra::Transformation3D(
      algebra::get_identity_rotation_3d(),
      em_map->get_centroid()-core::get_centroid(core::XYZsTemp(ps)));
  IMP_LOG(TERSE,"moving particles to center:"<<move_ps_to_map_center<<"\n");
  core::XYZs ps_xyz(ps);
  for(int i=0;i<(int)ps_xyz.size();i++) {
    ps_xyz[i].set_coordinates(
         move_ps_to_map_center.get_transformed(ps_xyz[i].get_coordinates()));
  }
  //now update all of the transformations accordinly
  algebra::Transformation3Ds trans_for_fit;
  for(int i=0;i<(int)transformations.size();i++) {
    trans_for_fit.push_back(transformations[i]*
                            move_ps_to_map_center.get_inverse());
    IMP_LOG(VERBOSE,"Using transformation:"<<trans_for_fit[i]<<" instead of"
            <<transformations[i]<<std::endl);
  }
  IMP_NEW(IMP::em::SampledDensityMap,model_dens_map,
         (*(em_map->get_header())));
  model_dens_map->set_particles(ps,wei_key);
  model_dens_map->resample();
  model_dens_map->calcRMS();
  std::cout<<"RMS: "<< model_dens_map->get_header()->rms<<std::endl;
  IMP_INTERNAL_CHECK(model_dens_map->same_dimensions(em_map),
                       "sampled density map is of wrong dimensions"<<std::endl);
  float score;
  if ((model_dens_map->get_header()->rms<EPS) && fast_version) {
    IMP_WARN("Map is empty, fast mode will return nan for all "<<
             "transformations\n");
  }
  if (!fast_version) {
    IMP_LOG(IMP::VERBOSE,"running slow version of compute_fitting_scores"
            <<std::endl);
    IMP_NEW(IMP::em::SampledDensityMap,model_dens_map2,
            (*(em_map->get_header())));
    model_dens_map2->set_particles(ps,wei_key);
    model_dens_map2->resample();
    model_dens_map2->calcRMS();
    IMP_LOG(VERBOSE,
     "model_dens_map2 rms:"<<model_dens_map2->get_header()->rms<<
              " particles centroid: "<<
            core::get_centroid(core::XYZsTemp(ps))<<std::endl);
    algebra::Vector3Ds original_cooridnates;
    for(core::XYZs::const_iterator it = ps_xyz.begin();
        it != ps_xyz.end(); it++) {
      original_cooridnates.push_back(it->get_coordinates());
    }
    for (algebra::Transformation3Ds::const_iterator it =
           trans_for_fit.begin(); it != trans_for_fit.end();it++) {
      //transform the particles
      for(unsigned int i=0;i<ps_xyz.size();i++){
        ps_xyz[i].set_coordinates(
                       it->get_transformed(original_cooridnates[i]));
      }
      model_dens_map2->resample();
      model_dens_map2->calcRMS();
      IMP_LOG(VERBOSE,
        "model_dens_map2 rms:"<<model_dens_map2->get_header()->rms<<
        " particles centroid: "<< core::get_centroid(core::XYZsTemp(ps))<<
              "trans: "<< *it<<
        std::endl);
      float threshold = model_dens_map2->get_header()->dmin-EPS;
      if (!local) {
        score  = 1.-
          CoarseCC::cross_correlation_coefficient(em_map,
                     model_dens_map2,threshold,true);
        }
        else {
        score  = 1.-
          CoarseCC::local_cross_correlation_coefficient(em_map,
             model_dens_map2,threshold);
        }
        IMP_LOG(VERBOSE,"adding score:"<<score<<std::endl);
        fr.add_solution(*it*move_ps_to_map_center,score);
      }
      //move back to original coordinates
      for(unsigned int i=0;i<ps_xyz.size();i++){
        ps_xyz[i].set_coordinates(original_cooridnates[i]);
      }
      model_dens_map2=NULL;
    }
    else { //fast version
      // static int counter=0;
      IMP_LOG(IMP::VERBOSE,
              "running fast version of compute_fitting_scores"<<std::endl);
      for (algebra::Transformation3Ds::const_iterator it =
         trans_for_fit.begin(); it != trans_for_fit.end();it++) {
        Pointer<DensityMap> transformed_sampled_map = get_transformed(
              model_dens_map,*it);
        IMP_INTERNAL_CHECK(
           transformed_sampled_map->same_dimensions(model_dens_map),
           "sampled density map changed dimensions after transformation"
           <<std::endl);
        IMP_INTERNAL_CHECK(transformed_sampled_map->same_dimensions(em_map),
                 "sampled density map is of wrong dimensions"<<std::endl);
        float threshold = transformed_sampled_map->get_header()->dmin;
        if (!local) {
          score  = 1.-
          CoarseCC::cross_correlation_coefficient(em_map,
                     transformed_sampled_map,threshold,true);
        }
        else {
        score  = 1.-
          CoarseCC::local_cross_correlation_coefficient(em_map,
             transformed_sampled_map,threshold);
        }
        IMP_LOG(VERBOSE,"adding score:"<<score<<std::endl);
        fr.add_solution(*it*move_ps_to_map_center,score);
        transformed_sampled_map=NULL;
      }
    }
    model_dens_map=NULL;
  //move the particles back to original position
    algebra::Transformation3D move_ps_to_map_center_inv =
      move_ps_to_map_center.get_inverse();
    for(int i=0;i<(int)ps_xyz.size();i++) {
      ps_xyz[i].set_coordinates(
       move_ps_to_map_center_inv.get_transformed(ps_xyz[i].get_coordinates()));
    }
    //return results
    return fr;
}

Float compute_fitting_score(const Particles &ps,
                            DensityMap *em_map,
                            FloatKey wei_key) {
    IMP::em::SampledDensityMap *model_dens_map =
      new IMP::em::SampledDensityMap(*(em_map->get_header()));
   model_dens_map->set_particles(ps,wei_key);
   model_dens_map->resample();
   return em::CoarseCC::calc_score(em_map, model_dens_map,
                                 1.0,true,false);
}


IMPEM_END_NAMESPACE
