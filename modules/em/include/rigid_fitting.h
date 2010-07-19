/**
 *  \file  rigid_fitting.h
 *  \brief preforms rigid fitting between a set of particles and a density map
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 */
#ifndef IMPEM_RIGID_FITTING_H
#define IMPEM_RIGID_FITTING_H

#include <IMP/core/MonteCarlo.h>
#include <IMP/core/ConjugateGradients.h>
#include <IMP/RestraintSet.h>
#include <IMP/algebra/Transformation3D.h>
#include <IMP/VersionInfo.h>
#include <IMP/Particle.h>
#include <IMP/Model.h>
#include "DensityMap.h"
#include "FitRestraint.h"
#include "em_config.h"
#include <IMP/OptimizerState.h>
#include <IMP/core/rigid_bodies.h>
#include <IMP/ScoreState.h>
#include <algorithm>
IMPEM_BEGIN_NAMESPACE


//! A simple list of fitting solutions.
/**
   \see local_rigid_fitting_around_point
   \see local_rigid_fitting_around_points
   \see local_rigid_fitting_grid_search
   \see compute_fitting_scores
 */
class IMPEMEXPORT FittingSolutions {
typedef std::pair<algebra::Transformation3D,Float> FittingSolution;
struct sort_by_cc
{
  bool operator()(const FittingSolution &s1, const FittingSolution & s2) const
  {
    return s1.second < s2.second;
  }
};
public:
  //! Get the number of solutions in the set
  inline int get_number_of_solutions() const {return fs_.size();}
  //! Get the score of the i'th solution
  /**
    \return the i'th transformation, or throw an exception
     if the index is out of range
  */
  inline algebra::Transformation3D get_transformation(unsigned int i) const {
    IMP_USAGE_CHECK(i<fs_.size(),"The index requested ("<<
       i<<") in get_transformation is our of range ("<<
       fs_.size()<<")"<<std::endl);
    return fs_[i].first;
  }
  //! Get the score of the i'th solution
  /**
    \return the i'th score, or throw an exception
            if the index is out of range
  */
  inline Float get_score(unsigned int i) const {
    IMP_USAGE_CHECK(i<fs_.size(),"The index requested ("<<
       i<<") in get_transformation is out of range ("<<
       fs_.size()<<")"<<std::endl);
    return fs_[i].second;
  }
  //! Add a solution to the fitting solution set
  void add_solution(const algebra::Transformation3D &t,Float score);
  //! Sort solutions by cross-correlation scores
  void sort();
protected:
  std::vector<FittingSolution> fs_;
};

//! Local rigid fitting of a rigid body around a center point
/**
\brief Fit a set of particles to a density map around an anchor point.
       The fitting is assessed using the cross-correaltion score.
       The optimization is a standard MC/CG procedure.
       The function returns a list of solutions sortedo the cross-correlation
       score.
\note The transformations are the rigid-body tranformation
       (with respect to an internal coordinate system). To get the actual
       delta transformation from the original placement of the rigid body,
       use the operator/ with a reference trasnforamtion outside of this
       function.
\note The returned cross-correlation score is 1-cc, as we usually want to
      minimize a scroing function. Thus a score of 1 means no-correlation
      and a score of 0. is perfect correlation.
\note The input rigid body should be also IMP::atom::Hierarchy
\param[in] rb          The rigid body to fit
\param[in] radius_key  The raidus key of the particles in the rigid body
\param[in] weight_key  The weight key of the particles in the rigid body
\param[in] dmap        The density map to fit to
\param[in] anchor_centroid    The point to fit the particles around
\param[in] display_log If provided, then intermediate states
                       in during the optimization procedure are printed
\param[in] number_of_optimization_runs  number of Monte Carlo optimizations
\param[in] number_of_mc_steps  number of steps in a Monte Carlo optimization
\param[in] number_of_cg_steps  number of Conjugate Gradients steps in
                               a Monte Carlo step
\param[in] max_translation maximum translation step in a MC optimization step
\param[in] max_rotation maximum rotation step in a single MC optimization step
\param[in] fast if true the density map of the rigid body is not resampled
                but transformed at each iteration of the optimization
\return the refined fitting solutions
*/
IMPEMEXPORT FittingSolutions local_rigid_fitting_around_point(
   core::RigidBody rb, const FloatKey &radius_key,
   const FloatKey &weight_key,
   DensityMap *dmap, const algebra::VectorD<3> &anchor_centroid,
   OptimizerState *display_log,
   Int number_of_optimization_runs = 5, Int number_of_mc_steps = 10,
   Int number_of_cg_steps=100,
   Float max_translation=2., Float max_rotation=.3,bool fast=false);

//! Local rigid fitting of a rigid body
/**
\brief Fit a set of particles to a density map around their centroid.
       The fitting is assessed using the cross-correaltion score.
       The optimization is a standard MC/CG procedure.
       The function returns a list of solutions sortedo the cross-correlation
       score.
\note The transformations are the rigid-body tranformation
       (with respect to an internal coordinate system). To get the actual
       delta transformation from the original placement of the rigid body,
       use the operator/ with a reference trasnforamtion outside of this
       function.
\note The returned cross-correlation score is 1-cc, as we usually want to
      minimize a scroing function. Thus a score of 1 means no-correlation
      and a score of 0. is perfect correlation.
\note The input rigid body should be also IMP::atom::Hierarchy
\param[in] rb          The rigid body to fit
\param[in] radius_key  The raidus key of the particles in the rigid body
\param[in] weight_key  The weight key of the particles in the rigid body
\param[in] dmap        The density map to fit to
\param[in] display_log If provided, then intermediate states
                       in during the optimization procedure are printed
\param[in] number_of_optimization_runs  number of Monte Carlo optimizations
\param[in] number_of_mc_steps  number of steps in a Monte Carlo optimization
\param[in] number_of_cg_steps  number of Conjugate Gradients steps in
                               a Monte Carlo step
\param[in] max_translation maximum translation step in a MC optimization step
\param[in] max_rotation maximum rotation step in radians in a single
                        MC optimization step
\param[in] fast if true the density map of the rigid body is not resampled
                but transformed at each iteration of the optimization
\return the refined fitting solutions
*/

inline FittingSolutions local_rigid_fitting(
   core::RigidBody rb, const FloatKey &radius_key,
   const FloatKey &weight_key,
   DensityMap *dmap,
   OptimizerState *display_log=NULL,
   Int number_of_optimization_runs = 5, Int number_of_mc_steps = 10,
   Int number_of_cg_steps=100,
   Float max_translation=2., Float max_rotation=.3,
   bool fast=false) {
   algebra::Vector3D rb_cen=
 IMP::core::get_centroid(core::XYZsTemp(core::get_leaves(atom::Hierarchy(rb))));
   return local_rigid_fitting_around_point(
     rb, radius_key, weight_key,dmap, rb_cen,display_log,
     number_of_optimization_runs, number_of_mc_steps,
     number_of_cg_steps, max_translation, max_rotation,fast);
}


//! Local rigid fitting of a rigid body around a set of center points
/**
\brief Fit a set of particles to a density map around each of the input points.
       The function apply local_rigid_fitting_around_point around each center.
\note The input rigid body should be also IMP::atom::Hierarchy
\param[in] rb          The rigid body to fit
\param[in] rad_key  The raidus key of the particles in the rigid body
\param[in] wei_key  The weight key of the particles in the rigid body
\param[in] dmap        The density map to fit to
\param[in] anchor_centroids    The points to fit the particles around
\param[in] display_log If provided, then intermediate states
                       in during the optimization procedure are printed
\param[in] number_of_optimization_runs  number of Monte Carlo optimizations
\param[in] number_of_mc_steps  number of steps in a Monte Carlo optimization
\param[in] number_of_cg_steps  number of Conjugate Gradients steps in
                               a Monte Carlo step
\param[in] max_translation maximum translation step in a MC optimization step
\param[in] max_rotation maximum rotation step in a single MC optimization step
\return the refined fitting solutions
*/
IMPEMEXPORT FittingSolutions local_rigid_fitting_around_points(
   core::RigidBody rb,
   const FloatKey &rad_key, const FloatKey &wei_key,
   DensityMap *dmap, const std::vector<algebra::VectorD<3> > &anchor_centroids,
   OptimizerState *display_log,
   Int number_of_optimization_runs = 5, Int number_of_mc_steps = 10,
   Int number_of_cg_steps=100,
   Float max_translation=2., Float max_rotation=.3);


//! Local grid search rigid fitting
/**
\brief Fit a set of particles to a density map around their centroid.
       The fitting is assessed using the cross-correaltion score.
       The optimization is a grid search
\note The transformations are not clustered.
\note The returned cross-correlation score is 1-cc, as we usually want to
      minimize a scroing function. Thus a score of 1 means no-correlation
      and a score of 0. is perfect correlation.
\param[in] ps       The particles to be fitted (treated rigid)
\param[in] rad_key  The raidus key of the particles in the rigid body
\param[in] wei_key  The weight key of the particles in the rigid body
\param[in] dmap        The density map to fit to
\param[in] max_voxels_translation Sample translations within
                                 -max_voxel_translation to max_voxel_translation
\param[in] translation_step      The translation sampling step
\param[in] max_angle_in_radians Sample rotations with +- max_angle_in_radians
                                around the current rotation
\param[in] number_of_rotations   The number of rotations to sample
\return the refined fitting solutions
*/
IMPEMEXPORT FittingSolutions local_rigid_fitting_grid_search(
   const Particles &ps, const FloatKey &rad_key,
   const FloatKey &wei_key,
   DensityMap *dmap,
   Int max_voxels_translation=2,
   Int translation_step=1,
   Float max_angle_in_radians = 0.174,
   Int number_of_rotations = 100);


//! Compute fitting scores for a given set of rigid transformations
/**
\brief Score how well a set of particles fit to the map in various
       rigid transformations.
\param[in] ps       The particles to be fitted (treated rigid)
\param[in] em_map        The density map to fit to
\param[in] rad_key  The raidus key of the particles in the rigid body
\param[in] wei_key  The weight key of the particles in the rigid body
\param[in] fast_version If fast_version is used the sampled density map of the
                        input particles (ps) is not resampled for each
                        transformation but instead the corresponding grid
                        is rotated. This option significantly improves the
                        running times but the returned scores are less accurate
\param[in] transformations   A set of rigid transformations
\return The scored fitting solutions
\note the function assumes the density map holds its density
 */
IMPEMEXPORT FittingSolutions compute_fitting_scores(const Particles &ps,
   DensityMap *em_map,
   const FloatKey &rad_key, const FloatKey &wei_key,
   const algebra::Transformation3Ds& transformations,
   bool fast_version=false);



//! Compute fitting scores for a given set of rigid transformations
/**
\brief Scores how well a set of particles fit a map
\param[in] ps       The particles to be fitted
\param[in] em_map   The density map to fit to
\param[in] rad_key  The raidus key of the particles in the rigid body
\param[in] wei_key  The weight key of the particles in the rigid body
\note the function assumes the density map holds its density
 */
IMPEMEXPORT Float compute_fitting_score(const Particles &ps,
   DensityMap *em_map,
   FloatKey rad_key=core::XYZR::get_default_radius_key(),
   FloatKey wei_key=atom::Mass::get_mass_key());

IMPEM_END_NAMESPACE
#endif  /* IMPEM_RIGID_FITTING_H */
