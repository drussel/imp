/**
 *  \file Em2DRestraint.h
  *  \brief A restraint to score the fitness of a model to a set of EM images
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPEM2D_EM_2DRESTRAINT_H
#define IMPEM2D_EM_2DRESTRAINT_H

#include "IMP/em2d/em2d_config.h"
#include "IMP/em2d/project.h"
#include "IMP/em2d/ProjectionFinder.h"
#include "IMP/em2d/ProjectionMask.h"
#include "IMP/em2d/Image.h"
#include "IMP/em2d/scores2D.h"
#include "IMP/em2d/RegistrationResult.h"
#include <IMP/Restraint.h>
#include <IMP/log.h>
#include <IMP/SingletonContainer.h>

IMPEM2D_BEGIN_NAMESPACE






//! This restraint ensures that a set of particles are similar to a set
//! of EM images
/*!
  This restraint generates projections of the model that
  are compared with the EM images. The projections are generated from
  the radius of its particles. In the case of atoms, the radius is generated
  automatically. For other particles the radius has to be provided.
*/
class IMPEM2DEXPORT Em2DRestraint : public Restraint
{
  // IMP::Objects should be stored using Pointer objects
  //    to make sure that they are reference counted properly.

  //! SingletonContainer to store the particles that are restrained
  Pointer<SingletonContainer> particles_container_;
  // mutable because it has to change to get projections while evaluating
//  mutable ProjectionFinder finder_;
  mutable Pointer<ProjectionFinder> finder_;

  //! Projection Masks to fast model projection
  em2d::Images em_images_;
  em2d::Images em_variances_;
  unsigned int number_of_optimized_projections_;
  bool fast_optimization_mode_;
  bool only_coarse_registration_;
  Em2DRestraintParameters params_;
public:

  //! Create the restraint.
  Em2DRestraint() {};

  //! This function initializes the restraint with the required parameters
  /*!
    \param[in] apix Angstroms per pixel in the images
    \param[in] resolution resolution to use when generating projections
    \param[in] n_projections number of projections to generate to perform
               the initial coarse registration.
  */
  void setup(ScoreFunction *score_function,
             const Em2DRestraintParameters &params) {
  params_ = params;
  finder_ = new ProjectionFinder;
  finder_->setup(score_function, params);
  fast_optimization_mode_ = false;
  only_coarse_registration_ = false;
}



  //! Sets the particles  that should correspond to the EM images.
  void set_particles(SingletonContainer *particles_container);

  //! Sets the EM images to use as restraints
  void set_images(const em2d::Images em_images);

  /*! Sets the variance images of EM images. This is useful if the
      image is a class average
      Note: Not all the scoring function use this
      data
  */
  void set_variance_images(const em2d::Images variance_images);

  /*! Sets fast mode for computing the restraint. This mode only makes sense
      it the set_coarse_registration_mode option is false. This option only
      optimizes some coarse results (those given by the argument)
      to get the refined value.
      This option is very fast compared to a full optimization, and almost
      always is a good idea to use it with 1-5 results.
      This mode is still significantly slow compared to
      set_coarse_registration_mode(), but the values optimized are optimum.
  */
  void set_fast_mode(unsigned int n);

  /*! If the value provided to this function is true, restraint operates
      only using a coarse registration scheme (maximizing the CCC).
      This option (which is set to false by default) can speed the evaluation
      of the restraint by a order of magnitude, but the values obtained are
      not going to be optimum. You could wnat to activate it at the beginning of
      an optimization.
  */
  void set_coarse_registration_mode(bool opt) {
    if(opt) only_coarse_registration_ = true;
  }

  RegistrationResults get_registration_results() const {
    return finder_->get_registration_results();
  }

  IMP_RESTRAINT(Em2DRestraint);
};

IMP_OBJECTS(Em2DRestraint,Em2DRestraints);



//! Dummy restraint between two particles
/*!
    The restraint does do anything and returns 0 as a value. Use this
    restraint to force two particles to be considered by DOMINO as belonging
    to a restraint. This trick can be useful when building the merge tree for
    DOMINO, as can add branches that consider pairs of particles before entire
    subsets.
    NOTE: Using this restraint for DOMINO requires a fairly good knowledge of
        the works of merge trees.
*/

class IMPEM2DEXPORT DummyRestraint: public Restraint {
protected:
  Pointer<Particle> p0_;
  Pointer<Particle> p1_;

public:

  DummyRestraint(IMP::Particle *p, IMP::Particle *q) {
    p0_ = p;
    p1_ = q;
  }


  void show(std::ostream &out = std::cout) const {
    out << "Dummy Restraint" << std::endl;
  }

  IMP_RESTRAINT(DummyRestraint);

};
IMP_OBJECTS(DummyRestraint,DummyRestraints);



//! Dummy restraint for a set of particles. Same use as DummyRestraint
/*!
  \param[in]
*/

class IMPEM2DEXPORT ParticlesDummyRestraint: public Restraint {
protected:
  Pointer<SingletonContainer> container_;

public:

  ParticlesDummyRestraint(SingletonContainer *sc) {
    container_ = sc;
  }


  void show(std::ostream &out = std::cout) const {
    std::cout << "Particles Restraint" << std::endl;
  }

  IMP_RESTRAINT(ParticlesDummyRestraint);

};
IMP_OBJECTS(ParticlesDummyRestraint, ParticlesDummyRestraints);



IMPEM2D_END_NAMESPACE

#endif  /* IMPEM2D_EM_2DRESTRAINT_H */
