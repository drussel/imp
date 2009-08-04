/**
 *  \file IMP/Restraint.h     \brief Abstract base class for all restraints.
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 *
 */

#ifndef IMP_RESTRAINT_H
#define IMP_RESTRAINT_H

#include "config.h"
#include "DerivativeAccumulator.h"
#include "Model.h"
#include "Particle.h"
#include "VersionInfo.h"
#include "Object.h"
#include "Pointer.h"
#include "log.h"
#include "utility.h"
#include "container_macros.h"

#include <vector>
#include <iostream>
#include <limits>

IMP_BEGIN_NAMESPACE

//! Sets of Particles
/** This is primarily used for representing lists of interacting particles */
typedef std::vector<Particles> ParticlesList;

class Model;

//! Abstract class for representing restraints
/** Restraints should take their score function or UnaryFunction
    as the first argument. Restraints which act on large numbers of
    particles should allow the particle list to be skipped in the
    constructor and should provide methods so that the set of particles
    can be modified after construction.

    A restraint can be added to the model multiple times or to multiple
    restraint sets in the same model.

    \note When logging is VERBOSE, restraints should print enough information
    in evaluate to reproduce the the entire flow of data in evaluate. When
    logging is TERSE the restraint should print out only a constant number of
    lines per evaluate call.

    \note Physical restraints should use the units of kcal/mol for restraint
    values and kcal/mol/A for derivatives.

    \note Restraints will print a warning message if they are destroyed
    without ever having been added to a model as this is an easy mistake
    to make. To disable this warning for a particular restraint, call
    set_was_owned(true).

    See the examples::ExampleRestraint  example for how to implement
    a simple restraint.
 */
class IMPEXPORT Restraint : public Object, public Ownable
{
public:
  Restraint(std::string name="Restraint %1%");

  //! Return the score for this restraint for the current state of the model.
  /** \param[in] accum If not NULL, use this object to accumulate partial first
                       derivatives.
      \return Current score.
   */
  virtual Float evaluate(DerivativeAccumulator *accum) = 0;

  //! Set whether the restraint is active i.e. if it should be evaluated.
  /** \param[in] is_active If true, the restraint is active.
   */
  void set_is_active(const bool is_active);

  //! Get whether the restraint is active. i.e. if it should be evaluated.
  /** \return true if the restraint is active.
   */
  bool get_is_active() const;

  //! The model the restraint is part of.
  /** \param[in] model The model.
      Restraints that want to take action when they are added to the model
      can override this method (but be sure to call Restraint::set_model()
      to set the actual model pointer).
   */
  virtual void set_model(Model* model);

  //! Return the model containing this restraint
  Model *get_model() const {
    IMP_assert(model_,
               "get_model() called before set_model()");
    return model_;
  }

  //! Return true if this particle is part of a model
  bool get_is_part_of_model() const {
    return model_;
  }

  //! Return a list of sets of particles that are restrained by this restraint
  /** This function returns a list of sets of particles that are
      interacting within this restraint. Particles can appear in more
      than one set. However, if two particles never appear in the same
      set, then changing one of the particles should not change the
      derivatives of the other particle.

      The default implementation returns a single set containing all
      particles stored in this restraint.
   */
  virtual ParticlesList get_interacting_particles() const=0;

  IMP_REF_COUNTED_DESTRUCTOR(Restraint);
private:
  /* This pointer should never be ref counted as Model has a
     pointer to this object. Not that Model is refcounted yet.
   */
  WeakPointer<Model> model_;

  /* True if restraint has not been deactivated.
     If it is not active, evaluate should not be called
   */
  bool is_active_;
};

IMP_OUTPUT_OPERATOR(Restraint);

IMP_END_NAMESPACE

#endif  /* IMP_RESTRAINT_H */
