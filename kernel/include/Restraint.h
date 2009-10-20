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
#include "Interaction.h"

#include <vector>
#include <iostream>
#include <limits>

IMP_BEGIN_NAMESPACE


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
    a simple restraint and see IMP_RESTRAINT().
 */
class IMPEXPORT Restraint : public Interaction
{
public:
  Restraint(std::string name="Restraint %1%");

  //! Return the score for this restraint for the current state of the model.
  /** \param[in] accum If not NULL, use this object to accumulate partial first
                       derivatives.
      \return Current score.

      This method will update the ScoreStates in the model if necessary.
   */
  double evaluate(DerivativeAccumulator *accum) const;

  /** \name Incremental Evaluation

      When optimizers move the particles a few at a time, scoring can
      be made more efficient by incremental score evaluation. To do
      so, a Restraint must implement Restraint::get_is_incremental()
      so that it returns \c true and implement
      Restraint::incremental_evaluate() to do the following

      - the return value should be the total score given the new
        conformation

      - for a Particle \c p, the sum of the derivatives of
        p->get_prechange_particle()

      and \c p should be equal to the difference in derivatives
      between the current and prior conformations. This is most easily
      done by accumulating the negative of the prior derivative in
      p->get_prechange_particle() and the new derivative in p, for any
      particle touched.

      @{
  */
  //! Return true if the incremental_evaluate() function is implemented
  virtual bool get_is_incremental() const {return false;}
  //! Return the restraint score evaluated in an incremental manner
  /** This method will update the ScoreStates in the model if necessary.
  */
  double incremental_evaluate(DerivativeAccumulator *) const;

  /** @} */

  /** \brief The restraint can override this in order to take action
      when added to a Model

      Users should generally not call this method directly, instead
      they should just used Model::add_restraint() to add the
      restraint to the Model.

      Restraints
      that want to take action when they are added to the model can
      override this method (but be sure to call Restraint::set_model()
      to set the actual model pointer).
   */
  virtual void set_model(Model* model);

  //! Return the model containing this restraint
  Model *get_model() const {
    IMP_INTERNAL_CHECK(model_,
               "get_model() called before set_model()");
    return model_;
  }

  //! Return true if this particle is part of a model
  bool get_is_part_of_model() const {
    return model_;
  }

#ifndef IMP_DOXYGEN
  virtual double
    unprotected_incremental_evaluate(DerivativeAccumulator *) const {
    IMP_FAILURE(get_name() << " does not support incremental evaluation.",
                ErrorException);
    return 0;
  }
  virtual double unprotected_evaluate(DerivativeAccumulator *) const=0;

#endif

  //! Restraints cannot update other objects
  ObjectsTemp get_output_objects() const {
    return ObjectsTemp();
  }
  //! Restraints cannot update other particles
  ParticlesTemp get_output_particles() const {
    return ParticlesTemp();
  }

  IMP_REF_COUNTED_DESTRUCTOR(Restraint);
private:
  /* This pointer should never be ref counted as Model has a
     pointer to this object. Not that Model is refcounted yet.
   */
  WeakPointer<Model> model_;
};

IMP_OUTPUT_OPERATOR(Restraint);

IMP_END_NAMESPACE

#endif  /* IMP_RESTRAINT_H */
