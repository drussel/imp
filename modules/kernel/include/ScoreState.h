/**
 *  \file ScoreState.h   \brief Shared score state.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMP_SCORE_STATE_H
#define IMP_SCORE_STATE_H

#include "kernel_config.h"
#include "RefCounted.h"
#include "Pointer.h"
#include "DerivativeAccumulator.h"
#include "VersionInfo.h"
#include "utility.h"
#include "container_base.h"

#include <iostream>

IMP_BEGIN_NAMESPACE

class ScoreState;
IMP_OBJECTS(ScoreState,ScoreStates);


//! Shared score state.
/** ScoreStates allow code to be injected before and after the restraint
    evaluation process. Such code can be used to, for example:
    - log the optimization process
    - maintain constraints (see Constraint)

    ScoreStates have two methods which are called during
    the Model::evaluate() function
    - before_evaluate() which is allowed to change the contents of
    containers and the value of attributes of particles and
    - after_evaluate() which can change particle derivatives

    \note The functions Interaction::get_input_particles() and
    Interaction::get_output_particles() should return the input and
    output respectively for the before_evaluate() call. The after_evaluate()
    call must have input particles chosen from among the union of the
    input and output sets for the before call and output particles chosen
    from among the inputs of the before call.

    \implementationwithoutexample{ScoreState, IMP_SCORE_STATE}
 */
class IMPEXPORT ScoreState : public Object
{
  friend class Model;
#ifndef _MSC_VER
 protected:
#else
 public:
#endif
  /** Override this method to take action when the score stated is added to
      a model. */
  virtual void set_model(Model* model);

public:
  ScoreState(std::string name="ScoreState %1%");

  // Force update of the structure.
  void before_evaluate();

  // Do post evaluation work if needed
  void after_evaluate(DerivativeAccumulator *accpt);

  //! \return the stored model data
  Model *get_model() const {
    IMP_INTERNAL_CHECK(model_,
               "Must call set_model before get_model on state");
    return model_.get();
  }

  bool get_has_model() const {
    return model_;
  }

  /** \name Interactions
      Certain sorts of operations, such as evaluation of restraints in
      isolation, benefit from being able to determine which containers
      and particles are needed by which restraints.

      Input and output particles are ones whose attributes are read.
      @{
  */
  virtual ContainersTemp get_input_containers() const=0;
  virtual ContainersTemp get_output_containers() const=0;
  virtual ParticlesTemp get_input_particles() const=0;
  virtual ParticlesTemp get_output_particles() const=0;
  /** @} */

  /** \brief For python, cast a generic Object to this type. Throw a
      ValueException of object is not the right type.*/
  static ScoreState* get_from(Object *o) {
    return object_cast<ScoreState>(o);
  }

  virtual ScoreStates create_decomposition() const {
    return ScoreStates(1, const_cast<ScoreState*>(this));
  }

protected:
  // Update the state given the current state of the model.
  /* This is also called prior to every calculation of the model score.
      It should be implemented by ScoreStates in order to provide functionality.

      \note This can't have the same name as the public function due to the
      way C++ handles overloading and name lookups--if only one is implemented
      in the child class it will only find that one.
   */
  virtual void do_before_evaluate() = 0;

  // Do any necessary updates after the model score is calculated.
  /* \param[in] accpt The object used to scale derivatives in the score
                       calculation, or NULL if derivatives were not requested.
   */
  virtual void do_after_evaluate(DerivativeAccumulator *accpt)=0;

  IMP_REF_COUNTED_DESTRUCTOR(ScoreState);

 private:
  // all of the particle data
  WeakPointer<Model> model_;
};


IMP_END_NAMESPACE


#endif  /* IMP_SCORE_STATE_H */
