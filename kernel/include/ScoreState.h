/**
 *  \file ScoreState.h   \brief Shared score state.
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 *
 */

#ifndef IMP_SCORE_STATE_H
#define IMP_SCORE_STATE_H

#include "config.h"
#include "RefCounted.h"
#include "Pointer.h"
#include "Model.h"
#include "DerivativeAccumulator.h"
#include "VersionInfo.h"
#include "utility.h"
#include "Interaction.h"

#include <iostream>

IMP_BEGIN_NAMESPACE

class Model;

//! Shared score state.
/** ScoreStates should be used to maintain invariants of the Model.
    Examples include
    - ensuring that a particular PairContainer contains all pairs
    of particles within a certain distance of one another.
    - ensuring that a particular set of particles moves as a rigid
    body
    - ensuring that the forces which act on a centroid are propagated
    to the constituent particles.

    Do do this, ScoreStates have two methods which are called during
    the Model::evaluate() function
    - before_evaluate()
    - after_evaluate()

    ScoreStates can change the state of particles and restraints.
    However, optimizers may not pick up new particles or changes
    to whether particular attributes are optimized or not.

    The result of a ScoreState computation may be needed by several
    other ScoreStates or restraints. To ensure that a given ScoreState
    is updated when needed and only updated once per Model::evaluate()
    call, each call to Model::evaluate() is assigned a unique ID which
    is passed to the ScoreState. The ScoreState base uses this ID to
    make sure that state is only updated once. To use this projection
    mechanism, inherit from ScoreState and provide implementations of
    do_before_evaluate() and do_after_evaluate(). Or, better yet, use
    the IMP_SCORESTATE macro.

    \note When logging is VERBOSE, state should print enough information
    in evaluate to reproduce the the entire flow of data in update. When
    logging is TERSE the restraint should print out only a constant number
    of lines per update call.

    Implementors should see IMP_SCORE_STATE().
 */
class IMPEXPORT ScoreState : public Interaction
{
  friend class Model;
  void set_model(Model* model);

public:
  ScoreState(std::string name=std::string());

  // Update if needed
  /* The protected do_before_evaluate method will be called if the iteration
      count has not yet been seen.
   */
  void before_evaluate(unsigned int iteration);

  // Force update of the structure.
  void before_evaluate();

  // Do post evaluation work if needed
  void after_evaluate(unsigned int iteration,
                      DerivativeAccumulator *accpt);

  //! Force update of the structure
  void after_evaluate(DerivativeAccumulator *accpt);

  //! Get the name of the state
  const std::string& get_name() const {
    return name_;
  }
  //! Set the name of the state
  void set_name(const std::string &name) {
    name_=name;
  }

  //! return the stored model data
  Model *get_model() const {
    IMP_INTERNAL_CHECK(model_,
               "Must call set_model before get_model on state");
    return model_.get();
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

  //! Get the current iteration count value.
  /** The value is updated before update() is called
   */
  unsigned int get_before_evaluate_iteration() const {
    return update_iteration_;
  }

  //! Get the current after_evaluate iteration count value.
  /** The value is updated before after_evaluate() is called
   */
  unsigned int get_after_evaluate_iteration() const {
    return after_iteration_;
  }

  IMP_REF_COUNTED_DESTRUCTOR(ScoreState);

 private:

  unsigned int update_iteration_;
  unsigned int after_iteration_;
  // all of the particle data
  WeakPointer<Model> model_;
  std::string name_;
};

IMP_OUTPUT_OPERATOR(ScoreState);

//! Removes the ScoreState when the RIAA object is destroyed
class ScoreStatePointer: public RAII {
  Pointer<ScoreState> ss_;
public:
  IMP_RAII(ScoreStatePointer, (ScoreState *ss, Model *m),, {
      ss_=ss;
      m->add_score_state(ss);
    }, {
      ss_->get_model()->remove_score_state(ss_);
      ss_=NULL;
    });
};


IMP_END_NAMESPACE

#endif  /* IMP_SCORE_STATE_H */
