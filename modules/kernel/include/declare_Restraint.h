/**
 *  \file IMP/declare_Restraint.h
 *  \brief Abstract base class for all restraints.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPKERNEL_DECLARE_RESTRAINT_H
#define IMPKERNEL_DECLARE_RESTRAINT_H

#include "kernel_config.h"
#include "ModelObject.h"
#include "model_object_macros.h"
#include "constants.h"
#include <IMP/base/tracking.h>
#include <IMP/base/deprecation_macros.h>

IMP_BEGIN_NAMESPACE
class DerivativeAccumulator;

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
    set_was_used(true).

    \headerfile Restraint.h "IMP/Restraint.h"

    \implementation{Restraint, IMP_RESTRAINT, IMP::example::ExampleRestraint}
 */
class IMPEXPORT Restraint : public ModelObject
{
public:
  /** Create a restraint and register it with the model. The restraint is
      not added to implicit scoring function in the Model.*/
  Restraint(Model *m, std::string name);
#ifndef IMP_DOXYGEN
  Restraint(std::string name="Restraint %1%");
#endif

  //! Return the score for this restraint for the current state of the model.
  /** \return Current score.

      This method is equivalent to calling:
      \code
      model->evaluate(RestraintsTemp(1,this), calc_derivs)
      \endcode
   */
  double evaluate(bool calc_derivs) const;


  //! See Model::evaluate_if_good()
  double evaluate_if_good(bool calc_derivatives) const;

  //! See Model::evaluate_with_maximum()
  double evaluate_if_below(bool calc_derivatives, double max) const;

  /** \name Evaluation implementation
      These methods are called in order to perform the actual restraint
      scoring. The restraints should assume that all appropriate ScoreState
      objects have been updated and so that the input particles and containers
      are up to date. The returned score should be the unweighted score.

      \note These functions probably should be called \c do_evaluate, but
      were grandfathered in.
      \note Although the returned score is unweighted, the DerivativeAccumulator
      passed in had better be properly weighted.
      @{
  */
  /** Return the unweighted score for the restraint.*/
  virtual double unprotected_evaluate(DerivativeAccumulator *) const=0;
  /** The function calling this will treat any score >= get_maximum_score
      as bad and so can return early as soon as such a situation is found.*/
  virtual double unprotected_evaluate_if_good(DerivativeAccumulator *da,
                                              double max) const {
    IMP_UNUSED(max);
    return unprotected_evaluate(da);
  }

  /** The function calling this will treat any score >= max as bad.*/
  virtual double unprotected_evaluate_if_below(DerivativeAccumulator *da,
                                               double max) const {
    IMP_UNUSED(max);
    return unprotected_evaluate(da);
  }
  /** @} */

  //! Decompose this restraint into constituent terms
  /** Given the set of input particles, decompose the restraint into as
      simple parts as possible. For many restraints, the simplest
      part is simply the restraint itself.

      If a restraint can be decomposed, it should return a
      RestraintSet so that the maximum score and weight can be
      passed properly.

      The restraints returned have had set_model() called and so can
      be evaluated.
   */
  Restraint* create_decomposition() const;

  //! Decompose this restraint into constituent terms for the current conf
  /** Return a decomposition that is value for the current conformation,
      but will not necessarily be valid if any of the particles are
      changed. This is the same as create_decomposition() for
      non-conditional restraints.

      The restraints returned have had set_model() called and so can be
      evaluated.
   */
  Restraint* create_current_decomposition() const;


  /** \name Weights
      Each restraint's contribution to the model score is weighted. The
      total weight for the restraint is the some over all the paths containing
      it. That is, if a restraint is in a RestraintSet with weight .5 and
      another with weight 2, and the restaint itself has weight 3, then the
      total weight of the restraint is \f$.5 \cdot 3 + 2 \cdot 3 = 7.5  \f$.
      @{
  */
  void set_weight(Float weight);
  Float get_weight() const { return weight_; }
  /** @} */
  /** \name Filtering
      We are typically only interested in "good" conformations of
      the model. These are described by specifying maximum scores
      per restraint and for the whole model. Samplers, optimizers
      etc are free to ignore configurations they encounter which
      go outside these bounds.

      \note The maximum score is for the unweighted restraint.
       That is, the restraint evaluation is bad if the value
       is greater than the maximum score divided by the weight.
      @{
  */
  double get_maximum_score() const {
    return max_;
  }
  void set_maximum_score(double s);
  /** @} */

#if !defined(SWIG) && !defined(IMP_DOXYGEN)
  /** This method cannot be implemented in python due to memory
      management issues (and the question of why you would ever
      want to).
   */
  virtual ScoringFunction *create_scoring_function(double weight=1.0,
                                                   double max
                                                   = NO_MAX) const;
#endif
#if !defined(IMP_DOXYGEN)
  void set_last_score(double s) { last_score_=s;}
#endif

  /** Return the (unweighted) score for this restraint last time it was
      evaluated.
      \note If some sort of special evaluation (eg Model::evaluate_if_good())
      was the last call, the score, if larger than the max, is not accurate.
   */
  double get_last_score() const {return last_score_;}
  /** Return whether this restraint violated it maximum last time it was
      evaluated.
   */
  bool get_was_good() const {return last_score_ < max_;}

#ifdef IMP_USE_DEPRECATED
  /** \deprecated use get_inputs() instead.*/
  IMP_DEPRECATED_WARN ParticlesTemp get_input_particles() const;
  /** \deprecated use get_inputs() instead.*/
  IMP_DEPRECATED_WARN ContainersTemp get_input_containers() const;
#endif

  IMP_REF_COUNTED_DESTRUCTOR(Restraint);
  /** A Restraint should override this if they want to decompose themselves
      for domino and other purposes. The returned restraints will be made
      in to a RestraintSet, if needed and the weight and maximum score
      set for the restraint set.
  */
  IMP_PROTECTED_METHOD(virtual Restraints, do_create_decomposition, (), const, {
    return Restraints(1, const_cast<Restraint*>(this));
    })
  /** A Restraint should override this if they want to decompose themselves
      for display and other purposes. The returned restraints will be made
      in to a RestraintSet, if needed and the weight and maximum score
      set for the restraint set.

      The returned restraints should be only the non-zero terms and should
      have their last scores set appropriately;
   */
    IMP_PROTECTED_METHOD(virtual Restraints, do_create_current_decomposition,
                         (), const, {
                           return do_create_decomposition();
                         });
  IMP_IMPLEMENT_INLINE(
  void do_update_dependencies(const DependencyGraph &,
                              const DependencyGraphVertexIndex &), {});
  IMP_IMPLEMENT_INLINE(ModelObjectsTemp do_get_outputs() const, {
      return ModelObjectsTemp();
    });
 private:
  friend class Model;
  friend class ScoringFunction;
  double weight_;
  double max_;
#if !defined(IMP_DOXYGEN) && !defined(SWIG)
 public:
  // data cached by the model
  double last_score_;
#endif
};

IMP_END_NAMESPACE

#endif  /* IMPKERNEL_DECLARE_RESTRAINT_H */
