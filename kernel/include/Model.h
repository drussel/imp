/**
 *  \file Model.h   \brief Storage of a model, its restraints,
 *                         constraints and particles.
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 *
 */

#ifndef IMP_MODEL_H
#define IMP_MODEL_H

#include "config.h"
#include "Object.h"
#include "Particle.h"
#include "internal/version_info.h"
#include "container_macros.h"
#include "base_types.h"
#include "VersionInfo.h"

#include <limits>

IMP_BEGIN_NAMESPACE

class Particle;
class Restraint;
class ScoreState;
typedef VectorOfRefCounted<Restraint*> Restraints;
typedef VectorOfRefCounted<ScoreState*> ScoreStates;


//! Class for storing model, its restraints, and particles.
/** The Model maintains a standard IMP container for each of Particle,
    ScoreState and Restraint object types.

    \note Think carefully about using the iterators over the entire set
          of Particles or Restraints. Most operations should be done using
          a user-passed set of Particles or Restraints instead.
 */
class IMPEXPORT Model: public Object
{
private:
  friend class Restraint;
  friend class Particle;
  typedef Particle::Storage ParticleStorage;

  unsigned int iteration_;
  ParticleStorage particles_;
  unsigned int next_particle_index_;
  std::map<FloatKey, FloatRange> ranges_;
  enum Stage {NOT_EVALUATING, BEFORE_EVALUATE, EVALUATE, AFTER_EVALUATE};
  Stage cur_stage_;

  Stage get_stage() const {
    return cur_stage_;
  }

  void add_particle_internal(Particle *p) {
    IMP_CHECK_OBJECT(this);
    IMP_IF_CHECK(EXPENSIVE) {
      for (ParticleStorage::const_iterator it= particles_.begin();
           it != particles_.end(); ++it) {
        IMP_check(*it != p, "Particle already in Model",
                  ValueException);
      }
    }
    particles_.push_back(p);
    p->iterator_= --particles_.end();
    p->model_= this;
    internal::ref(p);
    std::ostringstream oss;
    oss << "P" << next_particle_index_;
    p->set_name(oss.str());
    ++next_particle_index_;
  }


#if defined(SWIG)
 public:
#else
 private:
  IMP_NO_DOXYGEN(template <class T> friend void IMP::internal::unref(T*);)
#endif

  virtual ~Model();
public:
  /** Construct an empty model */
  Model();


  /** @name States

      The Model stores a list of ScoreStates which are given an
      opportunity to update the stored Particles and their internal
      state before and after the restraints are evaluated. Use the
      methods below to manipulate the list of ScoreState objects.
  */
  /**@{*/
  IMP_LIST(public, ScoreState, score_state, ScoreState*, ScoreStates);
  /**@}*/

  /** @name Restraints

      The Model scores the current configuration use the stored Restraint
      objects. Use the methods below to manipulate the list.
   */
  /**@{*/
  IMP_LIST(public, Restraint, restraint, Restraint*, Restraints);
  /**@}*/
 public:

  //! Remove the particle from this model
  /** Since particles are ref counted the object will still be valid
      objects until all references are removed, however attributes of
      removed particles cannot be changed or inspected.

      \note It is an error to remove particles from the model during
      Restraint evaluation or ScoreState evaluation. It is OK to
      remove them during OptimizerState updates, although not all
      optimizers support this yet.
  */
  void remove_particle(Particle *p) {
    IMP_OBJECT_LOG;
    IMP_CHECK_OBJECT(this);
    IMP_CHECK_OBJECT(p);
    IMP_check(p->get_model() == this,
              "The particle does not belong to this model",
              ValueException);
    IMP_LOG(VERBOSE, "Removing particle " << p->get_name()
            << std::endl);
    IMP_assert(get_stage() == Model::NOT_EVALUATING,
               "Particles cannot be removed from the model during evaluation");
    particles_.erase(p->iterator_);
    p->model_=NULL;
    internal::unref(p);
  }

  /** @name Methods to debug particles
      It is sometimes useful to inspect the list of all particles when
      debugging. These methods allow you to do that.
      \note Only use this if you really know what you are doing as
      Particles can be added to the object from many different places.
      As a result, the list of Particles should only be used for
      debugging.
   */
  /**@{*/
  unsigned int get_number_of_particles() const {
    return particles_.size();
  }
#ifdef IMP_DOXYGEN
  class ParticleInterator; class ParticleConstIterator;
#else
 typedef ParticleStorage::const_iterator ParticleConstIterator;
 typedef ParticleStorage::iterator ParticleIterator;
#endif
  ParticleIterator particles_begin() {
    return particles_.begin();
  }
  ParticleIterator particles_end() {
    return particles_.end();
  }

  ParticleConstIterator particles_begin() const {
    return particles_.begin();
  }
  ParticleConstIterator particles_end() const {
    return particles_.end();
  }
  /** @} */

  /** @name Float Attribute Ranges
      Each Float attribute has an associated range which reflect the
      range of values that it is expected to take on during optimization.
      The optimizer can use these ranges to make the optimization process
      more efficient. By default, the range estimates are simply the
      range of values for that attribute in the various particles, but
      it can be set to another value. For example, an attribute storing
      an angle should have the range set to eg (0,PI).

      The ranges are not enforced, they are just guidelines.
      @{
  */
  FloatRange get_range(FloatKey k) const;

  void set_range(FloatKey k, FloatRange range) {
    ranges_[k]=range;
  }
  /** @} */

  //! Evaluate all of the restraints in the model and return the score.
  /** \param[in] calc_derivs If true, also evaluate the first derivatives.
      \return The score.

      Evaluation proceeds as follows:
      - ScoreState::before_evaluate() is called on all ScoreStates
      - Restraint::evaluate() is called on all Restraints
      - ScoreState::after_evaluate() is called on all ScoreStates
      The sum of the Restraint::evaluate() return values is returned.
   */
  Float evaluate(bool calc_derivs);

  //! Show the model contents.
  /** \param[in] out Stream to write model description to.
   */
  void show (std::ostream& out = std::cout) const;

  //! \return version and authorship information.
  VersionInfo get_version_info() const {
    IMP_CHECK_OBJECT(this);
    return internal::version_info;
  }
};

IMP_OUTPUT_OPERATOR(Model);

#ifndef IMP_DOXYGEN
// these require Model be defined

inline void Particle::assert_values_mutable() const {
  IMP_assert(get_model()->get_stage() != Model::EVALUATE,
             "Restraints are not allowed to change attribute values during "
             << "evaluation.");
  IMP_assert(get_model()->get_stage() != Model::AFTER_EVALUATE,
             "ScoreStates are not allowed to change attribute values after "
             << "evaluation.");
}

inline void Particle::assert_can_change_optimization() const {
  IMP_assert(get_model()->get_stage() == Model::NOT_EVALUATING,
             "The set of optimized attributes cannot be changed during "
             << "evaluation.");
}

inline void Particle::assert_can_change_derivatives() const {
  IMP_assert(get_model()->get_stage() != Model::BEFORE_EVALUATE,
             "Derivatives can only be changed during restraint "
             << "evaluation and score state after evaluation calls.");
}
#endif

IMP_END_NAMESPACE

#endif  /* IMP_MODEL_H */
