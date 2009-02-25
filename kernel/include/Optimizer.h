/**
 *  \file Optimizer.h     \brief Base class for all optimizers.
 *
 *  Copyright 2007-8 Sali Lab. All rights reserved.
 *
 */

#ifndef IMP_OPTIMIZER_H
#define IMP_OPTIMIZER_H

#include "config.h"
#include "base_types.h"
#include "VersionInfo.h"
#include "Object.h"
#include "utility.h"
#include "Model.h"
#include "Particle.h"
#include "Pointer.h"

#include <limits>
#include <cmath>

IMP_BEGIN_NAMESPACE

class OptimizerState;

typedef std::vector<OptimizerState*> OptimizerStates;

//! Base class for all optimizers.
/** The Optimizer maintains a list of OptimizerStates which are
    updated each time the conformation is changed.

    The optimizers have one key method Optimizer::optimize which takes
    the number of steps to perform. The optimizers can have other
    stopping conditions as appropriate.
 */
class IMPEXPORT Optimizer: public Object
{
public:
  Optimizer();
  virtual ~Optimizer();

  /** Optimize the model
      \param[in] max_steps The maximum number of steps to take.
      \return The final score.
   */
  virtual Float optimize(unsigned int max_steps) = 0;

  //! \return version and authorship information.
  virtual VersionInfo get_version_info() const = 0;

  //! Get the model being optimized
  Model *get_model() const {
    return model_.get();
  }

  //! Set the model being optimized
  /**
     \note The model is not owned by the optimizer and so is not
     deleted when the optimizer is deleted. Further, the Optimizer
     does not prevent the model from being deleted when all python
     references go away.
   */
  void set_model(Model *m) {model_=m;}

  //! Print info about the optimizer state
  /** It should end in a newline */
  virtual void show(std::ostream &out= std::cout) const {
    out << "Some optimizer" << std::endl;
  }

  IMP_LIST(public, OptimizerState, optimizer_state, OptimizerState*);

protected:
  //! Update optimizer state, should be called at each successful step
  void update_states() const ;

  //! Methods and classes to iterate through optimized attributes
  //@{
  //! An index to an optimized particle attribute
  struct FloatIndex
  {
    /**
       \todo mac gcc breaks on the protection and friends here
     */
    friend class Optimizer;
    friend class FloatIndexIterator;
    Model::ParticleConstIterator p_;
    Particle::OptimizedKeyIterator fk_;
    FloatIndex(Model::ParticleConstIterator p): p_(p){}
  public:
    FloatIndex() {}
  };


  //! An interator through the optimized attributes
  /** The value type is an FloatIndex
   */
  class FloatIndexIterator
   {
    typedef FloatIndexIterator This;
    Model::ParticleConstIterator pe_;
    mutable FloatIndex i_;

    void search_valid() const {
      while (i_.fk_ == (*i_.p_)->optimized_keys_end()) {
        if (i_.fk_ == (*i_.p_)->optimized_keys_end()) {
          ++i_.p_;
          if (i_.p_== pe_) return;
          else {
            i_.fk_= (*i_.p_)->optimized_keys_begin();
          }
        } else {
          ++i_.fk_;
        }
      }
      IMP_assert(i_.p_ != pe_, "Should have just returned");
      IMP_assert(i_.fk_ != (*i_.p_)->optimized_keys_end(),
                 "Broken iterator end");
      IMP_assert((*i_.p_)->get_is_optimized(*i_.fk_),
                   "Why did the loop end?");
    }
    void find_next() const {
      ++i_.fk_;
      search_valid();
    }
  public:
    FloatIndexIterator(Model::ParticleConstIterator pc,
                       Model::ParticleConstIterator pe): pe_(pe), i_(pc) {
      if (pc != pe) {
        i_.fk_= (*pc)->optimized_keys_begin();
        search_valid();
      }
    }
    typedef FloatIndex value_type;
    typedef FloatIndex& reference;
    typedef FloatIndex* pointer;
    typedef std::forward_iterator_tag iterator_category;
    typedef int difference_type;

    const This &operator++() {
      find_next();
      return *this;
    }
    reference operator*() const {
      IMP_assert((*i_.p_)->get_is_optimized(*i_.fk_),
                 "The iterator is broken");
      return i_;
    }
    pointer operator->() const {
      return &i_;
    }
    bool operator==(const This &o) const {
      if (i_.p_ != o.i_.p_) return false;
      if (i_.p_== pe_) return o.i_.p_ ==pe_;
      else return i_.fk_ == o.i_.fk_;
    }
    bool operator!=(const This &o) const {
      return !operator==(o);
    }
  };

  //! Iterate through the optimized attributes
  FloatIndexIterator float_indexes_begin() const {
    return FloatIndexIterator(model_->particles_begin(),
                              model_->particles_end());
  }

  FloatIndexIterator float_indexes_end() const {
    return FloatIndexIterator(model_->particles_end(),
                              model_->particles_end());
  }
  //!@}


  //! Methods for getting and setting optimized attributes
  /** Optimizers don't have to go through the particles themselves
      looking for values to optimize unless they care about special
      properties of the optimized values.
  */
  //!@{
  //! Set the value of an optimized attribute
  /** The attribute must be optimized or an ErrorException is thrown.
   */
  void set_value(FloatIndex fi, Float v) const {
    IMP_assert(fi.p_ != model_->particles_end(),
               "Out of range FloatIndex in Optimizer");
    IMP_assert((*fi.p_)->get_is_optimized(*fi.fk_),
               "Keep your mits off unoptimized attributes "
               << (*fi.p_)->get_name() << " " << *fi.fk_ << std::endl);
    (*fi.p_)->set_value(*fi.fk_, v);
  }

  //! Get the value of an optimized attribute
  Float get_value(FloatIndex fi) const {
    /* cast to const needed here to help MSVC */
    IMP_assert(static_cast<Model::ParticleConstIterator>(fi.p_)
               != model_->particles_end(),
               "Out of range FloatIndex in Optimizer");
    return (*fi.p_)->get_value(*fi.fk_);
  }

  //! Get the derivative of an optimized attribute
  Float get_derivative(FloatIndex fi) const {
    IMP_assert(fi.p_ != model_->particles_end(),
               "Out of range FloatIndex in Optimizer");
    return (*fi.p_)->get_derivative(*fi.fk_);
  }
  //!@}


  double width(FloatKey k) const {
    if (!widths_.contains(k)) {
      FloatPair w= model_->get_range(k);
      double wid=static_cast<double>(w.second)- w.first;
      if (wid > .0001) {
        //double nwid= std::pow(2, std::ceil(log2(wid)));
        widths_.insert(k, wid);
      } else {
        widths_.insert(k, 1);
      }
    }
    return widths_.get_value(k);
    //return 1.0;
  }

  //! Methods to get and set scaled optimizable values
  /** Certain optimizers benefit from having all the optimized values
      scaled to vary over a similar range. These accessors use the
      Model::get_range ranges to scale the values before returning
      them and unscale them before setting them.
  */
  //{@
  //! Set the value of an optimized attribute
  /** The attribute must be optimized or an ErrorException is thrown.
   */
  void set_scaled_value(FloatIndex fi, Float v) const {
    double wid = width(*fi.fk_);
    set_value(fi, v*wid);
  }

  //! Get the value of an optimized attribute
  double get_scaled_value(FloatIndex fi) const  {
    double uv= get_value(fi);
    double wid = width(*fi.fk_);
    return uv/wid;
  }

  //! Get the derivative of an optimized attribute
  double get_scaled_derivative(FloatIndex fi) const {
    double uv=get_derivative(fi);
    double wid= width(*fi.fk_);
    return uv*wid;
  }

  //! Clear the cache of range information. Do this at the start of optimization
  void clear_range_cache() {
    widths_.clear();
  }
  //!@}

  //! A collection of indexes
  typedef std::vector<FloatIndex> FloatIndexes;

private:
  typedef internal::AttributeTable<internal::FloatAttributeTableTraits>
   FloatTable;
  mutable FloatTable widths_;
  Pointer<Model> model_;
};

IMP_OUTPUT_OPERATOR(Optimizer);

IMP_END_NAMESPACE

#endif  /* IMP_OPTIMIZER_H */
