/**
 *  \file Optimizer.h     \brief Base class for all optimizers.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMP_OPTIMIZER_H
#define IMP_OPTIMIZER_H

#include "kernel_config.h"
#include "base_types.h"
#include "VersionInfo.h"
#include "Object.h"
#include "utility.h"
#include "Model.h"
#include "Particle.h"
#include "Pointer.h"
#include "OptimizerState.h"
#include <limits>
#include <cmath>

IMP_BEGIN_NAMESPACE

//! Base class for all optimizers.
/** The Optimizer maintains a list of OptimizerStates which are
    updated each time the conformation is changed.

    The optimizers have one key method Optimizer::optimize() which takes
    the number of steps to perform. The optimizers can have other
    stopping conditions as appropriate.

    A typical optimization loop proceeds by:
    - the optimizer calls Model::evaluate() to compute the score
    (and possibly the derivatives) of the
    current conformation of the Model.
    - the optimizer uses this information to update the optimizeable
    parameters of the Particles contained in the Model.

    \implementationwithoutexample{Optimizer, IMP_OPTIMIZER}
*/
class IMPEXPORT Optimizer: public Object
{
 public:
  Optimizer(Model *m= NULL, std::string name="Optimizer %1%");

  /** Optimize the model

      \param[in] max_steps The maximum number of iterations of the
      optimizer to perform. Increasing this number will generally make
      the optimizer spend more time searching for a solution, but
      beyond that, the details of what changes will vary.

      \return The final score.
  */
  double optimize(unsigned int max_steps);

#ifndef IMP_DOXYGEN
  /** \name Score threshold
      Optimizers can be set to stop if they achieve a score below
      a score threshold. This is useful so that they don't spend time
      improving already very good solutions.
      @{
  */
  void set_score_threshold(double s) {min_score_=s;}
  double get_score_threshold() const {return min_score_;}
  /** @} */
#endif

  /** Optimization can be stopped if all the thresholds in the Model are
      satisfied. */
  void set_stop_on_good_score(bool tf) {
    stop_on_good_score_=tf;
  }
  bool get_stop_on_good_score() const {
    return stop_on_good_score_;
  }

  //! Get the model being optimized
  Model *get_model() const {
    return model_.get();
  }

  //! Set the model being optimized
  /**
     \note The model is not owned by the optimizer and so is not
     deleted when the optimizer is deleted. Further, the Optimizer
     does not prevent the model from being deleted when all Python
     references go away.
  */
  void set_model(Model *m) {model_=m;}

  //! Print info about the optimizer state
  /** It should end in a newline */
  virtual void show(std::ostream &out= std::cout) const {
    out << "Some optimizer" << std::endl;
  }

  /** @name States

      The stored OptimizerState objects are updated each time the
      Optimizer decides to accept a new configuration of the Model.
      To manipulate the list of optimizer states use the methods below.
  */
  /**@{*/
  IMP_LIST(public, OptimizerState, optimizer_state, OptimizerState*,
           OptimizerStates);
  /**@}*/


  IMP_REF_COUNTED_NONTRIVIAL_DESTRUCTOR(Optimizer);

 protected:
  //! override this function to do actual optimization
  virtual double do_optimize(unsigned int ns) =0;
  //! Update optimizer state, should be called at each successful step
  void update_states() const ;

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
    std::string get_string() const {
      return (*p_)->get_name() + ": " + fk_->get_string();
    }
  };


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
      IMP_INTERNAL_CHECK(i_.p_ != pe_, "Should have just returned");
      IMP_INTERNAL_CHECK(i_.fk_ != (*i_.p_)->optimized_keys_end(),
                         "Broken iterator end");
      IMP_INTERNAL_CHECK((*i_.p_)->get_is_optimized(*i_.fk_),
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
    This operator++(int) {
      This ret=*this;
      find_next();
      return ret;
    }
    reference operator*() const {
      IMP_INTERNAL_CHECK((*i_.p_)->get_is_optimized(*i_.fk_),
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


  /** @name Methods for getting and setting optimized attributes
      Optimizers don't have to go through the particles themselves
      looking for values to optimize unless they care about special
      properties of the optimized values. Instead they can iterate
      through the list of optimized attributes, each of which is
      identified by a FloatIndex. With these FloatIndex objects
      they can get and set the values and derivatives as needed.
  */
  //!@{

  FloatIndexIterator float_indexes_begin() const {
    return FloatIndexIterator(model_->particles_begin(),
                              model_->particles_end());
  }

  FloatIndexIterator float_indexes_end() const {
    return FloatIndexIterator(model_->particles_end(),
                              model_->particles_end());
  }

  void set_value(FloatIndex fi, Float v) const {
    IMP_INTERNAL_CHECK(fi.p_ != model_->particles_end(),
                       "Out of range FloatIndex in Optimizer");
    IMP_INTERNAL_CHECK((*fi.p_)->get_is_optimized(*fi.fk_),
                       "Keep your mits off unoptimized attributes "
                       << (*fi.p_)->get_name() << " " << *fi.fk_ << std::endl);
    (*fi.p_)->set_value(*fi.fk_, v);
  }

  Float get_value(FloatIndex fi) const {
    /* cast to const needed here to help MSVC */
    IMP_INTERNAL_CHECK(static_cast<Model::ParticleConstIterator>(fi.p_)
                       != model_->particles_end(),
                       "Out of range FloatIndex in Optimizer");
    return (*fi.p_)->get_value(*fi.fk_);
  }

  Float get_derivative(FloatIndex fi) const {
    IMP_INTERNAL_CHECK(fi.p_ != model_->particles_end(),
                       "Out of range FloatIndex in Optimizer");
    return (*fi.p_)->get_derivative(*fi.fk_);
  }

  //!@}

  typedef std::vector<FloatIndex> FloatIndexes;

  double width(FloatKey k) const {
    if (!widths_.fits(k.get_index())
        || !FloatTable::Traits::get_is_valid(widths_.get(k.get_index())) ) {
      FloatRange w= model_->get_range(k);
      double wid=static_cast<double>(w.second)- w.first;
      if (wid > .0001) {
        //double nwid= std::pow(2, std::ceil(log2(wid)));
        widths_.add(k.get_index(), wid);
      } else {
        widths_.add(k.get_index(), 1);
      }
    }
    return widths_.get(k.get_index());
    //return 1.0;
  }

  /** @name Methods to get and set scaled optimizable values
      Certain optimizers benefit from having all the optimized values
      scaled to vary over a similar range. These accessors use the
      Model::get_range ranges to scale the values before returning
      them and unscale them before setting them.
  */
  //{@
  void set_scaled_value(FloatIndex fi, Float v) const {
    double wid = width(*fi.fk_);
    set_value(fi, v*wid);
  }

  double get_scaled_value(FloatIndex fi) const  {
    double uv= get_value(fi);
    double wid = width(*fi.fk_);
    return uv/wid;
  }

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

 private:
  static void set_optimizer_state_optimizer(OptimizerState *os, Optimizer *o);

  typedef internal::VectorStorage<internal::FloatAttributeTableTraits>
    FloatTable;
  mutable FloatTable widths_;
  WeakPointer<Model> model_;
  double min_score_;
  bool stop_on_good_score_;
};


//! Save and restore the set of optimized attributes for a set of particles
class SaveOptimizeds {
  ParticlesTemp pt_;
  std::vector<internal::ParticleStorage::OptimizedTable> saved_;
public:
  IMP_RAII(SaveOptimizeds, (const ParticlesTemp &pt),{},
           {
             pt_=pt;
             saved_= std::vector<internal::ParticleStorage::OptimizedTable>
               (pt_.size());
             for (unsigned int i=0; i< pt_.size(); ++i) {
               saved_[i]= pt_[i]->ps_->optimizeds_;
             }
           },
           {
             for (unsigned int i=0; i< pt_.size(); ++i) {
               pt_[i]->ps_->optimizeds_= saved_[i];
             }
           });
};

IMP_OBJECTS(Optimizer,Optimizers);

IMP_END_NAMESPACE

#endif  /* IMP_OPTIMIZER_H */
