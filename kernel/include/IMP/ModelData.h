/**
 *  \file ModelData.h    \brief Storage for all model particle data.
 *
 *  Copyright 2007-8 Sali Lab. All rights reserved.
 *
 */

#ifndef __IMP_MODEL_DATA_H
#define __IMP_MODEL_DATA_H

#include <vector>
#include <map>

#include "IMP_config.h"
#include "base_types.h"

namespace IMP
{

class Model;
class Particle;
class DerivativeAccumulator;


//! Storage for all model particle data.
/** All data for particles is stored through indexing of data
    in this structure. Float data is assumed to be potentially
    differentiable and is stored in a structure that can accomodate
    differentiation. Int and string data is stored directly in vectors.

    \note ModelData should only be used in writing Optimizers.
 */
class IMPDLLEXPORT ModelData
{
  friend class Model;
  friend class Particle;
  friend class OptFloatIndexIterator;
  friend class DerivativeAccumulator;
  friend class std::auto_ptr<ModelData>;

  // variables
  class FloatData
  {
  public:
    Float value_;
    Float deriv_;
    int stats_index_;
    bool is_optimized_;
  };

  //! variable statistics
  /** intended for keeping track of change sizes during optimization
      for efficiency issues (e.g. updates of neighborhoods)
   */
  class Statistics
  {
  public:
    Float min_;
    Float max_;
    Float max_delta_;
    Float min_delta_;
  };
public:


  //! Add particle float attribute (assumed differentiable) to the model.
  /** The returned index can be used for obtaining and setting the attribute
      value.
      \param[in] value Initial value of the attribute.
      \return index of a new float attribute.
   */
  FloatIndex add_float(const Float value);

  //! Set particle attribute value.
  /** \param[in] idx Index of the attribute.
      \param[in] value Value the attribute should be given.
   */
  void set_value(const FloatIndex idx, const Float value);

  //! Get particle float attribute (inline).
  /** \param[in] idx Index of the particle float attribute.
      \return value of particle float attribute.
   */
  Float get_value(const FloatIndex idx) const {
    IMP_assert(idx.get_index() < float_data_.size(),
               "Out of range index requested");
    return float_data_[idx.get_index()].value_;
  }

  //! Get derivative of the given particle float attribute.
  /** \param[in] idx Index of the particle float attribute.
   */
  Float get_deriv(const FloatIndex idx) const;

  //! Indicate if the particle float attribute is to be optimized.
  /** \param[in] idx Index of the particle float attribute.
      \return True if particle float attribute is to be optimized.
   */
  bool get_is_optimized(const FloatIndex idx) const;

  //! Set whether the particle float attribute is to be optimized.
  /** \param[in] idx Index of the particle float attribute.
      \param[in] is_optimized True if particle float attribute is to be
                              optimized.
   */
  void set_is_optimized(const FloatIndex idx, bool is_optimized);

  //! Set all derivatives to zero.
  void zero_derivatives();

  //! Add particle int attribute to the model.
  /** The returned index can be used for obtaining and setting the
      attribute value.
      \param[in] value Initial value of the attribute.
      \return index of a new int attribute.
   */
  IntIndex add_int(const Int value);

  //! Set particle attribute value.
  /** \param[in] idx Index of the attribute.
      \param[in] value Value the attribute should be given.
   */
  void set_value(const IntIndex idx, const Int value);

  //! Get particle int attribute (inline).
  /** \param[in] idx Index of the particle int attribute.
      \return value of particle float attribute.
   */
  Int get_value(const IntIndex idx) const {
    IMP_assert(idx.get_index() < int_data_.size(),
               "Out of range int requested");
    return int_data_[idx.get_index()];
  }

  //! Add particle string attribute to the model.
  /** The returned index can be used for obtaining and setting the attribute
      value.
      \param[in] value Initial value of the attribute.
      \return index of a new string attribute.
   */
  StringIndex add_string(const String value);

  //! Set particle attribute value.
  /** \param[in] idx Index of the attribute.
      \param[in] value Value the attribute should be given.
   */
  void set_value(const StringIndex idx, const String value);

  //! Get particle string attribute (inline).
  /** \param[in] idx Index of the particle string attribute.
      \return value of particle string attribute.
   */
  String get_value(const StringIndex idx) const {
   IMP_assert(idx.get_index() < string_data_.size(),
               "Out of range string requested");
    return string_data_[idx.get_index()];
  }


  void show(std::ostream &out=std::cout) const;
protected:
  ModelData();
  ~ModelData();

  //! Add value to derivative (used by DerivativeAccumulator).
  /** \param[in] idx Index of the particle float attribute.
      \param[in] value Value to add to the float attribute derivative.
   */
  void add_to_deriv(const FloatIndex idx, const Float value);

  //! used by model to see if restraints need to check their particles
  bool check_particles_active() {
    return check_particles_active_;
  }
  void set_check_particles_active(bool check_particles_active) {
    check_particles_active_ = check_particles_active;
  }

  //! particle variables and attributes
  /** these are stored outside of particles to allow
      restraints to get access them directly through
      indexes rather than through particle dereferencing.
   */
  std::vector<FloatData> float_data_;

  //! See float_data_.
  std::vector<Int> int_data_;
  //! See float_data_.
  std::vector<String> string_data_;

  //! float attribute state change statistics associated with a particular
  //! name in some subset of particles
  std::map<std::string, int> stat_indexes_;
  //! See stat_indexes_.
  std::vector<Statistics> float_stats_;

  //! flag set whenever a particle is activated or deactivated
  bool check_particles_active_;
};

IMP_OUTPUT_OPERATOR(ModelData);

//! Optimizable variable iterator
/** Returns all optimizable Floats in the ModelData.
 */
class IMPDLLEXPORT OptFloatIndexIterator
{
public:
  OptFloatIndexIterator() {}

  //! Reset the iterator.
  /** After the next call to next(), get() will return the first optimizable
      Float variable.
      \param[in] model_data The model data that is being referenced.
   */
  void reset(ModelData* model_data);

  //! Move to the next optimizable Float variable.
  /** Check if another optimizable Float variable is available, and if so,
      make sure it is called by the next call to get().
      \return True if another optimizable variable is available.
   */
  bool next();

  //! Return the current available optimizable Float variable.
  /** Should only be called if next() returned True.
      \return the index of the Float variable.
   */
  FloatIndex get() const;

protected:
  int cur_;
  ModelData* model_data_;
};


} // namespace IMP

#endif  /* __IMP_MODEL_DATA_H */
