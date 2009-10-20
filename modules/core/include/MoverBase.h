/**
 *  \file MoverBase.h    \brief A class to help implement movers.
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 *
 */

#ifndef IMPCORE_MOVER_BASE_H
#define IMPCORE_MOVER_BASE_H

#include "config.h"
#include "Mover.h"

#include <IMP/SingletonContainer.h>
#include <IMP/macros.h>

#include <vector>

IMPCORE_BEGIN_NAMESPACE

//! A class to help implement movers
/** This class helps in implementing Movers by allowing changes to be easily
    rolled back. It maintains a list of particles and a list of attributes.
    All changes to the product of those two lists will be rolled back
    when reject_move() is called.

    See NormalMover for a simple example using this class.
 */
class IMPCOREEXPORT MoverBase: public Mover
{
  std::vector<Floats> floats_;
  std::vector<Ints> ints_;
  IMP::internal::OwnerPointer<SingletonContainer> pc_;
public:
  virtual void accept_move(){}
  virtual void reject_move();

  /** This sets everything up and then calls the generate_move method.
   */
  virtual void propose_move(Float f);


  /** @name Methods to manipulate the set of controlled attributes

       Each of the attributes whose FloatKey or IntKey is added to
       the list below is controlled by the MoverBase in each particle in
       the container.
  */
  /**@{*/
  IMP_LIST(protected, FloatKey, float_key, FloatKey, FloatKeys);
  IMP_LIST(public, IntKey, int_key, IntKey, IntKeys);
  /**@}*/
  SingletonContainer* get_container() const {
    return pc_;
  }

protected:
  //! implement this method to propose a move
  /** See NormalMover for a simple example.
   */
  virtual void generate_move(Float f)=0;

  //! Get the value of a controlled attribute
  /** \param [in] i The index of the particle.
      \param [in] j The index of the attribute.
   */
  Float get_float(unsigned int i, unsigned int j) const {
    IMP_INTERNAL_CHECK(pc_->get_number_of_particles() == floats_.size(),
               "Only call get_float from within generate_proposal");
    return pc_->get_particle(i)->get_value(get_float_key(j));
  }

  //! Get an int attribute value
  /** \param [in] i The index of the particle.
      \param [in] j The index of the attribute.
   */
  Int get_int(unsigned int i, unsigned int j) const {
    IMP_INTERNAL_CHECK(pc_->get_number_of_particles() == ints_.size(),
               "Only call get_int from within generate_proposal");
    return pc_->get_particle(i)->get_value(get_int_key(j));
  }

  //! Propose a value
  /** \param[in] i The index of the particle.
      \param[in] j The index of the key
      \param[in] t The value to propose
   */
  void propose_value(unsigned int i, unsigned int j, Float t) {
    if (pc_->get_particle(i)->get_is_optimized(get_float_key(j))) {
      pc_->get_particle(i)->set_value(get_float_key(j), t);
    }
  }
  //! Propose a value
  /** \param[in] i The index of the particle.
      \param[in] j The index of the key
      \param[in] t The value to propose
   */
  void propose_value(unsigned int i, unsigned int j, Int t) {
    pc_->get_particle(i)->set_value(get_int_key(j), t);
  }

  MoverBase(SingletonContainer *sc): pc_(sc) {}
  IMP_REF_COUNTED_DESTRUCTOR(MoverBase);
};

IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_MOVER_BASE_H */
