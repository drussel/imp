/**
 * \file  DiscreteSet.h
 * \brief Holds a discrete sampling space.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 */

#ifndef IMPDOMINO_DISCRETE_SET_H
#define IMPDOMINO_DISCRETE_SET_H

#include "IMP/Particle.h"
#include <map>
#include  <sstream>
#include "IMP/base_types.h"
#include "domino_config.h"

IMPDOMINO_BEGIN_NAMESPACE

class IMPDOMINOEXPORT DiscreteSet
{
public:
  DiscreteSet(){}
  //! Create the discrete set
  /** \param[in] atts the attributes for the states held in the set
   */
  DiscreteSet(const std::vector<FloatKey> &atts);

  //! Get all attributes sampled in the set
  /**
   */
  const std::vector<FloatKey> * get_att_keys() const {return &atts_;}
  //! Get the number of types attributes each state holds
  inline unsigned int  get_number_of_attributes()const {
    return atts_.size();
  }

  //! Add a new state to the set.
  /** \param[in] p the new state
      \exception if the new state does not have values for all of the
                 attributes of the set.
   */
  void add_state(Particle* p);
  //! Get a value of an attribute of a state
  /**
    \param[in] state_ind the index of the state
    \param[in] key       the key of the attribute
    \exception if the state_ind is out of range or if the attribute
               is not sampled in the set.
   */
  Float get_state_val(long state_ind, IMP::FloatKey key) const;

  //! Get a state
  /**
    \param[in] state_ind the index of the state
    \exception if the state_ind is out of range
   */
  Particle  * get_state(long state_ind) const;
  const Particles * get_states() const{return &states_;};

  //! Get the number of states held in the set
  inline long get_number_of_states() const { return states_.size();}

  void show(std::ostream& out=std::cout) const;
protected:
  Particles states_;
  std::vector<FloatKey> atts_;
};

IMPDOMINO_END_NAMESPACE

#endif  /* IMPDOMINO_DISCRETE_SET_H */
