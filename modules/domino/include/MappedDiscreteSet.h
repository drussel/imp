/**
 * \file  MappedDiscreteSet.h
 * \brief Holds a discrete sampling space.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 */

#ifndef IMPDOMINO_MAPPED_DISCRETE_SET_H
#define IMPDOMINO_MAPPED_DISCRETE_SET_H

#include "IMP/Particle.h"
#include <map>
#include  <sstream>
#include "IMP/base_types.h"
#include "domino_config.h"
#include "DiscreteSet.h"
#include <IMP/container/ListSingletonContainer.h>

IMPDOMINO_BEGIN_NAMESPACE

// it is declared in several places
#ifndef IMP_SWIG
//! The key for the string Domino uses as a unique index
IMPDOMINOEXPORT StringKey node_name_key();
#endif

//! MappedDiscreteSet
class IMPDOMINOEXPORT MappedDiscreteSet : public DiscreteSet
{
public:
  //! Constructor
  MappedDiscreteSet();
  //! Constructor
  /**
  /param[in] ps_target particles to be mapped on a discrete set
   */
  MappedDiscreteSet(SingletonContainer *ps_target);
  //! Create the discrete set
  /** \param[in] ps_target the set of particles to be mapped on a
      discrete set
      \param[in] atts the attributes for the states held in the set
   */
  MappedDiscreteSet(SingletonContainer *ps_target,
                    FloatKeys atts);

  //! Add a new state to the set.
  /** \exception if the new state does not have values for all of the
                 attributes of the set.
      \param[in] sampled_p The sampled particle
      \param[in] state     A state the sampled particle can have.
   */
  void add_mapped_state(Particle* sampled_p,Particle *state);
  //! Get a state
  /**
    \param[in] state_ind the index of the state
    \param[in] p_target
    \exception if the state_ind is out of range
   */
  Particle * get_mapped_state(Particle *p_target,long state_ind) const;

  //! Get a value of an attribute of a state
  /**
    \param[in] p_target the target particle
    \param[in] state_ind the index of the state
    \param[in] key       the key of the attribute
    \exception if the state_ind is out of range or if the attribute
               is not sampled in the set.
   */
  Float get_mapped_state_val(Particle* p_target,
                        long state_ind, IMP::FloatKey key) const;

  //! Get the number of states held in the set
  long get_number_of_mapped_states(Particle *p_target) const;

  //! Check if each particle has mapped states
  bool is_valid() const;

  void show(std::ostream& out=std::cout) const;
  //get the particles the discrete set covers
  Particles get_particles() const {
    Particles ps;
    for(std::map<Particle *, Particles >::const_iterator it=
         states_map_.begin(); it != states_map_.end(); it++) {
      ps.push_back(it->first);
    }
    return ps;
  }
protected:
  std::map<Particle *, Particles> states_map_;
};

IMPDOMINO_END_NAMESPACE

#endif  /* IMPDOMINO_MAPPED_DISCRETE_SET_H */
