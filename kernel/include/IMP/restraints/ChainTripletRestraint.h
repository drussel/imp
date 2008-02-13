/**
 *  \file ChainTripletRestraint.h   
 *   \brief Restraint triplets of particles in chains.
 *
 *  Copyright 2007-8 Sali Lab. All rights reserved.
 *
 */

#ifndef __IMP_CHAIN_TRIPLET_RESTRAINT_H
#define __IMP_CHAIN_TRIPLET_RESTRAINT_H

#include "../IMP_config.h"
#include "../Restraint.h"
#include <vector>

namespace IMP
{

class TripletScore;

//! Restrain each three consecutive particles in each chain.
/** \ingroup restraint
 */
class IMPDLLEXPORT ChainTripletRestraint : public Restraint
{
public:
  //! Create the triplet restraint.
  /** \param[in] trip_score Triplet score to apply.
   */
  ChainTripletRestraint(TripletScore* trip_score);
  virtual ~ChainTripletRestraint(){}

  IMP_RESTRAINT("0.5", "Daniel Russel");

  //! Add a chain of particles
  /** Each three successive particles are restrained.
   */
  void add_chain(const Particles &ps);

  //! Clear all the stored chains
  void clear_chains();

protected:
  std::auto_ptr<TripletScore> ts_;
  std::vector<unsigned int> chain_splits_;
};

} // namespace IMP

#endif  /* __IMP_CHAIN_TRIPLET_RESTRAINT_H */
