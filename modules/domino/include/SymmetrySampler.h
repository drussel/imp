/**
 * \file  SymmetrySampler.h
 * \brief Sample transformations of particles while preseving N-symmetry.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 */

#ifndef IMPDOMINO_SYMMETRY_SAMPLER_H
#define IMPDOMINO_SYMMETRY_SAMPLER_H

#include "IMP/Particle.h"
#include <map>
#include  <sstream>
#include "IMP/base_types.h"
#include "domino_config.h"
#include "DiscreteSampler.h"
#include "CombState.h"
#include "TransformationDiscreteSet.h"
#include <IMP/algebra/Transformation3D.h>
#include <IMP/algebra/Cylinder3D.h>
#include <IMP/container/ListSingletonContainer.h>
IMPDOMINO_BEGIN_NAMESPACE
//! SymmetrySampler
/**
The class symmetrically sample particles.
/note We currently assume that the particles are Hierarchys,
      it should be changes to general Hierarchy.
 */
class IMPDOMINOEXPORT SymmetrySampler : public DiscreteSampler
{
public:
  //! Constructor
  /**
  /param[in] ps the particles that should obey N-symmetry ( N is the
                number of particles).
  /param[in] ts a set of transformations for the first particle
  /param[in] c  the cylinder represent the symmery axis of the particles
  /note The sampler assumes that the order of the particles in the ring
        is the order of the input particles.
  /note The sampled set of transformations for particle number i is the set of
        input transformations rotated by (360/N)*i around the symmetry axis.
  /todo consider calculating the cylinder in construction
   */
  SymmetrySampler(container::ListSingletonContainer *ps,
                  TransformationDiscreteSet *ts,
                  const algebra::Cylinder3D &c);
  void move2state(const CombState *cs);
  void populate_states_of_particles(
               container::ListSingletonContainer* particles,
               std::map<std::string, CombState *> *states) const;
  void show(std::ostream& out = std::cout) const {out<<"SymmetrySampler";}
  DiscreteSet* get_space(Particle */*p*/) const{return ts_;}
 protected:

  void reset_placement(const CombState *cs);

  algebra::Cylinder3D cyl_;
  std::map<Particle*,int> symm_deg_;
  Pointer<container::ListSingletonContainer> ps_;
  TransformationDiscreteSet *ts_;
  std::map<Particle*,algebra::Transformation3D> ref_;
};

IMPDOMINO_END_NAMESPACE

#endif  /* IMPDOMINO_SYMMETRY_SAMPLER_H */
