/**
 *  \file core_macros.h    \brief Various important macros
 *                           for implementing decorators.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPCORE_MACROS_H
#define IMPCORE_MACROS_H

/** Helper macro for implementing IMP::core::Mover. In
    addition to the IMP_OBJECT methods, it declares
    - IMP::core::Mover::propose_move()
    - IMP::core::Mover::accept_move()
    - IMP::core::Mover::reject_move()
 */
#define IMP_MOVER(Name)                                 \
  virtual void propose_move(Float size);                \
  virtual void accept_move();                           \
  virtual void reject_move();                           \
  IMP_OBJECT(Name)

/** Helper macro for implementing
    IMP::core::ClosePairsFinder objects. In addition to the
    IMP_OBJECT methods it declares:
    - IMP::core::get_close_pairs()
*/
#define IMP_CLOSE_PAIRS_FINDER(Name)                                    \
  ParticlePairsTemp get_close_pairs(SingletonContainer *pc) const;      \
  ParticlePairsTemp get_close_pairs(SingletonContainer *pca,            \
                                    SingletonContainer *pcb) const;     \
  ParticlesTemp get_input_particles(SingletonContainer *pc) const;      \
  ParticlesTemp get_input_particles(SingletonContainer *a,              \
                                   SingletonContainer *b) const;        \
  ContainersTemp get_input_containers(SingletonContainer *pc) const;    \
  ContainersTemp get_input_containers(SingletonContainer *a,            \
                                   SingletonContainer *b) const;        \
  IntPairs get_close_pairs(const algebra::BoundingBox3Ds &bbs) const;   \
  IntPairs get_close_pairs(const algebra::BoundingBox3Ds &bas,          \
                           const algebra::BoundingBox3Ds &bbs) const;   \
  IMP_OBJECT(Name)

#endif  /* IMPCORE_MACROS_H */
