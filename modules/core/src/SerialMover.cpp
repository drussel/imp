/**
 *  \file SerialMover.cpp
 *  \brief A mover that apply other movers one at a time
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 *
 */
#include <IMP/core/SerialMover.h>
#include <IMP/core.h>
#include <iostream>

IMPCORE_BEGIN_NAMESPACE

SerialMover::SerialMover(const MoversTemp& mvs):
             mvs_(mvs), imov_(-1) {};

ParticlesTemp SerialMover::propose_move(Float f) {
  IMP_LOG(VERBOSE,"SerialMover:: propose move f is  : " << f <<std::endl);
  ++imov_;
  if(imov_==mvs_.size()) imov_=0;

  return mvs_[imov_]->propose_move(f);
}

void SerialMover::reset_move() {
 mvs_[imov_]->reset_move();
}

void SerialMover::do_show(std::ostream &out) const {
  out << "number of movers: " << mvs_.size() << "\n";
}

IMPCORE_END_NAMESPACE
