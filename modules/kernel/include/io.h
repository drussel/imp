/**
 *  \file IMP/io.h
 *  \brief Handle read/write of Model data from/to files.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPKERNEL_IO_H
#define IMPKERNEL_IO_H

#include "kernel_config.h"
#include "file.h"
#include "deprecation.h"
#include "FailureHandler.h"
#include "OptimizerState.h"
#include "internal/utility.h"
#include <boost/format.hpp>

IMP_BEGIN_NAMESPACE

/** \name Buffer I/O
    Write/read the state of the particles to/from a buffer in memory.
    \note Not all particles need to have all the attributes,
    missing attributes will be skipped. However, the set of attributes
    must match on the write and read particles.

    \note There is no handling of architectural issues. That is, this
    is only guaranteed to work if it is read and written on the same
    operating system and system bit length. We could probably fix this.

    \note both these methods should be considered unstable.
    @{
*/
//! return a binary buffer with the data
IMPEXPORT std::vector<char>
write_particles_to_buffer(const ParticlesTemp &particles,
                          const FloatKeys &keys);
//! load found attributes into the particles
IMPEXPORT void read_particles_from_buffer( const std::vector<char> &buffer,
                                       const ParticlesTemp &particles,
                                       const FloatKeys &keys);

/** @} */

IMP_END_NAMESPACE

#endif /* IMPKERNEL_IO_H */
