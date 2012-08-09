/**
 *  \file IMP/rmf/restraint_io.h
 *  \brief Handle read/write of Model data from/to files.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPRMF_RESTRAINT_IO_H
#define IMPRMF_RESTRAINT_IO_H

#include "rmf_config.h"
#include "link_macros.h"
#include <IMP/base/object_macros.h>
#include <IMP/Restraint.h>
#include <IMP/restraint_macros.h>

IMPRMF_BEGIN_NAMESPACE



/** \name Restraint I/O
    Restraints are written as am RMF::FEATURE node with subnodes for
    the decomposed restraints (Restraint::create_current_decompositon()).
    The Restraints::get_last_score() value is what is saved to the file,
    so make sure that the restraints have been evaluated before saving
    a frame.

    Any particles returnd by Restraint::get_input_particles() that are
    also linked in the RMF file will be included in the RMF file as
    inputs for the Restraint. This allows external software like e.g.,
    Chimera to associate these restrains with a certain set of particles.
    @{
*/
IMP_DECLARE_LINKERS(Restraint, restraint, restraints,
                    Restraint*, Restraints, // InType
                    Restraint*, RestraintsTemp, // OutType
                    (RMF::FileConstHandle fh, Model *m),
                    );

/** Certain restraint are made from a really large number of terms (eg
    IMP::core::DopePairScore based ones). Tracking and displaying all those
    terms can be very time consuming. If the number of terms is larger
    than the maximum, the terms are not displayed. By default this is
    100.*/
IMPRMFEXPORT void set_maximum_number_of_terms(RMF::FileHandle fh,
                                              unsigned int num);
/** @} */


IMPRMF_END_NAMESPACE

#endif /* IMPRMF_RESTRAINT_IO_H */
