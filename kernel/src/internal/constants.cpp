/**
 *  \file constants.cpp  \brief Various useful constants.
 *
 *  Copyright 2007-8 Sali Lab. All rights reserved.
 *
 */

#include "IMP/internal/constants.h"

namespace IMP
{

namespace internal
{

const unit::ExponentialNumber<23> NA(6.02214179);

const unit::internal::AtomsPerMol unit::ATOMS_PER_MOL = {};

// definition of KB
const unit::Shift<unit::Divide<unit::Joule, unit::Kelvin>::type, -23>::type
KB(1.3806503);

const unit::Kelvin T(297.15);


const unit::ExponentialNumber<3> JOULES_PER_KILOCALORIE(4.1868);

} // namespace internal

} // namespace IMP
