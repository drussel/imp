/**
 *  \file CLASSNAMEsRestraint.h
 *  \brief Apply a CLASSNAMEScore to each CLASSNAME in a list.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPCONTAINER_HEADERNAMES_RESTRAINT_H
#define IMPCONTAINER_HEADERNAMES_RESTRAINT_H

#include "container_config.h"

#include <IMP/core/internal/CoreCLASSNAMEsRestraint.h>

#include <iostream>

IMPCONTAINER_BEGIN_NAMESPACE

//! Applies a CLASSNAMEScore to each CLASSNAME in a list.
/** This restraint stores the used particles in a PLURALVARIABLETYPE.
    The container used can be set so that the list can be shared
    with other containers (or a nonbonded list can be used). By default
    a ListCLASSNAMEContainer is used and the
    {add_, set_, clear_}FUNCTIONNAME{s} methodas can be used.

    Examples using various multiplicity containers:
    \pythonexample{restrain_in_sphere}
    \pythonexample{nonbonded_interactions}

    \see CLASSNAMERestraint
 */
class IMPCONTAINEREXPORT CLASSNAMEsRestraint :
#if defined(SWIG) || defined(IMP_DOXYGEN)
public Restraint
#else
public core::internal::CoreCLASSNAMEsRestraint
#endif
{
  typedef core::internal::CoreCLASSNAMEsRestraint P;
public:

 //! Create the restraint with a shared container
  /** \param[in] ss The function to apply to each particle.
      \param[in] pc The container containing the stored particles. This
      container is not copied.
      \param[in] name The object name
   */
  CLASSNAMEsRestraint(CLASSNAMEScore *ss,
                      CLASSNAMEContainer *pc,
                      std::string name="CLASSNAMEsRestraint %1%");

#if defined(IMP_DOXYGEN) || defined(SWIG)
  IMP_RESTRAINT(CLASSNAMEsRestraint);

  //! Get the container used to store Particles
  PLURALVARIABLETYPE get_arguments() const;

  CLASSNAMEContainer* get_container() const;

  CLASSNAMEScore* get_score() const;
#else
  IMP_OBJECT(CLASSNAMEsRestraint);
#endif
};

IMP_OBJECTS(CLASSNAMEsRestraint,CLASSNAMEsRestraints);

IMPCONTAINER_END_NAMESPACE

#endif  /* IMPCONTAINER_HEADERNAMES_RESTRAINT_H */
