/**
 *  \file IMP/container/ListCLASSNAMEContainer.h
 *  \brief Store a list of PLURALVARIABLETYPE
 *
 *  BLURB
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */

#ifndef IMPCONTAINER_LIST_HEADERNAME_CONTAINER_H
#define IMPCONTAINER_LIST_HEADERNAME_CONTAINER_H

#include "container_config.h"
#include <IMP/internal/InternalListCLASSNAMEContainer.h>

IMPCONTAINER_BEGIN_NAMESPACE

//! Store a list of PLURALVARIABLETYPE
/** \note The indexes can change when particles are inserted
    as the list is maintained in sorted order.
 */
class IMPCONTAINEREXPORT ListCLASSNAMEContainer:
#if defined(IMP_DOXYGEN) || defined(SWIG)
public CLASSNAMEContainer
#else
public IMP::internal::InternalListCLASSNAMEContainer
#endif
{
  typedef IMP::internal::InternalListCLASSNAMEContainer P;
public:
  ListCLASSNAMEContainer(const PLURALVARIABLETYPE &ps);

  //! construct and pass an initial set of CLASSNAMEs
  ListCLASSNAMEContainer(const PLURALVARIABLETYPE &ps,
                         std::string name);

  ListCLASSNAMEContainer(Model *m,
                         std::string name= "ListCLASSNAMEContainer %1%");
  ListCLASSNAMEContainer(Model *m, const char *name);

#if defined(IMP_DOXYGEN) || defined(SWIG)
 /** @name Methods to control the contained objects

     This container stores a list of CLASSNAME objects. To manipulate
     the list use these methods.
  */
  /**@{*/
  void add_FUNCTIONNAME(ARGUMENTTYPE vt);
  void add_FUNCTIONNAMEs(const PLURALVARIABLETYPE &c);
  void set_FUNCTIONNAMEs(PLURALVARIABLETYPE c);
  void clear_FUNCTIONNAMEs();
  /**@}*/
  IMP_HEADERNAME_CONTAINER(ListCLASSNAMEContainer);
#else
  IMP_OBJECT(ListCLASSNAMEContainer);
#endif
};

IMP_OBJECTS(ListCLASSNAMEContainer,ListCLASSNAMEContainers);

IMPCONTAINER_END_NAMESPACE

#endif  /* IMPCONTAINER_LIST_HEADERNAME_CONTAINER_H */
