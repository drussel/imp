/**
 *  \file ListCLASSNAMEContainer.h    \brief Store a list of PLURALVARIABLETYPE
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 */

#ifndef IMPCORE_INTERNAL_CORE_LIST_HEADERNAME_CONTAINER_H
#define IMPCORE_INTERNAL_CORE_LIST_HEADERNAME_CONTAINER_H

#include "../core_config.h"
#include <IMP/CLASSNAMEContainer.h>
#include <IMP/internal/container_helpers.h>
#include "HELPERNAME_helpers.h"
#include <IMP/ScoreState.h>
#include <IMP/scoped.h>
#include "../generic.h"

IMPCORE_BEGIN_INTERNAL_NAMESPACE


class IMPCOREEXPORT CoreListCLASSNAMEContainer:
  public internal::ListLikeCLASSNAMEContainer
{
  IMP_ACTIVE_CONTAINER_DECL(CoreListCLASSNAMEContainer);
 public:
  CoreListCLASSNAMEContainer(Model *m, std::string name);
  CoreListCLASSNAMEContainer(Model *m, const char *name);
  CoreListCLASSNAMEContainer();
  void add_FUNCTIONNAME(ARGUMENTTYPE vt) {
    IMP_USAGE_CHECK(IMP::internal::is_valid(vt),
                    "Passed CLASSNAME cannot be NULL (or None)");
    add_to_list(IMP::internal::get_index(vt));
    IMP_USAGE_CHECK(!get_has_added_and_removed_containers()
                    || !get_removed_container()
                    ->get_contains(vt),
      "You cannot remove and add the same item in one time step.");
  }
  void add_FUNCTIONNAMEs(const PLURALARGUMENTTYPE &c) {
    if (c.empty()) return;
    PLURALINDEXTYPE cp= IMP::internal::get_index(c);
    add_to_list(cp);
    IMP_IF_CHECK(USAGE) {
      for (unsigned int i=0; i< c.size(); ++i) {
        IMP_USAGE_CHECK(IMP::internal::is_valid(c[i]),
                        "Passed CLASSNAME cannot be NULL (or None)");
        IMP_USAGE_CHECK(!get_has_added_and_removed_containers()
                        || !get_removed_container()
                        ->get_contains(c[i]),
        "You cannot remove and add the same item in one time step.");

      }
    }
  }
  void remove_FUNCTIONNAMEs(const PLURALARGUMENTTYPE &c);
  void set_FUNCTIONNAMEs(PLURALARGUMENTTYPE c) {
    PLURALINDEXTYPE cp= IMP::internal::get_index(c);
    update_list(cp);
  }
  void set_FUNCTIONNAMEs(PLURALINDEXTYPE cp) {
    update_list(cp);
  }
  void clear_FUNCTIONNAMEs() {
    PLURALINDEXTYPE t;
    update_list(t);
  }
  bool get_is_up_to_date() const {
    return true;
  }
  IMP_LISTLIKE_HEADERNAME_CONTAINER(CoreListCLASSNAMEContainer);
};


IMPCORE_END_INTERNAL_NAMESPACE

#endif  /* IMPCORE_INTERNAL_CORE_LIST_HEADERNAME_CONTAINER_H */
