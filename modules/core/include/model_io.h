/**
 *  \file model_io.h     \brief Support for io of the model.
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 *
 */

#ifndef IMPCORE_MODEL_IO_H
#define IMPCORE_MODEL_IO_H

#include "config.h"
#include "internal/version_info.h"
#include <IMP/Particle.h>
#include <IMP/Model.h>
#include <IMP/FailureHandler.h>

#include <string>
#include <iostream>

IMPCORE_BEGIN_NAMESPACE
/** \defgroup yaml Model IO
    IO is done a model-at-a-time and only supports particles. The
    reason IO of spacific particles is not supported is that such
    can easily break decorator-enforced invariants that the user has
    no way of understanding or maintaining.

    The model is stored as YAML file. YAML is a simple file format
    designed for storing a hierarchical set of values in a human
    editable and human readible format. The general format is that of
    "name: value" pairs on separate lines with indentation used to
    indicated nesting (as in python).

    \note At this point we do not guarantee that the file format will
    not change and loose backward compatibility.
*/

//! Write as Model to a file
/** \note This method is here for python users since we don't have stream
    support in python.

    \throw ValueException if the file cannot be opened.
    \ingroup helpers
    \ingroup yaml
*/
IMPCOREEXPORT void write(Model *m,
                         std::string file_name);
IMP_NO_DOXYGEN(IMP_NO_SWIG(IMPCOREEXPORT void write(Model *m,
                                                    std::ostream &out,
                                      std::string indent=std::string());))

//! Read the Model from a stream
/** The model must already have particles matching all read particles.
    Currently the particles must already have the same attributes
    as are being read, but this probably should change due to lists
    stored in attributes.

    The intentended usage model is that the model (with restraints) is
    initialized. Optimization is performed and then the model is written
    out to a file. Some time later, when you want to reload the model,
    you can reuse the initialization code to set up the restraints,
    and then read in the values for the attributes.

    \throw ValueException if the file cannot be opened.

    \note The base indent is determined from the first line.

    \ingroup helpers
    \ingroup yaml
*/
IMPCOREEXPORT void read(std::string file_name, Model *m);

IMP_NO_DOXYGEN(IMP_NO_SWIG(IMPCOREEXPORT
                           void read(std::istream &in, Model *m);))



/** \brief Dump the state of the model to a file on an error and then
    go on the the other handlers.

    When an error (check or assertion failure) occurs, the model is
    dumped to the specified file.

    \untested(set_failure_dump)
 */
class IMPCOREEXPORT DumpModelOnFailure: public FailureHandler {
  Model *m_;
  std::string file_name_;
 public:
  DumpModelOnFailure(Model *m, std::string file_name);
  IMP_FAILURE_HANDLER(DumpModelOnFailure, internal::version_info);
};


IMPCORE_END_NAMESPACE

#endif /* IMPCORE_MODEL_IO_H */
