/**
 *  \file IMP/io.h
 *  \brief Handle read/write of Model data from/to files.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMP_IO_H
#define IMP_IO_H

#include "kernel_config.h"
#include "Model.h"
#include "file.h"
#include "FailureHandler.h"
#include "OptimizerState.h"
#include "internal/utility.h"
#include <boost/format.hpp>

IMP_BEGIN_NAMESPACE
/** \name Model IO
    \anchor model_text_io
    Models can be written to files in an easy-to-read
    file format based on \quote{name: value} pairs on separate lines with
    indentation used to indicated nesting (as in Python). The functions
    take a list of the particles to write/read. The particles
    need to be provided in the same order for reading and writing.
    This is not checked.

    The files should have the suffix ".imp".

    If a passed particle has a particle attribute which is a particle
    not in the list, an IOException is thrown. This can change to
    silently ignoring such attributes if desired.

    If a list of attributes is passed to read or write, only those
    attributes are read.

    \note If you are just interested in saving the state of a model during
    runtime, use an IMP::ConfigurationSet instead. It should be
    substantially faster.
    @{
*/
#ifndef IMP_DOXYGEN
IMPEXPORT void write_model(Model *m,
                           const ParticlesTemp &particles,
                           TextOutput out);
#endif
IMPEXPORT void write_model(const ParticlesTemp &particles,
                           TextOutput out);
IMPEXPORT void read_model(TextInput in,
                          const ParticlesTemp &particles
#ifndef IMP_DOXYGEN
                          ,Model *m=NULL
#endif
);

IMPEXPORT void write_model(const ParticlesTemp &particles,
                           const FloatKeys &keys,
                           TextOutput out);

IMPEXPORT void read_model(TextInput in,
                          const ParticlesTemp &particles,
                          const FloatKeys &keys);

#if !defined(IMP_DOXYGEN) && !defined(SWIG)
IMPEXPORT void write_model(Model *m,
                           TextOutput out);

IMPEXPORT void read_model(TextInput in, Model *m);
#endif
/** @} */

#if defined(IMP_USE_NETCDF) || defined(IMP_DOXYGEN)
/** \name Binary I/O
    \anchor model_binary_io
    When lots of data is being written, it can be useful to write the
    data as binary instead of text. Binary writing requires NetCDF.

    The files should have the suffix ".impb".

    For writing if the append is true is a positive integer, then
    the data is added to the file if it already exists.

    For reading, and IOException will be thrown if an invalid frame
    is requested. Frames are always sequential.

    \requires{binary I/O functions, NetCDF}
    @{
*/
IMPEXPORT void write_binary_model(const ParticlesTemp &particles,
                                  const FloatKeys &keys,
                                  std::string filename,
                                  bool append=false);
IMPEXPORT void read_binary_model(std::string filename,
                                 const ParticlesTemp &particles,
                                 const FloatKeys &keys,
                                 int frame=-1);
/** @} */
#endif


IMP_MODEL_SAVE(Write, (const ParticlesTemp &ps, std::string file_name),
               Particles ps_;,
               ps_=ps;,
               ,
               {
                 IMP_LOG(TERSE, "Writing text model file "
                         << file_name << std::endl);
                 write_model(ps_,file_name);
               });
#if defined(IMP_USE_NETCDF) || defined(IMP_DOXYGEN)
/** \class WriteBinaryOptimizerState
    In contrast to other similar OptimizerStates, this one expectes to write
    all models to the same file. As a result, the file name should not contain
    %1% (if it does, then separate files will be written). The first call will
    overwrite the file.

    \requires{class WriteBinaryOptimizerState, NetCDF}

    \class WriteBinaryFailureHandler

    \requires{class WriteBinaryFailureHandler, NetCDF}
 */
IMP_MODEL_SAVE(WriteBinary, (const ParticlesTemp &ps, const FloatKeys &fks,
                             std::string file_name),
               Particles ps_; FloatKeys fks_; mutable bool first_;,
               ps_=ps; fks_=fks;first_=-1;,
               ,
               {
                 IMP_LOG(TERSE, "Writing text model file "
                         << file_name << std::endl);
                 write_binary_model(ps_,fks_,file_name, !first_);
                 first_=false;
               });
#endif



IMP_END_NAMESPACE

#endif /* IMP_IO_H */
