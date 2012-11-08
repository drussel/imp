/**
 *  \file RMF/utility.h
 *  \brief Helper functions for manipulating RMF files.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPLIBRMF_UTILITY_H
#define IMPLIBRMF_UTILITY_H

#include "RMF_config.h"
#include "internal/intrusive_ptr_object.h"
#include "internal/errors.h"
#include "NodeConstHandle.h"
namespace RMF {

  class FileConstHandle;
  class FileHandle;

  /** \name Copy functions

      \note These functions make use of the association field.
      @{
  */
/** Turn on and off printing of hdf5 error messages. They can help in
      diagnostics, but, for the moment, can only be output to standard
      error and so are off by default.
*/
inline void set_show_hdf5_errors(bool tf) {
  internal::show_hdf5_errors=tf;
}


  /** Copy the hierarchy structure and set structure from one rmf
      file to another.*/
RMFEXPORT void copy_structure(FileConstHandle input, FileHandle output);

  /** Add the associations between the input and output needed to
      use the other copy functions.*/
  RMFEXPORT void link_structure(FileConstHandle input, FileHandle output);

  /** Copy the hierarchy structure entailed by the set of nodes passed
      for the input. They most form a coherent subhierarchy of their
      input file (no nodes whose parents aren't included).*/
RMFEXPORT void copy_structure(const NodeConstHandles& input, FileHandle output);


  /** Copy the data of a single frame from between two files. Parts missing
      in the output file will be skipped.*/
  RMFEXPORT void copy_frame(FileConstHandle input, FileHandle output,
                            unsigned int inframe,
                            unsigned int outframe);
  /** Copy all values of the given nodes to the output file.*/
  RMFEXPORT void copy_values( FileConstHandle input, FileHandle output);
  /** @} */

  /** Return true of the two have the same structure.*/
  RMFEXPORT bool get_equal_structure(FileConstHandle input,
                                     FileConstHandle output,
                                     bool print_diff=false);
  /** Return true of the two have the same structure.*/
  RMFEXPORT bool get_equal_frame(FileConstHandle input, FileConstHandle out,
                                 unsigned int inframe, unsigned int outframe,
                                 bool print_diff=false);

} /* namespace RMF */

#endif /* IMPLIBRMF_UTILITY_H */