/**
 *  \file ImageReaderWriter.h
 *  \brief Virtual class for reader/writers of images
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
*/

#ifndef IMPEM_IMAGE_READER_WRITER_H
#define IMPEM_IMAGE_READER_WRITER_H

#include "em_config.h"
#include "ImageHeader.h"
#include <IMP/algebra/Matrix2D.h>

IMPEM_BEGIN_NAMESPACE

template<typename T>
class ImageReaderWriter
{
public:
  virtual ~ImageReaderWriter() {}

  virtual void read(String filename, ImageHeader& header,
                                     algebra::Matrix2D<T>& data) const {
  }

  virtual void read_from_floats(String filename, ImageHeader& header,
                                     algebra::Matrix2D<T>& data) const  {
  }

  virtual void read_from_ints(String filename, ImageHeader& header,
                                     algebra::Matrix2D<T>& data) const {
  }

  virtual void write(String filename, ImageHeader& header,
                                     algebra::Matrix2D<T>& data) const  {
  }

  virtual void write_to_floats(String filename, ImageHeader& header,
                                     algebra::Matrix2D<T>& data) const {
  }

  virtual void write_to_ints(String filename, ImageHeader& header,
                                     algebra::Matrix2D<T>& data) const {
  }
};

IMPEM_END_NAMESPACE

#endif /* IMPEM_IMAGE_READER_WRITER_H */
