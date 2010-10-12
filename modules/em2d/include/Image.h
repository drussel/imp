/**
 *  \file Image.h
 *  \brief IMP images for Electron Microscopy using openCV matrices
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
*/

#ifndef IMPEM2D_IMAGE_H
#define IMPEM2D_IMAGE_H

#include "IMP/em2d/em2d_config.h"
#include "IMP/em/ImageHeader.h"
#include "IMP/em2d/ImageReaderWriter.h"
#include "IMP/VectorOfRefCounted.h"
#include <IMP/Object.h>
#include "IMP/em2d/opencv_interface.h"
#include "IMP/VectorOfRefCounted.h"
#include <complex>
#include <limits>
#include <typeinfo>

IMPEM2D_BEGIN_NAMESPACE

//! 2D Electron Microscopy images in IMP
class IMPEM2DEXPORT Image : public Object
{
public:

  //! Empty constructor
  Image() {
    name_ = "";
    header_.set_image_type(em::ImageHeader::IMG_IMPEM);
    // Stats not computed
    header_.set_fSig(-1);
    header_.set_fImami(0);
  }

  //! Constructor with size
  Image(Int rows, Int cols) {
    data_.create(rows,cols,CV_64FC1);
    header_.set_header();
    header_.set_image_type(em::ImageHeader::IMG_IMPEM);
    header_.set_fSig(-1);
    header_.set_fImami(0);
  }

  //! Access to the matrix of data
  cv::Mat& get_data() {
    return data_;
  }

  //! Sets the entire matrix of data
  void set_data(cv::Mat &mat) {
    mat.copyTo(data_);
    this->adjust_header();
  }

////////////////////////// TO DO
  //! Set the value of one LOGICAL element of the matrix of data
  void set_value(int i, int j,double val) const {
    // data_(i,j)=val;
  }

////////////////////////// TO DO
  //! Access to one LOGICAL element of the matrix of data
  double operator()(int i, int j) const {
    // return data_(i,j);
    return 0;
  }

  //! Access to the header
  inline em::ImageHeader& get_header() {
    return header_;
  }


  void resize(int rows,int cols) {
    data_.create(rows,cols,CV_64FC1);
    header_.set_number_of_slices(1.0);
    header_.set_number_of_rows(rows);
    header_.set_number_of_columns(cols);
  }

  void resize(Image &img) {
    resize(img.get_data().rows,img.get_data().cols);
  }

  //! Adjusts the information of the imager header taking into account the
  //! dimensions of the data and setting the time, date, type, etc ...
  void adjust_header() {
    header_.set_image_type(em::ImageHeader::IMG_IMPEM);
    header_.set_number_of_slices(1.0);
    header_.set_number_of_rows(data_.rows);
    header_.set_number_of_columns(data_.cols);
    header_.set_time();
    header_.set_date();
    header_.set_title(name_);
    header_.set_header(); // Initialize header
  }

  //! Reads and casts the image from the file (the image matrix of data must
  //! be stored as floats)
  void read_from_floats(String filename,
                          em2d::ImageReaderWriter<double>& reader) {
    reader.read_from_floats(filename,header_,data_);
  }

  //! Writes the image to a file (the image matrix of data is stored as floats
  //! when writing)
  void write_to_floats(String filename,
                          em2d::ImageReaderWriter<double>& writer) {
    adjust_header(); // adjust the header to guarantee consistence
    writer.write_to_floats(filename,header_,data_);
  }

  void show(std::ostream& out) const {
    out << "Image name   : " << name_ ;
    header_.show(out);
  }

  //! Define the basic things needed by any Object
  IMP_OBJECT_INLINE(Image, show(out), {});

protected:

  //! Name of the image. Frequently it will be the name of the file
  String name_;
  //! Matrix with the data for the image
  cv::Mat data_;
  //! Header for the image with all the pertinent information
  em::ImageHeader header_;

}; // Image

//! A vector of reference counted pointers to EM images of type double
IMP_OBJECTS(Image,Images);

IMP_OUTPUT_OPERATOR(Image);


//! Reads images from files (For compatibility with SPIDER format,
//! the images are read from floats)
/**
  \param[in] names filenames of the images
  \param[in] rw  reader/writer to use
**/
IMPEM2DEXPORT Images read_images(Strings names,
                                  em2d::ImageReaderWriter<double> &rw);


//! Saves images to files (For compatibility with SPIDER format,
//! the images are written to floats)
/**
  \param[in] images Images to save
  \param[in] names filenames of the images
  \param[in] rw  reader/writer to use
**/
IMPEM2DEXPORT void save_images(Images images, Strings names,
                             em2d::ImageReaderWriter<double> &rw);

IMPEM2D_END_NAMESPACE

#endif  /* IMPEM2D_IMAGE_H */
