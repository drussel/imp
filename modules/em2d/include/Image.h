/**
 *  \file em2d/Image.h
 *  \brief IMP images for Electron Microscopy using openCV matrices
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
*/

#ifndef IMPEM2D_IMAGE_H
#define IMPEM2D_IMAGE_H

#include "IMP/em2d/em2d_config.h"
#include "IMP/em2d/PolarResamplingParameters.h"
#include "IMP/em2d/ImageReaderWriter.h"
#include "IMP/em2d/opencv_interface.h"
#include "IMP/em2d/image_processing.h"
#include "IMP/em2d/FFToperations.h"
#include "IMP/em/ImageHeader.h"
#include <IMP/Pointer.h>
#include <IMP/Object.h>
#include "IMP/VectorOfRefCounted.h"
#include <limits>
#include <typeinfo>






IMPEM2D_BEGIN_NAMESPACE

//! 2D Electron Microscopy images in IMP
class IMPEM2DEXPORT Image : public Object
{
public:

  Image();

  //! Create image of the proper size
  Image(int rows, int cols);

  //! Create the image reading from a file
  Image(String filename, const ImageReaderWriter *reader):
    Object("Image%1%"){
    read(filename,reader);
  }

  //! Access to the matrix of data
  cv::Mat &get_data() {
    return data_;
  }

  //! Sets the entire matrix of data
  void set_data(const cv::Mat &mat);

  void set_value(int i, int j, double val) {
    data_.at<double>(i,j) = val;
  }

  double operator()(int i, int j) const {
    return data_.at<double>(i,j);
  }


  //! Access to the header
  inline em::ImageHeader& get_header() {
    update_header();
    return header_;
  }

  void set_size(int rows,int cols);

  //! Resize to the same size of the parameter image
  void set_size(Image *img);

  //! Adjusts the information of the imager header taking into account the
  //! dimensions of the data and setting the time, date, type, etc ...
  void update_header();

  //! Reads and casts the image from the file (the image matrix of data must
  //! be stored as floats)
  void read(String filename, const ImageReaderWriter *reader) {
    Pointer<const ImageReaderWriter> ptr(reader);
    reader->read(filename,header_,data_);
  }

  //! Writes the image to a file (the image matrix of data is stored as floats
  //! when writing)
  void write(const String &filename, const ImageReaderWriter *writer) {
    Pointer<const ImageReaderWriter> ptr(writer);
    update_header(); // adjust the header to guarantee consistence
    writer->write(filename,header_,data_);
  }

  void show(std::ostream& out) const {
    out << "Image name   : " << name_ ;
    header_.show(out);
  }

  //! Define the basic things needed by any Object
//  IMP_OBJECT_INLINE(Image, show(out), {});
  IMP_OBJECT_INLINE(Image, show(out), destroyed_msg());

  void destroyed_msg() {
    IMP_LOG(TERSE, "Image destroyed " << this->name_ << std::endl);
  }

  void set_name(const String &name) {
    name_ = name;
  }

  String get_name() const {
    return name_;
  }

#if !defined(IMP_DOXYGEN) && !defined(SWIG)
  void get_min_and_max_values(double *min, double *max) const;
#endif

protected:
  void set_size_data(int rows,int cols);
  void set_defaults();

  //! Name of the image. Frequently it will be the name of the file
  String name_;
  //! Matrix with the data for the image
//  cv::Mat data_;
  cv::Mat data_;
  //! Header for the image with all the pertinent information
  em::ImageHeader header_;

}; // Image

//! A vector of reference counted pointers to EM images of type double
IMP_OBJECTS(Image,Images);

IMP_OUTPUT_OPERATOR(Image);




//! Reads images from files (For compatibility with SPIDER format,
//! the images are read from floats)
/*!
  \param[in] names filenames of the images
  \param[in] rw  reader/writer to use
*/
IMPEM2DEXPORT Images read_images(const Strings &names,
                                 const ImageReaderWriter *rw);

//! Saves images to files (For compatibility with SPIDER format,
//! the images are written to floats)
/*!
  \param[in] images Images to save
  \param[in] names filenames of the images
  \param[in] rw  reader/writer to use
*/
IMPEM2DEXPORT void save_images(Images images, const Strings &names,
                               const ImageReaderWriter *rw);

//! Cross correlation between two images
IMPEM2DEXPORT double get_cross_correlation_coefficient(Image *im1,Image *im2);

//! Autocorrelation image
inline void get_autocorrelation2d(Image *im1,Image *im2) {
  get_autocorrelation2d(im1->get_data(),im2->get_data());
}

//! Cross correlation between two images
inline void get_correlation2d(Image *im1,Image *im2,Image *corr) {
  get_correlation2d(im1->get_data(),im2->get_data(),corr->get_data());
}


IMPEM2DEXPORT void do_normalize(Image *im,bool force=false);


inline Floats get_histogram(Image *img, int bins) {
  return get_histogram(img->get_data(),bins);
}

inline void apply_variance_filter(Image *input,
                           Image *filtered,int kernelsize) {
  apply_variance_filter(input->get_data(),filtered->get_data(),kernelsize);
}

inline void apply_diffusion_filter(Image *input,
                                   Image *filtered,
                                   double beta,
                                   double pixelsize,
                                   int time_steps) {
  apply_diffusion_filter(input->get_data(),
                         filtered->get_data(),
                         beta,
                         pixelsize,
                         time_steps);
}


inline void do_fill_holes(Image *input,Image *result,double n_stddevs) {
  do_fill_holes(input->get_data(),result->get_data(),n_stddevs);

}

inline void do_combined_fill_holes_and_threshold(Image *input,
                                                 Image *result,
                                                 double n_stddevs) {
  do_combined_fill_holes_and_threshold(input->get_data(),
                                       result->get_data(),
                                       n_stddevs);
}



inline void do_extend_borders(Image *im1,Image *im2,unsigned int pix) {
  do_extend_borders(im1->get_data(),im2->get_data(),pix);
}


inline void do_segmentation(Image *input,
                            Image *result,
                            const SegmentationParameters &params) {
  do_segmentation(input->get_data(),result->get_data(),params);
}

IMPEM2DEXPORT void do_remove_small_objects(Image *input,
                            double percentage,
                            int background=0,
                            int foreground=1);


IMPEM2DEXPORT void do_subtract_images(Image *first,Image *second,
                                  Image *result);

IMPEM2DEXPORT void add_noise(Image *im1,double op1, double op2,
               const String &mode = "uniform", double df = 3);


IMPEM2DEXPORT void do_resample_polar(Image *im1,Image *im2,
                const PolarResamplingParameters &polar_params);


IMPEM2D_END_NAMESPACE

#endif  /* IMPEM2D_IMAGE_H */
