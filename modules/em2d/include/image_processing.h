/**
 *  \file image_processing.h
 *  \brief Image processing functions
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
*/

#ifndef IMPEM2D_IMAGE_PROCESSING_H
#define IMPEM2D_IMAGE_PROCESSING_H

#include "IMP/em2d/em2d_config.h"
#include "IMP/em2d/PolarResamplingParameters.h"
#include "IMP/em2d/opencv_interface.h"
// #include "IMP/em2d/Pixel.h"
// #include "IMP/algebra/Matrix2D.h"
#include <algorithm>
#include <cmath>

IMPEM2D_BEGIN_NAMESPACE



//! Morphological grayscale reconstruction (L Vincent, 1993)
/*!
  \param[in] mask image to reconstruct
  \param[out] marker this image contains the initial marker points and will
              contain the final result
  \param[in] neighbors_mode number of neighbors for a pixel to consider when
             doing the morphological reconstruction (values: 4, 8).
*/
// IMPEM2DEXPORT void do_morphological_reconstruction(algebra::Matrix2D_d &mask,
//                      algebra::Matrix2D_d &marker,
//                      int neighbors_mode=4);
//
//

//! Returns the neighbors for a givel pixel in a given mode
/*!
  \ note IMPORTANT: The neighbors are returned in CLOCKWISE order starting
                    with i-1,j+0  (12 o'clock)
  \param[in] p the pixel to get the neighbors for
  \param[in] matrix where the neighbors are computed
  \param[in] mode modes supported for neighbors: 4 (default) and 8 neighbors
  \params[in] sign of the neighbors requested
              0 - all neighbors (default)
              1 - positive or before raster-order neighbors
              -1 - negative or before antiraster-order neighbors
  \params[in] cycle if true, the neighbors are cyclical at the border of the
              matrix. Eg. the neighbor to the right of last column, is the
              initial column. If this parameter is false (default) the
              neighbors outside the matrix are removed.
*/
// IMPEM2DEXPORT Pixels get_neighbors2d(
//                            const Pixel &p,const algebra::Matrix2D_d &m,
//                             int mode=4,
//                             int sign=0,
//                             bool cycle=false);



////! Fills the holes in the matrix m of height h
// IMPEM2DEXPORT void do_fill_holes(
//            algebra::Matrix2D_d &m, algebra::Matrix2D_d &result,double h);
//
//
////! Gets the domes of m with height h
// IMPEM2DEXPORT void get_domes(
//         algebra::Matrix2D_d &m,algebra::Matrix2D_d &result,double h);
//
//


//! Morphological opening of a grayscale matrix m.
/*!
  \param[in] m Matrix to dilate
  \param[in] kernel structuring element used for the opening
  \param[in] result resulting matrix
*/
//IMPEM2DEXPORT void do_opening(const algebra::Matrix2D_d &m,
//              algebra::Matrix2D_d &kernel,
//              algebra::Matrix2D_d &result);

//! Morphological do_closing of a grayscale matrix m.
/*!
  \param[in] m Matrix to dilate
  \param[in] kernel structuring element used for the opening
  \param[in] result resulting matrix
*/
// IMPEM2DEXPORT void do_closing(const algebra::Matrix2D_d &m,
//              algebra::Matrix2D_d &kernel,
//              algebra::Matrix2D_d &result);


//! Thresholding to get a binary image
/*!
  \param[in] mode if 1 the values higher than the threshold are given value 1.
                  If the mode is -1, the values lower the threshold are given 1.
*/
// IMPEM2DEXPORT void apply_threshold(
//                  const algebra::Matrix2D_d &m,algebra::Matrix2D_d &result,
//                   double threshold, int mode);
//

//! Applies a binary mask to an image.
/*!
  \param[in] mask If the mask is 1, the result has the value of m. Otherwise
             the value is given by the parameter value.
  \param[in] value value to apply when the mask is 0
  \note  m and result can be the same matrix.

*/
//IMPEM2DEXPORT void do_masking(
//          const algebra::Matrix2D_d &m,algebra::Matrix2D_d &result,
//          const algebra::Matrix2D<int> &mask,double value);
//



/*!
  \param[in] I input
  \param[in] deriv output derivative
  \param[in] temp_x temporary matrix to store dI/dx
  \param[in] deriv_y temporary matrix to store dI/dy
  \param[in] h temporary matrix to store the "edge indicator function"
  \param[in] dx - step for x
  \param[in] dy - step for y
  \param[in] ang - parameter for weight diffusion and edge detection (90-0)
*/
//void get_diffusion_filtered_partial_der_t(
//                      const algebra::Matrix2D_d &I,
//                      algebra::Matrix2D_d &It,
//                       double dx, double dy, double ang);

//! Smoothing filter by application of the reaction-diffusion
//! equation of Beltrami flow. Adiga, JSB, 2005
/*!
  \param [in] beta contribution of diffusion versus edge enhancement.
              0 - pure reaction, 90 - pure diffusion
*/
// IMPEM2DEXPORT void get_diffusion_filtered(
//             const algebra::Matrix2D_d &I,
//             algebra::Matrix2D_d &result,
//              double beta,
//              double pixelsize,
//              unsigned int t_steps);



//! Removes small objects from a labeled image. The background value is assumed
//! to be 0, and the labels start at 1 up to the number of labels.
/*!
  \param[in] m the matrix
  \param[in] percentage The percentage respect to the largest object that
             other objects have to be in order to survive the removal.
  \param[in] n_labels If higher than 0 (default) is the number of labels in the
             image. Otherwise, the function computes the number (slower)
*/
//template<typename T>
//void remove_small_objects(algebra::Matrix2D<T> &m,
//            double percentage,int n_labels=0) {
//  int background =0;
//  int foreground =1;
//  // If not given, compute the number of labels. Requires one scan
//  if(n_labels== 0) {
//    n_labels = algebra::get_rounded(m.compute_max());
//  }
//  // Vector to count pixels
//  std::vector<unsigned int> pixel_count(n_labels);
//  for (unsigned int i=0;i<pixel_count.size();++i) {
//    pixel_count[i]=0;
//  }
//  // First scan, count pixels
//  for (unsigned int i=0;i<m.num_elements();++i) {
//    int val = algebra::get_rounded(m.data()[i]);
//    if(val != background) {
//      (pixel_count[val-1])++;
//    }
//  }
//  // Get largest object
//  unsigned int max_pixels=*max_element(pixel_count.begin(),pixel_count.end());
//  std::vector<double> percentages(n_labels);
//  for (unsigned int i=0;i<pixel_count.size();++i) {
//    percentages[i]=((double)pixel_count[i])/((double)max_pixels);
//  }
//  // Second scan, remove objects of size lower than the percentage
//  for (unsigned int i=0;i<m.num_elements();++i) {
//    int val = algebra::get_rounded(m.data()[i]);
//    if(percentages[val-1] > percentage) {
//      m.data()[i] = (T)foreground;
//    } else {
//      m.data()[i] = (T)background;
//    }
//  }
//}


////! (U. Adiga, 2005)
///*!
//  \param[in] m binary matrix to dilate and shrink
//  \param[in] greyscale greyscale matrix that controls the shrinking
//  \param[in] kernel dilation kernel
//  \note Only tested with binary matrices (background =0 and foreground = 1)
//*/
// IMPEM2DEXPORT void do_dilate_and_shrink_warp(algebra::Matrix2D_d &m,
//                            const algebra::Matrix2D_d &greyscale,
//                            algebra::Matrix2D_d &kernel);


// IMPEM2DEXPORT void do_histogram_stretching(algebra::Matrix2D_d &m,
//                          int boxes,int offset);



/***************************/


//! Removes small objects from a labeled image. The background value is assumed
//! to be 0, and the labels start at 1 up to the number of labels.
/*!
  \param[in] m the matrix
  \param[in] percentage The percentage respect to the largest object that
             other objects have to be in order to survive the removal.
  \param[in] n_labels The number of labels in the image. If 0, the function
             computes the number
*/
IMPEM2DEXPORT void remove_small_objects(cv::Mat &m,
                          double percentage,
                          int n_labels=0,
                          int background=0,
                          int foreground=1);


 IMPEM2DEXPORT void do_histogram_stretching(cv::Mat &m,
                                           int boxes,
                                           int offset);

//! (U. Adiga, 2005)
/*!
  \param[in] m binary matrix to dilate and shrink
  \param[in] greyscale greyscale matrix that controls the shrinking
  \param[in] kernel dilation kernel
  \note Only tested with binary matrices m with background =0 and foreground = 1
*/
 void IMPEM2DEXPORT do_dilate_and_shrink_warp(cv::Mat &m,
                                const cv::Mat &greyscale,
                                cv::Mat &kernel);



//! Morphological grayscale reconstruction (L Vincent, 1993)
/*!
  \param[in] mask image to reconstruct
  \param[out] marker this image contains the initial marker points and will
              contain the final result
  \param[in] neighbors_mode number of neighbors for a pixel to consider when
             doing the morphological reconstruction (values: 4, 8).
*/
void IMPEM2DEXPORT do_morphological_reconstruction(const cv::Mat &mask,
                                      cv::Mat &marker,
                                      int neighbors_mode=4);



//! Labeling function for a matrix
/*!
  \param[in] m binary matrix to scan. The matrix needs to contain zeros and
             ones but they can be stored as doubles, floats or ints
  \param[out] result matrix it is returned as a matrix of ints
  \param[out] labels The number of labels in the image
*/
IMPEM2DEXPORT int do_labeling(const cvIntMat &m,
                cvIntMat &mat_to_label);


//! Segmentation of images
/*!
  \param[in] m The EM image to segment
  \param[in] result The segmented image, with the shape of the molecule
*/
IMPEM2DEXPORT void do_segmentation(const cv::Mat &m,
                                   cv::Mat &result);


//! Smoothing filter by application of the reaction-diffusion
//! equation of Beltrami flow. Adiga, JSB, 2005
/*!
  \param [in] beta contribution of diffusion versus edge enhancement.
              0 - pure reaction, 90 - pure diffusion
*/
IMPEM2DEXPORT void do_diffusion_filtering(const cv::Mat &m,
                           cv::Mat &result,
                           double beta,
                           double pixelsize,
                           unsigned int time_steps);

//! Partial derivative with respect to time for an image filtered with
//! difusion-reaction
/*!
  \param[in] m input matrix
  \param[in] der output derivative matrix
  \param[in] dx - step for x
  \param[in] dy - step for y
  \param[in] ang - parameter for weight diffusion and edge detection (90-0)
*/
IMPEM2DEXPORT void get_diffusion_filtering_partial_derivative(const cv::Mat &m,
                                                cv::Mat &der,
                                               double dx,
                                               double dy,
                                               double ang);


//! Fills the holes in the matrix m of height h
/*!
  \param[in] m the input matrix
  \param[in] result the result matrix
  \param[in] h the height
  \note The function does not work in-place
*/
IMPEM2DEXPORT void do_fill_holes(const cv::Mat &m,
                                 cv::Mat &result,
                                 double h);


//! Gets the domes of m with height h
/*!
  \param[in] m the input matrix
  \param[in] result the result matrix
  \param[in] h the height
  \note The function does not work in-place
*/
IMPEM2DEXPORT void get_domes(cv::Mat &m,cv::Mat &result,double h) ;


//!
/*!
  \param[in]
*/
IMPEM2DEXPORT void do_preprocess_em2d(cv::Mat &m,
                                     cv::Mat &result,
                                     double n_stddevs);



//! Computes the histogram of a matrix.
/*!
  \param[in] m Matrix with the data
  \param[in] bins Number of bins to use in the histogram
  \param[out] vector with the values for each bin
*/
IMPEM2DEXPORT Floats get_histogram(const cv::Mat &m, int bins);


//! Variance filter for an image. Computes the variance for each pixel using
//! the surrounding ones.
 /*!
  \param[in] input image with the data
  \param[out] filtered matrix result of the filtering with the variances
  \param[in] kernelsize The variance is computed using kernelsize x kernelsize
              pixels around each one. Kernelsize can only be odd.
*/
IMPEM2DEXPORT void apply_variance_filter(const cv::Mat &input,
                                   cv::Mat &filtered,int kernelsize);


//!Add noise to the values of a matrix.
/*!
    Supported distributions:
   - uniform distribution, giving the range (lower, upper). DEFAULT
   - gaussian distribution, giving the mean and the standard deviation
   \code
   add_noise(v1,0, 1);
   // uniform distribution between 0 and 1

   v1.add_noise(0, 1, "uniform");
   // the same

   v1.add_noise(0, 1, "gaussian");
  // gaussian distribution with 0 mean and stddev=1

   \endcode
*/
IMPEM2DEXPORT void add_noise(cv::Mat &v, double op1, double op2,
               const String &mode = "uniform", double df = 3);


//! Resamples a matrix to polar coordinates.
/*!
  \param[in] m matrix to resample
  \param[out] result matrix to contain the resampling
  \param[in] polar params Parameters used for the resampling. Extremely useful
            for speeding up the procedure if they are given with the
            transformation maps, that can be built in the
            PolarResamplingParameters class
*/
IMPEM2DEXPORT void do_resample_polar(const cv::Mat &input,
                                  cv::Mat &resampled,
                               const PolarResamplingParameters &polar_params);

//! Normalize a openCV matrix to mean 0 and stddev 1. It is done in place
IMPEM2DEXPORT void do_normalize(cv::Mat &m);

//! Applies a transformation to a matrix. First rotates the matrix using the
//! matrix center as the origin of the rotation, and then applies
//! the translation
IMPEM2DEXPORT void get_transformed(const cv::Mat &input,
                                   cv::Mat &transformed,
                                   const algebra::Transformation2D &T);



/*! Extends the bordes of an image
 \param[in] orig The image to extend
 \param[in] dst The image destination
 \param[in] pix number of pixels to extend the borders
*/
IMPEM2DEXPORT void do_extend_borders(cv::Mat &orig,
                                  cv::Mat &dst,
                                  unsigned int pix);



IMPEM2D_END_NAMESPACE

#endif /* IMPEM2D_IMAGE_PROCESSING_H */
