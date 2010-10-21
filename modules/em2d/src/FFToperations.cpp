/**
 *  \file FFToperations.cpp
 *  \brief operations involving FFT
 Copyright 2007-2010 IMP Inventors. All rights reserved.
**/

#include "IMP/em2d/FFToperations.h"
#include "IMP/em/Image.h"
#include "IMP/em/SpiderReaderWriter.h"

IMPEM2D_BEGIN_NAMESPACE

// Forget about const in these functions. FFT1D and FFT2D when used with fftw3
// can not be const.


void correlation2D(algebra::Matrix2D_d &m1,
                   algebra::Matrix2D_d &m2,
                          algebra::Matrix2D_d &corr) {
  IMP_LOG(IMP::VERBOSE,"Computing 2D correlation " <<std::endl);

  IMP_USAGE_CHECK((m1.get_number_of_rows()==m2.get_number_of_rows()) &&
                  (m1.get_number_of_columns()==m1.get_number_of_columns()),
                  "em2d:correlation2D: Matrices have different size.");
  corr.resize(m1);
  algebra::Matrix2D_c M1,M2;
  FFT2D fft1(m1,M1); fft1.execute();
  FFT2D fft2(m2,M2); fft2.execute();
  for(unsigned long i=0;i<M1.num_elements();++i) {
    std::complex<double> x1 = M1.data()[i];
    std::complex<double> x2 = M2.data()[i];
    M2.data()[i] = x1*std::conj(x2);
  }
  IFFT2D ifft(M2,corr); ifft.execute(); // corr contains the correlation matrix
  matrix_to_image_interpretation(corr);
}

void correlation2D_no_preprocessing(
                   algebra::Matrix2D_c &M1,
                   algebra::Matrix2D_c &M2,
                   algebra::Matrix2D_d &corr) {
  IMP_LOG(IMP::VERBOSE,
          "Computing 2D correlation with no preprocessing" << std::endl);
  IMP_USAGE_CHECK((M1.get_number_of_rows()==M2.get_number_of_rows()) &&
                  (M1.get_number_of_columns()==M2.get_number_of_columns()),
        "em2d::correlation2D_no_preprocessing: Matrices have different size.");

  algebra::Matrix2D_c CORR(M1.get_size(0),M1.get_size(1));
  for(unsigned long i=0;i<M1.num_elements();++i) {
    CORR.data()[i] = M1.data()[i] * std::conj(M2.data()[i]);
  }
  IFFT2D ifft(CORR,corr);ifft.execute(); //corr contains the correlation matrix
  matrix_to_image_interpretation(corr);
}

void autocorrelation2D(algebra::Matrix2D_d& m,
                               algebra::Matrix2D_d& corr) {
  IMP_LOG(IMP::VERBOSE,"Computing 2D autocorrelation" <<std::endl);
  algebra::Matrix2D_c M;
  corr.resize(m);
  FFT2D fft(m,M); fft.execute();
  autocorrelation2D_no_preprocessing(M,corr);
}




void autocorrelation2D_no_preprocessing(
                    algebra::Matrix2D_c &M,
                    algebra::Matrix2D_d &corr) {
  IMP_LOG(IMP::VERBOSE,"Computing 2D autocorrelation with no preprocessing"
              <<std::endl);
  algebra::Matrix2D_c AUTOCORR(M.get_size(0),M.get_size(1));
  for(unsigned long i=0;i<M.num_elements();++i) {
    (AUTOCORR.data()[i]) = std::norm(M.data()[i]);
  }
  IFFT2D ifft(AUTOCORR,corr); ifft.execute();
  matrix_to_image_interpretation(corr);
 };


void print_matrix(algebra::Matrix2D_c &m,
                  std::ostream &out) {
  for (int j = m.get_start(0);j <= m.get_finish(0);j++) {
    for (int  i = m.get_start(1);i <= m.get_finish(1);++i) {
      out << m(j,i) << " ";
    }
    out << std::endl;
  }
}

void print_matrix(algebra::Matrix3D<std::complex<double> > &m,
                  std::ostream &out) {
  for (int k = m.get_start(0);k <= m.get_finish(0);k++) {
    if (m.get_size(0) > 1) {
      std::cout << "Slice No. " << k << std::endl;
    }
    for (int j = m.get_start(1);j <= m.get_finish(1);j++) {
      for (int i = m.get_start(2);i <= m.get_finish(2);++i) {
        out << m(k,j,i) << ' ';
      }
      out << std::endl;
    }
  }
}

void Matrix2D_c_to_img(algebra::Matrix2D_c &M,String name) {
  em::SpiderImageReaderWriter<double> srw;
  em::Image img;
  img.resize(M.get_size(0),M.get_size(1));
  for(unsigned long i=0;i<M.num_elements();++i) {
      (img.get_data().data()[i]) = std::norm(M.data()[i]);
  }
  img.write_to_floats(name,srw);

}


void autocorrelation2D_no_preprocessing(const cv::Mat& M, cv::Mat& corr) {
  IMP_LOG(IMP::VERBOSE,
              "Computing 2D autocorrelation no preprocessing" <<std::endl);
  IMP_USAGE_CHECK(((M.rows!=0) && (M.cols !=0)),
     "em2d:autocorrelation2D: Output matrix has not rows or cols");
  cv::Mat temp;
  cv::mulSpectrums(M,M, temp,0,true);
  cv::dft(temp, temp, cv::DFT_INVERSE + cv::DFT_SCALE, corr.rows);
  temp(cv::Rect(0, 0, corr.cols, corr.rows)).copyTo(corr);
  matrix_to_image_flip(corr);
}


void autocorrelation2D(const cv::Mat& m, cv::Mat& corr) {
  IMP_LOG(IMP::VERBOSE,"Computing 2D autocorrelation " <<std::endl);
  // resize the output array if needed
  corr.create(m.rows,m.cols, m.type());
  cv::Size dftSize;
  // compute the optimal size for faster DFT transform
  dftSize.width = cv::getOptimalDFTSize(m.cols);
  dftSize.height = cv::getOptimalDFTSize(m.rows);
  // temporary
  cv::Mat tempA(dftSize, m.type(), cv::Scalar::all(0));
  // copy A to the top-left corner of tempA
  cv::Mat roiA(tempA, cv::Rect(0,0,m.cols,m.rows));
  m.copyTo(roiA);
  cv::dft(tempA, tempA,0,m.rows);
  autocorrelation2D_no_preprocessing(tempA,corr);
}


void correlation2D(const cv::Mat& A, const cv::Mat& B, cv::Mat& corr) {

  IMP_LOG(IMP::VERBOSE,"Computing 2D correlation " <<std::endl);

  IMP_USAGE_CHECK(((A.rows==B.rows) && (A.cols == B.cols)),
                  "em2d:correlation2D: Matrices have different size.");
  // resize the output array if needed
  corr.create(A.rows,A.cols, A.type());
  cv::Size dftSize;
  // compute the optimal size for faster DFT transform
  dftSize.width = cv::getOptimalDFTSize(A.cols);
  dftSize.height = cv::getOptimalDFTSize(A.rows);

  // temporary matrices
  cv::Mat tempA(dftSize, A.type(), cv::Scalar::all(0));
  cv::Mat tempB(dftSize, B.type(), cv::Scalar::all(0));

  // copy A and B to the top-left corners of tempA and tempB, respectively
  cv::Mat roiA(tempA, cv::Rect(0,0,A.cols,A.rows));
  A.copyTo(roiA);
  cv::Mat roiB(tempB, cv::Rect(0,0,B.cols,B.rows));
  B.copyTo(roiB);

  // FFT the padded A & B in-place;
  // use "nonzeroRows" hint for faster processing
  cv::dft(tempA, tempA, 0, A.rows);
  cv::dft(tempB, tempB, 0, B.rows);

  // multiply the spectrums;
  // the function handles packed spectrum representations well
  cv::mulSpectrums(tempA, tempB, tempA,0,true);

  // inverse transform
  // Even though all the result rows will be non-zero,
  // we need only the first corr.rows of them, and thus we
  // pass nonzeroRows == corr.rows
  cv::dft(tempA, tempA, cv::DFT_INVERSE + cv::DFT_SCALE, corr.rows);

  // now copy the result back to C.
  tempA(cv::Rect(0, 0, corr.cols, corr.rows)).copyTo(corr);
  matrix_to_image_flip(corr);
}

void correlation2D_no_preprocessing(const cv::Mat& M1,
                                    const cv::Mat& M2, cv::Mat& corr) {

  IMP_LOG(IMP::VERBOSE,"Computing 2D correlation no preprocessing "<<std::endl);

  IMP_USAGE_CHECK(((M1.rows==M2.rows) && (M1.cols == M2.cols)),
                  "em2d:correlation2D: Matrices have different size.");

  cv::Mat aux;
  cv::mulSpectrums(M1, M2, aux,0,true);
  cv::dft(aux, aux, cv::DFT_INVERSE + cv::DFT_SCALE, aux.rows);
  // now copy the result back to C.
  aux(cv::Rect(0, 0, corr.cols, corr.rows)).copyTo(corr);
  matrix_to_image_flip(corr);
}



void matrix_to_image_flip(cv::Mat &m) {
  int half_rows = m.rows/2;
  int half_columns = m.cols/2;
  int new_i,new_j;
  for (int i=0;i<m.rows;++i) {
    for (int j=0;j<half_columns;++j ) {
      if(i>=half_rows) { new_i=i-half_rows; } else { new_i=i+half_rows; }
      new_j=j+half_columns;
      std::swap(m.at<double>(i,j),m.at<double>(new_i,new_j));
    }
  }
}


IMPEM2D_END_NAMESPACE
