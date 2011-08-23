/**
 *  \file Matrix3D.h
 *  \brief Management of 3D matrices (volumes) of data
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
*/

#ifndef IMPALGEBRA_MATRIX_3D_H
#define IMPALGEBRA_MATRIX_3D_H

#include "IMP/base_types.h"
#include "IMP/exception.h"
#include "MultiArray.h"
#include <complex>


IMPALGEBRA_BEGIN_NAMESPACE

//! Template class for managing 3D matrices. This class is based on
//! boost multi_array.
/** This class is primarily aimed at providing support for 3D bitmap
    images. See GridD for general spatial grids if that is what you
    are after.

  Check MultiArray class for a list of added functionality
**/
template<typename T>
class Matrix3D: public MultiArray<T,3>
{
public:
    typedef boost::multi_array_types::index index;
    typedef MultiArray<T,3> MA3;
    typedef Matrix3D<T> This;

  //! Empty constructor
  Matrix3D() : MA3() {}

  //! Constructor
  Matrix3D(int Zdim, int Ydim, int Xdim) : MA3() {
    this->resize(Zdim, Ydim, Xdim);
  }

  // Copy constructor
  Matrix3D(const This& v) : MA3() {
    this->reshape(v);
    MA3::copy((MA3 &)v);
  }

  //! Resizes the matrix
  /**
   * \param[in] Zdim Number of slices
   * \param[in] Ydim Number of rows
   * \param[in] Xdim Number of columns
   */
  void resize(int Zdim, int Ydim, int Xdim) {
    typename This::extent_gen extents;
    MA3::resize(extents[Zdim][Ydim][Xdim]);
  }

  //! Resizes the matrix copying the size of a given one
  /**
   * \param[in] v Matrix3D whose size to copy
   */
  template<typename T1>
  void resize(const Matrix3D<T1> &v) {
    this->resize(v.shape()[0], v.shape()[1], v.shape()[2]);
  }

  //! Reshapes the matrix copying the size and range of a given one
  /**
   * \param[in] v Matrix3D whose shape to copy
   */
  template<typename T1>
  void reshape(const Matrix3D<T1>& v) {
    boost::array<index,3> shape,start;
    int dims=v.num_dimensions();
    for(int i=0;i<dims;i++) {
      shape[i]=v.shape()[i];
      start[i]=v.index_bases()[i];
    }
    resize(shape[0], shape[1], shape[2]);
    this->reindex(start);
 }


  //! Pad with a given value. Padding is defined as doubling the size
  //! in each dimension and fill the new values with the value.
  /**
    \param[out] padded the output MultiArray
    \param[in] val the value to pad with
  **/
  void pad(This& padded,T val) {
    padded.resize(2*this->get_size(0),2*this->get_size(1),2*this->get_size(2));
    // Copy values
    padded.fill_with_value(val);
    std::vector<index> idx(3),idx_for_padded(3);
    while (internal::roll_inds(idx, this->shape(),this->index_bases())) {
      for(int i=0;i<3;i++) {
        idx_for_padded[i]=idx[i]+(int)this->get_size(i)/2;
      }
      padded(idx_for_padded)=(*this)(idx);
    }
  }

  //! Pad the MultiArray. Padding is defined as doubling the size
  //! in each dimension and fill the new values with the previous average value.
  /**
    \param[in] padded the MultiArray padded
  **/
  void pad(This& padded) {
    double avg = this->compute_avg();
    this->pad(padded,avg);
  }

  //! Cast values
  template<typename T1>
  void cast_values(Matrix3D<T1> &out) {
    out.resize(*this);
    for(unsigned int i=0; i<this->num_elements(); i++) {
      out.data()[i] = (T1)this->data()[i];
    }
  }


  //! Access operator. The returned element is the LOGICAL element of the
  //! matrix, NOT the direct one
  /**
   * \param[in] k first index
   * \param[in] j second index
   * \param[in] i third index
   */
  T& operator()(int k,int j, int i) const {
    if (this->get_start(0) <= k && k <= this->get_finish(0) &&
        this->get_start(1) <= j && j <= this->get_finish(1) &&
        this->get_start(2) <= i && i <= this->get_finish(2)) {
      return (T&)(*this)[k][j][i];
    } else {
      String msg = "Matri3D::(): Index out of range." ;
      throw ValueException(msg.c_str());
    }
  }

  //! Access operator. The returned element is the LOGICAL element of the
  //! matrix, NOT the direct one
  /**
   * \param[in] idx must be a class supporting access via []
   */
  template<typename T1>
  T& operator()(T1& idx) const {
    return (T&)((*this)[idx[0]][idx[1]][idx[2]]);
  }

  //! Returns the number of slices in the matrix
  int get_number_of_slices() const {
    return (int)this->get_size(0);
  }

  //! Returns the number of rows in the matrix
  int get_number_of_rows() const {
    return (int)this->get_size(1);
  }

  //! Returns the number of columns in the matrix
  int get_number_of_columns() const {
    return (int)this->get_size(2);
  }

  IMP_SHOWABLE_INLINE(Matrix3D,
                      { out << "3D matrix (" << get_number_of_slices() << ", "
                            << get_number_of_rows() << ", "
                            << get_number_of_columns() << ")"; });

  void operator=(const This& v) {
    this->reshape(v);
    MA3::copy((MA3 &)v);
  }

  //! Sum operator
  This operator+(const This& v) const {
    This result;
    result.resize(*this);
    internal::operate_arrays(*this, v, result, '+');
    return result;
  }

  //! Minus operator
  This operator-(const This& v) const {
    This result;
    result.resize(*this);
    internal::operate_arrays(*this, v, result, '-');
    return result;
  }

  //! Multiplication operator
  This operator*(const This& v) const {
    This result;
    result.resize(*this);
    internal::operate_arrays(*this, v, result, '*');
    return result;
  }

  //! Division operator
  This operator/(const This& v) const {
    This result;
    result.resize(*this);
    internal::operate_arrays(*this, v, result, '/');
    return result;
  }

  //! Sum operator for an array and a scalar
  This operator+(const T& v) const {
    This result;
    result.resize(*this);
    internal::operate_array_and_scalar(*this, v, result, '+');
    return result;
  }

  //! Minus operator for an array and a scalar
  This operator-(const T& v) const {
    This result;
    result.resize(*this);
    internal::operate_array_and_scalar(*this, v, result, '-');
    return result;
  }

  //! Multiplication operator for an array and a scalar
  This operator*(const T& v) const {
    This result;
    result.resize(*this);
    internal::operate_array_and_scalar(*this, v, result, '*');
    return result;
  }

  //! Division operator for an array and a scalar
  This operator/(const T& v) const {
    This result;
    result.resize(*this);
    internal::operate_array_and_scalar(*this, v, result, '/');
    return result;
  }

  //! Addition operator
  This& operator+=(const This& v) {
    internal::operate_arrays(*this, v, *this, '+');
    return *this;
  }

  //! Substraction operator
  This& operator-=(const This& v) {
    internal::operate_arrays(*this, v, *this, '-');
    return *this;
  }

  //! Multiplication operator
  This& operator*=(const This& v) {
    internal::operate_arrays(*this, v, *this, '*');
    return *this;
  }

  //! Division operator
  This& operator/=(const This& v) {
    internal::operate_arrays(*this, v, *this, '/');
    return *this;
  }

  //! Addition operator for an array and a scalar
  This& operator+=(const T& v) {
    internal::operate_array_and_scalar(*this, v, *this, '+');
    return *this;
  }

  //! Substraction operator for an array and a scalar
  This& operator-=(const T& v) {
    internal::operate_array_and_scalar(*this, v, *this, '-');
    return *this;
  }

  //! Multiplication operator for an array and a scalar
  This& operator*=(const T& v) {
    internal::operate_array_and_scalar(*this, v, *this, '*');
    return *this;
  }

  //! Division operator for an array and a scalar
  This& operator/=(const T& v) {
    internal::operate_array_and_scalar(*this, v, *this, '/');
    return *this;
  }

  //! Sum operator for a scalar and an array
  friend This operator+(const T& X, const This& a1) {
    This result;
    result.resize(a1);
    internal::operate_scalar_and_array(X,a1,result,"+");
    return result;
  }

  //! Minus operator for a scalar and an array
  friend This operator-(const T& X, const This& a1) {
    This result;
    result.resize(a1);
    internal::operate_scalar_and_array(X,a1,result,"-");
    return result;
  }

  //! Multiplication operator for a scalar and an array
  friend This operator*(const T& X, const This& a1) {
    This result;
    result.resize(a1);
    internal::operate_scalar_and_array(X,a1,result,"*");
    return result;
  }

  //! Division operator for a scalar and an array
  friend This operator/(const T& X, const This& a1) {
    This result;
    result.resize(a1);
    internal::operate_scalar_and_array(X,a1,result,"/");
    return result;
  }
  std::string get_name() const {
    return "Matrix2D";
  }
bool get_is_valid() const {
  return true;
  }

protected:
};

IMPALGEBRA_END_NAMESPACE

#endif  /* IMPALGEBRA_MATRIX_3D_H */
