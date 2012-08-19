/**
 *  \file IMP/algebra/VectorD.h   \brief Simple D vector class.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPALGEBRA_VECTOR_D_H
#define IMPALGEBRA_VECTOR_D_H

#include "algebra_config.h"
#include "GeometricPrimitiveD.h"
#include <IMP/base/types.h>
#include <IMP/base/base_macros.h>
#include <IMP/base/exception.h>
#include <IMP/base/utility.h>
#include <IMP/base/InputAdaptor.h>
#include "internal/vector.h"
#include <boost/static_assert.hpp>

#include <limits>
#include <cmath>

#if IMP_BUILD < IMP_FAST
#define IMP_VECTOR_CHECK check_vector()
#define IMP_VECTOR_CHECK_INDEX(i) check_index(i)
#define IMP_VECTOR_CHECK_COMPATIBLE(o) \
  check_compatible_vector(o); o.check_vector()
#else
#define IMP_VECTOR_CHECK
#define IMP_VECTOR_CHECK_INDEX(i)
#define IMP_VECTOR_CHECK_COMPATIBLE(o)
#endif


IMPALGEBRA_BEGIN_NAMESPACE
//! A Cartesian vector in D-dimensions.
/** Store a vector of Cartesian coordinates. It supports all expected
    mathematical operators, including using * for the dot product.
    \see Vector3D
    \see Vector2D

    \geometry
 */
template <int D>
class VectorD: public GeometricPrimitiveD<D>
{
  void check_vector() const {
    IMP_USAGE_CHECK(!data_.get_is_null(),
                    "Attempt to use uninitialized vector.");
  }
  template <int OD>
  void check_compatible_vector(const VectorD<OD> &o) const {
    IMP_USAGE_CHECK(o.get_dimension() == get_dimension(),
                    "Dimensions don't match: "
                    << get_dimension() << " vs "
                    << o.get_dimension());
  }
  void check_index(unsigned int i) const {
    IMP_INTERNAL_CHECK(i < data_.get_dimension(),
                       "Invalid component of vector requested: "
                       << i << " of " <<get_dimension());
  }
public:
#if !defined(SWIG) && !defined(IMP_DOXYGEN)
  template <int OD>
  VectorD(const VectorD<OD> &o) {
    BOOST_STATIC_ASSERT(D==-1 || OD==-1 || D==OD);
    IMP_USAGE_CHECK(D==-1
                    || o.get_dimension() ==static_cast<unsigned int>(D),
                    "Dimensions don't match in conversion");
    data_.set_coordinates(o.coordinates_begin(),
                          o.coordinates_end());
  }
  template <int OD>
  VectorD & operator=(const VectorD<OD> &o) {
    BOOST_STATIC_ASSERT(D==-1 || OD==-1 || D==OD);
    IMP_USAGE_CHECK(D==-1 || o.get_dimension() ==D,
                    "Dimensions don't match in conversion");
    data_.set_coordinates(o.coordinates_begin(),
                          o.coordinates_end());
  }
#endif

  /** \throw base::ValueException if f.size() is not appropriate.*/
  VectorD(const Floats &f) {
    if (D!=-1 && static_cast<int>(f.size()) != D) {
      IMP_THROW("Expected " << D << " but got " << f.size(),
                base::ValueException);
    }
    data_.set_coordinates(f.begin(), f.end());
  }

  /** The distance between b and e must be equal to D.
   */
  template <class It>
  VectorD(It b, It e) {
    data_.set_coordinates(b,e);
  }

  //! Initialize the 1-vector from its value.
  explicit VectorD(double x) {
  /* Note that MSVC gets confused with static asserts if we try to subclass
     VectorD, as we do for example in the various IMP::display Geometry
     subclasses, so replace with runtime checks. */
#if defined(IMP_SWIG_WRAPPER) || defined(_MSC_VER)
    IMP_USAGE_CHECK(D==1 || D==-1,
                    "Need " << D << " to construct a "
                    << D << "-vector.");
#else
    BOOST_STATIC_ASSERT(D==1);
#endif
    data_.set_coordinates(&x, &x+1);
  }

  //! Initialize a 2-vector from separate x,y values.
  VectorD(double x, double y) {
#if defined(IMP_SWIG_WRAPPER) || defined(_MSC_VER)
    IMP_USAGE_CHECK(D==2 || D==-1, "Need " << D << " to construct a "
              << D << "-vector.");
#else
    BOOST_STATIC_ASSERT(D==2);
#endif
    double d[]={x,y};
    data_.set_coordinates(d, d+2);
  }

  //! Initialize a 3-vector from separate x,y,z values.
  VectorD(double x, double y, double z) {
#ifdef IMP_SWIG_WRAPPER
    IMP_USAGE_CHECK(D==3 || D==-1, "Need " << D << " to construct a "
              << D << "-vector.");
#else
    BOOST_STATIC_ASSERT(D==3);
#endif
    double d[]={x,y,z};
    data_.set_coordinates(d, d+3);
  }

  //! Initialize a 4-vector from separate w,x,y,z values.
  VectorD(double x0, double x1, double x2, double x3) {
#if defined(IMP_SWIG_WRAPPER) || defined(_MSC_VER)
    IMP_USAGE_CHECK(D==4 || D==-1, "Need " << D << " to construct a "
              << D << "-vector.");
#else
    BOOST_STATIC_ASSERT(D==4);
#endif
    double d[]={x0, x1, x2, x3};
    data_.set_coordinates(d, d+4);
  }

  //! Default constructor
  VectorD() {
  }

  /** Return the ith Cartesian coordinate. In 3D use [0] to get
      the x coordinate etc.*/
  inline double operator[](unsigned int i) const {
    IMP_VECTOR_CHECK_INDEX(i);
    IMP_VECTOR_CHECK;
    return data_.get_data()[i];
  }
  /** Return the ith Cartesian coordinate. In 3D use [0] to get
      the x coordinate etc. */
  inline double& operator[](unsigned int i) {
    IMP_VECTOR_CHECK_INDEX(i);
    return data_.get_data()[i];
  }

  double get_scalar_product(const VectorD<D> &o) const {
    IMP_VECTOR_CHECK_COMPATIBLE(o);
    IMP_VECTOR_CHECK;
    double ret=0;
    for (unsigned int i=0; i< get_dimension(); ++i) {
      ret += operator[](i)* o.operator[](i);
    }
    return ret;
  }

  double get_squared_magnitude() const {
    return get_scalar_product(*this);
  }

  double get_magnitude() const {
    return std::sqrt(get_squared_magnitude());
  }

  VectorD get_unit_vector() const {
    double mag = get_magnitude();
    // avoid division by zero
    mag = std::max(mag, static_cast<double>(1e-12));
    return operator/(mag);
  }
#ifndef IMP_DOXYGEN
  double operator*(const VectorD<D> &o) const {
    IMP_VECTOR_CHECK_COMPATIBLE(o);
    return get_scalar_product(o);
  }

  VectorD operator*(double s) const {
    IMP_VECTOR_CHECK;
    VectorD ret=*this;
    ret*=s;
    return ret;
  }

  VectorD operator/(double s) const {
    IMP_VECTOR_CHECK;
    VectorD ret=*this;
    ret/=s;
    return ret;
  }

  VectorD operator-() const {
    IMP_VECTOR_CHECK;
    VectorD ret=*this;
    for (unsigned int i=0; i<get_dimension(); ++i) {
      ret[i] = -ret[i];
    }
    return ret;
  }

  VectorD operator-(const VectorD &o) const {
    IMP_VECTOR_CHECK_COMPATIBLE(o);
    IMP_VECTOR_CHECK;
    VectorD ret=*this;
    ret-=o;
    return ret;
  }

  VectorD operator+(const VectorD &o) const {
    IMP_VECTOR_CHECK_COMPATIBLE(o);
    IMP_VECTOR_CHECK;
    VectorD ret=*this;
    ret+=o;
    return ret;
  }

  VectorD& operator+=(const VectorD &o) {
    IMP_VECTOR_CHECK_COMPATIBLE(o);
    IMP_VECTOR_CHECK;
    for (unsigned int i=0; i<get_dimension(); ++i) {
      operator[](i) += o[i];
    }
    return *this;
  }

  VectorD& operator-=(const VectorD &o) {
    IMP_VECTOR_CHECK_COMPATIBLE(o);
    IMP_VECTOR_CHECK;
    for (unsigned int i=0; i<get_dimension(); ++i) {
      operator[](i) -= o[i];
    }
    return *this;
  }

  VectorD& operator/=(double f) {
    IMP_VECTOR_CHECK;
    for (unsigned int i=0; i<get_dimension(); ++i) {
      operator[](i) /= f;
    }
    return *this;
  }

  VectorD& operator*=(double f) {
    IMP_VECTOR_CHECK;
    for (unsigned int i=0; i<get_dimension(); ++i) {
      operator[](i) *= f;
    }
    return *this;
  }

  void show(std::ostream &out, std::string delim,
            bool parens=true) const {
    IMP_VECTOR_CHECK;
    if (parens) out << "(";
    for (unsigned int i=0; i<get_dimension(); ++i) {
      out << operator[](i);
      if (i != get_dimension()-1) {
        out << delim;
      }
    }
    if (parens) out << ")";
  }
  IMP_SHOWABLE_INLINE(VectorD, show(out, ", "););
#endif

#ifndef SWIG
  typedef double* CoordinateIterator;
  CoordinateIterator coordinates_begin() {return data_.get_data();}
  CoordinateIterator coordinates_end() {
    return data_.get_data()+get_dimension();
  }
  typedef const double* CoordinateConstIterator;
  CoordinateConstIterator coordinates_begin() const {
    return data_.get_data();
  }
  CoordinateConstIterator coordinates_end() const {
    return data_.get_data()+get_dimension();
  }
#endif

#ifndef SWIG
  // For some reason, this method breaks IMP::atom::get_rmsd() in Python, so
  // hide it from SWIG
  Floats get_coordinates() const {
    return Floats(coordinates_begin(), coordinates_end());
  }
#endif

#if !defined(IMP_DOXYGEN) && !defined(SWIG)
  const double *get_data() const {return data_.get_data();}
#endif
  unsigned int get_dimension() const {
    return data_.get_dimension();
  }
private:

  internal::VectorData<double, D, false> data_;
};

#ifndef IMP_DOXYGEN


template <int D>
inline std::ostream &operator<<(std::ostream &out, const VectorD<D> &v) {
  v.show(out);
  return out;
}

template <int D>
inline std::istream &operator>>(std::istream &in, VectorD<D> &v) {
  for (unsigned int i=0; i< D; ++i) {
    in >> v[i];
  }
  return in;
}

#endif

//! lexicographic comparison of two vectors
/** Note that this is not very reliable and probably should not be used.
    \relates VectorD
 */
template <int D>
inline int compare(const VectorD<D> &a, const VectorD<D> &b) {
  IMP_USAGE_CHECK(a.get_dimension()== b.get_dimension(),
                  "Dimensions don't match.");
  for (unsigned int i=0; i< a.get_dimension(); ++i) {
    if (a[i] < b[i]) return -1;
    else if (a[i] > b[i]) return 1;
  }
  return 0;
}

/** \relates VectorD */
template <int D>
inline VectorD<D> operator*(double s, const VectorD<D> &o) {
  return o*s;
}

//! compute the squared distance between two vectors
/** \relates VectorD
 */
template <int D>
inline double get_squared_distance(const VectorD<D> &v1, const VectorD<D> &v2) {
  return (v1-v2).get_squared_magnitude();
}

//! compute the distance between two vectors
/** \relates VectorD
 */
template <int D>
inline double get_distance(const VectorD<D> &v1, const VectorD<D> &v2) {
  return std::sqrt(get_squared_distance(v1, v2));
}

//! Return the basis vector for the given coordinate
/** Return the unit vector pointing in the direction of the requested
    coordinate. That is
    \code
    get_basis_vector_d<3>(2)== Vector3D(0,0,1);
    \endcode
    \relatesalso VectorD
 */
template <int D>
inline VectorD<D> get_basis_vector_d(unsigned int coordinate) {
  IMP_USAGE_CHECK(coordinate<D, "There are only " << D << " basis vectors");
  double vs[D];
  for (unsigned int i=0; i< D; ++i) {
    if (i==coordinate) vs[i]=1;
    else vs[i]=0;
  }
  return VectorD<D>(vs, vs+D);
}

//! Return a dynamically sized basis vector
inline VectorD<-1> get_basis_vector_kd( int D,
                                    unsigned int coordinate) {
  IMP_USAGE_CHECK(D>0, "D must be positive");
  IMP_USAGE_CHECK(coordinate<static_cast<unsigned int>(D),
                  "There are only " << D << " basis vectors");
  boost::scoped_array<double> vs(new double[D]);
  for (int i=0; i< D; ++i) {
    if (i==static_cast<int>(coordinate)) vs[i]=1;
    else vs[i]=0;
  }
  return VectorD<-1>(vs.get(), vs.get()+D);
}

//! Return a vector of zeros
template <int D>
inline VectorD<D> get_zero_vector_d() {
  IMP_USAGE_CHECK(D>0, "D must be positive");
  Floats vs(D, 0);
  return VectorD<D>(vs.begin(), vs.end());
}

//! Return a dynamically sized vector of zeros
inline VectorD<-1> get_zero_vector_kd( int D) {
  IMP_USAGE_CHECK(D>0, "D must be positive");
  Floats vs(D, 0);
  return VectorD<-1>(vs.begin(), vs.end());
}


//! Return a vector of ones (or another constant)
inline VectorD<-1> get_ones_vector_kd(unsigned int D, double v=1) {
  IMP_USAGE_CHECK(D>0, "D must be positive");
  boost::scoped_array<double> vv(new double[D]);
  for ( int i=0; i< D; ++i) {
    vv[i]=v;
  }
  return VectorD<-1>(vv.get(), vv.get()+D);
}

//! Return a vector of ones (or another constant)
template <int D>
inline VectorD<D> get_ones_vector_d(double v=1) {
  IMP_USAGE_CHECK(D>0, "D must be positive");
  boost::scoped_array<double> vv(new double[D]);
  for (unsigned int i=0; i< D; ++i) {
    vv[i]=v;
  }
  return VectorD<D>(vv.get(), vv.get()+D);
}


#ifndef SWIG

/** \name Norms
    We define a number of standard, \f$L^p\f$, norms on VectorD.
    - \f$L^1\f$ is the Manhattan distance, the sum of the components
    - \f$L^2\f$ is the standard Euclidean length
    - \f$L^{\inf}\f$ is the maximum of the components
    @{
*/

template <int D>
inline double get_l2_norm(const VectorD<D> &v) {
  return v.get_magnitude();
}

template <int D>
inline double get_l1_norm(const VectorD<D> &v) {
  double n=std::abs(v[0]);
  for (unsigned int i=1; i< v.get_dimension(); ++i) {
    n+= std::abs(v[i]);
  }
  return n;
}

template <int D>
inline double get_linf_norm(const VectorD<D> &v) {
  double n=std::abs(v[0]);
  for (unsigned int i=1; i< v.get_dimension(); ++i) {
    n= std::max(n, std::abs(v[i]));
  }
  return n;
}

/** @} */

#ifndef IMP_DOXYGEN

template <int D>
struct SpacesIO
{
  const VectorD<D> &v_;
  SpacesIO(const VectorD<D> &v): v_(v){}
};

template <int D>
struct CommasIO
{
  const VectorD<D> &v_;
  CommasIO(const VectorD<D> &v): v_(v){}
};
template <int D>
inline std::ostream &operator<<(std::ostream &out, const SpacesIO<D> &s)
{
  s.v_.show(out, " ", false);
  return out;
}
template <int D>
inline std::ostream &operator<<(std::ostream &out, const CommasIO<D> &s)
{
  s.v_.show(out, ", ", false);
  return out;
}

//! Use this before outputing to delimited vector entries with a space
/** std::cout << spaces_io(v);
    produces "1.0 2.0 3.0"
    \relatesalso VectorD
 */
template <int D>
inline SpacesIO<D> spaces_io(const VectorD<D> &v) {
  return SpacesIO<D>(v);
}




//! Use this before outputing to delimited vector entries with a comma
/** std::cout << commas_io(v);
    produces "1.0, 2.0, 3.0"
    \relatesalso VectorD
 */
template <int D>
inline CommasIO<D> commas_io(const VectorD<D> &v) {
  return CommasIO<D>(v);
}
#endif // doxygen

#endif  //swig
/** \relates VectorD */
typedef VectorD<1> Vector1D;
/** \relates VectorD */
typedef base::Vector<VectorD<1> > Vector1Ds;
/** \relates VectorD */
typedef VectorD<2> Vector2D;
/** \relates VectorD */
typedef base::Vector<VectorD<2> > Vector2Ds;
/** \relates VectorD */
typedef VectorD<3> Vector3D;
/** \relates VectorD */
typedef base::Vector<VectorD<3> > Vector3Ds;
/** \relates VectorD */
typedef VectorD<4> Vector4D;
/** \relates VectorD */
typedef base::Vector<VectorD<4> > Vector4Ds;
/** \relates VectorD */
typedef VectorD<5> Vector5D;
/** \relates VectorD */
typedef base::Vector<VectorD<5> > Vector5Ds;
/** \relates VectorD */
typedef VectorD<6> Vector6D;
/** \relates VectorD */
typedef base::Vector<VectorD<6> > Vector6Ds;
/** \relates VectorD */
typedef VectorD<-1> VectorKD;
/** \relates VectorD */
typedef base::Vector<VectorD<-1> > VectorKDs;


/** \relates VectorD */
template <int D>
inline const VectorD<D> &get_vector_d_geometry(const VectorD<D> &g) {return g;}
/** \relates VectorD */
template <int D>
inline void set_vector_d_geometry(VectorD<D> &g, const VectorD<D> &v) {g=v;}


/** \relatesalso VectorD
    Return the vector that is the elementwise product of the two.
*/
template <int D>
inline VectorD<D> get_elementwise_product(const algebra::VectorD<D>& a,
                                   const algebra::VectorD<D>& b) {
  VectorD<D> ret(a);
  for (unsigned int i=0; i< ret.get_dimension(); ++i) {
    ret[i]*=b[i];
  }
  return ret;
}

/** \relatesalso VectorD
    Return the vector that is the elementwise product of the two.
*/
template <int D>
inline VectorD<D> get_elementwise_product(const Ints& a,
                                   const algebra::VectorD<D>& b) {
  IMP_USAGE_CHECK(a.size()== b.get_dimension(),
                  "Dimensions don't match,");
  VectorD<D> ret(b);
  for (unsigned int i=0; i< ret.get_dimension(); ++i) {
    ret[i]*=a[i];
  }
  return ret;
}




/** A class to flexibly accept vectors as inputs to functions.
    \relates VectorD
 */
template <int D>
class VectorInputD: public VectorD<D>, public base::InputAdaptor {
public:
  VectorInputD(const VectorD<D> &v): VectorD<D>(v){}
  VectorInputD(const Floats &v): VectorD<D>(v){}
};

/** Also accept floating point values for Vector1Ds

    \relates VectorD
 */
template <>
class VectorInputD<1>: public VectorD<1>, public base::InputAdaptor {
public:
  VectorInputD(const VectorD<1> &v): VectorD<1>(v){}
  VectorInputD(const Floats &v): VectorD<1>(v){}
  VectorInputD(double v): VectorD<1>(v){}
};

/** \relates VectorInputD */
typedef VectorInputD<1> VectorInput1D;
/** \relates VectorInputD */
typedef base::Vector<VectorInputD<1> > VectorInput1Ds;
/** \relates VectorInputD */
typedef VectorInputD<2> VectorInput2D;
/** \relates VectorInputD */
typedef base::Vector<VectorInputD<2> > VectorInput2Ds;
/** \relates VectorInputD */
typedef VectorInputD<3> VectorInput3D;
/** \relates VectorInputD */
typedef base::Vector<VectorInputD<3> > VectorInput3Ds;
/** \relates VectorInputD */
typedef VectorInputD<4> VectorInput4D;
/** \relates VectorInputD */
typedef base::Vector<VectorInputD<4> > VectorInput4Ds;
/** \relates VectorInputD */
typedef VectorInputD<5> VectorInput5D;
/** \relates VectorInputD */
typedef base::Vector<VectorInputD<5> > VectorInput5Ds;
/** \relates VectorInputD */
typedef VectorInputD<6> VectorInput6D;
/** \relates VectorInputD */
typedef base::Vector<VectorInputD<6> > VectorInput6Ds;
/** \relates VectorInputD */
typedef VectorInputD<-1> VectorInputKD;
/** \relates VectorInputD */
typedef base::Vector<VectorInputD<-1> > VectorInputKDs;






IMPALGEBRA_END_NAMESPACE

#endif  /* IMPALGEBRA_VECTOR_D_H */
