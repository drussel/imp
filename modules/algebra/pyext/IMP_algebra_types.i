
%define IMP_SWIG_ALGEBRA_VALUE_D(Namespace, Namebase)
IMP_SWIG_VALUE_INSTANCE(Namespace, Namebase##1D, Namebase##D, Namebase##1Ds);
IMP_SWIG_VALUE_INSTANCE(Namespace, Namebase##2D, Namebase##D, Namebase##2Ds);
IMP_SWIG_VALUE_INSTANCE(Namespace, Namebase##3D, Namebase##D, Namebase##3Ds);
IMP_SWIG_VALUE_INSTANCE(Namespace, Namebase##4D, Namebase##D, Namebase##4Ds);
IMP_SWIG_VALUE_INSTANCE(Namespace, Namebase##5D, Namebase##D, Namebase##5Ds);
IMP_SWIG_VALUE_INSTANCE(Namespace, Namebase##6D, Namebase##D, Namebase##6Ds);
IMP_SWIG_VALUE_INSTANCE(Namespace, Namebase##KD, Namebase##D, Namebase##KDs);
IMP_SWIG_VALUE_IMPL(Namespace, Namebase##D<1>, Namebase##D, Namebase##test##1,Namebase##1Ds);
IMP_SWIG_VALUE_IMPL(Namespace, Namebase##D<2>, Namebase##D, Namebase##test##2,Namebase##2Ds);
IMP_SWIG_VALUE_IMPL(Namespace, Namebase##D<3>, Namebase##D, Namebase##test##3,Namebase##3Ds);
IMP_SWIG_VALUE_IMPL(Namespace, Namebase##D<4>, Namebase##D, Namebase##test##4,Namebase##4Ds);
IMP_SWIG_VALUE_IMPL(Namespace, Namebase##D<5>, Namebase##D, Namebase##test##5,Namebase##5Ds);
IMP_SWIG_VALUE_IMPL(Namespace, Namebase##D<6>, Namebase##D, Namebase##test##6,Namebase##6Ds);
IMP_SWIG_VALUE_IMPL(Namespace, Namebase##D<-1>, Namebase##D, Namebase##test##k, Namebase##KDs);
IMP_SWIG_VALUE_TEMPLATE(Namespace, Namebase##D);
%extend Namespace::Namebase##D {
  int __cmp__(const Namebase##D<D> &) const {
    IMP_THROW("Geometric primitives cannot be compared",
              IMP::ValueException);
  }
}
%enddef

%define IMP_SWIG_ALGEBRA_VALUE(Namespace, Name, PluralName)
IMP_SWIG_VALUE(Namespace, Name, PluralName);
%extend Namespace::Name {
  int __cmp__(const Name &) const {
    IMP_THROW("Geometric primitives cannot be compared",
              IMP::ValueException);
  }
}
%enddef


%define IMP_SWIG_ALGEBRA_OBJECT_D(Namespace, Namebase)
IMP_SWIG_OBJECT_INSTANCE(Namespace, Namebase##1D, Namebase##1D, Namebase##1Ds);
IMP_SWIG_OBJECT_INSTANCE(Namespace, Namebase##2D, Namebase##2D, Namebase##2Ds);
IMP_SWIG_OBJECT_INSTANCE(Namespace, Namebase##3D, Namebase##3D, Namebase##3Ds);
IMP_SWIG_OBJECT_INSTANCE(Namespace, Namebase##4D, Namebase##4D, Namebase##4Ds);
IMP_SWIG_OBJECT_INSTANCE(Namespace, Namebase##5D, Namebase##5D, Namebase##5Ds);
IMP_SWIG_OBJECT_INSTANCE(Namespace, Namebase##6D, Namebase##6D, Namebase##6Ds);
IMP_SWIG_OBJECT_INSTANCE(Namespace, Namebase##KD, Namebase##KD, Namebase##KDs);
IMP_SWIG_OBJECT_TEMPLATE(Namespace, Namebase##D);
%enddef

%define IMP_SWIG_ALGEBRA_TEMPLATE_D(Namespace, Namebase)
%template(Namebase##1D) Namespace::Namebase##D<1>;
%template(Namebase##2D) Namespace::Namebase##D<2>;
%template(Namebase##3D) Namespace::Namebase##D<3>;
%template(Namebase##4D) Namespace::Namebase##D<4>;
%template(Namebase##5D) Namespace::Namebase##D<5>;
%template(Namebase##6D) Namespace::Namebase##D<6>;
%template(Namebase##KD) Namespace::Namebase##D<-1>;
%template(_##Namebase##1Ds) ::std::vector< Namespace::Namebase##D<1> >;
%template(_##Namebase##2Ds) ::std::vector< Namespace::Namebase##D<2> >;
%template(_##Namebase##3Ds) ::std::vector< Namespace::Namebase##D<3> >;
%template(_##Namebase##4Ds) ::std::vector< Namespace::Namebase##D<4> >;
%template(_##Namebase##5Ds) ::std::vector< Namespace::Namebase##D<5> >;
%template(_##Namebase##6Ds) ::std::vector< Namespace::Namebase##D<6> >;
%enddef

%define IMP_SWIG_ALGEBRA_PRIVATE_TEMPLATE_D(Namespace, Namebase)
%template(_Namebase##1D) Namespace::Namebase##D<1>;
%template(_Namebase##2D) Namespace::Namebase##D<2>;
%template(_Namebase##3D) Namespace::Namebase##D<3>;
%template(_Namebase##4D) Namespace::Namebase##D<4>;
%template(_Namebase##5D) Namespace::Namebase##D<5>;
%template(_Namebase##6D) Namespace::Namebase##D<6>;
%template(_Namebase##KD) Namespace::Namebase##D<-1>;
%enddef

%define IMP_SWIG_ALGEBRA_TEMPLATE_OBJECT_D(Namespace, Namebase)
%template(Namebase##1D) Namespace::Namebase##D<1>;
%template(Namebase##2D) Namespace::Namebase##D<2>;
%template(Namebase##3D) Namespace::Namebase##D<3>;
%template(Namebase##4D) Namespace::Namebase##D<4>;
%template(Namebase##5D) Namespace::Namebase##D<5>;
%template(Namebase##6D) Namespace::Namebase##D<6>;
%template(Namebase##KD) Namespace::Namebase##D<-1>;
%enddef


%define IMP_SWIG_ALGEBRA_FUNCTION_D_D(ReturnType, function_name, Argument0)
%inline %{
  namespace IMP {
    namespace algebra {
  ReturnType##1D function_name(const Argument0##1D& a) {
  return function_name<1>(a);
}
  ReturnType##2D function_name(const Argument0##2D& a) {
  return function_name<2>(a);
}
ReturnType##3D function_name(const Argument0##3D& a) {
  return function_name<3>(a);
}
ReturnType##4D function_name(const Argument0##4D& a) {
  return function_name<4>(a);
}
ReturnType##5D function_name(const Argument0##5D& a) {
  return function_name<5>(a);
}
ReturnType##6D function_name(const Argument0##6D& a) {
  return function_name<6>(a);
}
ReturnType##KD function_name(const Argument0##KD& a) {
  return function_name<-1>(a);
}
    }
  }
%}
%enddef

%define IMP_SWIG_ALGEBRA_FUNCTION_D_DD(ReturnType, function_name, Argument0, Argument1)
%inline %{
namespace IMP {
namespace algebra {
ReturnType##1D function_name(const Argument0##1D& a, const Argument1##1D& b) {
  return function_name<1>(a,b);
}
ReturnType##2D function_name(const Argument0##2D& a, const Argument1##2D& b) {
  return function_name<2>(a,b);
}
ReturnType##3D function_name(const Argument0##3D& a, const Argument1##3D& b) {
  return function_name<3>(a,b);
}
ReturnType##4D function_name(const Argument0##4D& a, const Argument1##4D& b) {
  return function_name<4>(a,b);
}
ReturnType##5D function_name(const Argument0##5D& a, const Argument1##5D& b) {
  return function_name<5>(a,b);
}
ReturnType##6D function_name(const Argument0##6D& a, const Argument1##6D& b) {
  return function_name<6>(a,b);
}
ReturnType##KD function_name(const Argument0##KD& a, const Argument1##KD& b) {
  return function_name<-1>(a,b);
}
}
}
%}
%enddef


%define IMP_SWIG_ALGEBRA_FUNCTION_N_DD(ReturnType, function_name, Argument0, Argument1)
%inline %{
namespace IMP {
namespace algebra {
ReturnType function_name(const Argument0##1D& a, const Argument1##1D& b) {
  return function_name<1>(a,b);
}
ReturnType function_name(const Argument0##2D& a, const Argument1##2D& b) {
  return function_name<2>(a,b);
}
ReturnType function_name(const Argument0##3D& a, const Argument1##3D& b) {
  return function_name<3>(a,b);
}
ReturnType function_name(const Argument0##4D& a, const Argument1##4D& b) {
  return function_name<4>(a,b);
}
ReturnType function_name(const Argument0##5D& a, const Argument1##5D& b) {
  return function_name<5>(a,b);
}
ReturnType function_name(const Argument0##6D& a, const Argument1##6D& b) {
  return function_name<6>(a,b);
}
ReturnType function_name(const Argument0##KD& a, const Argument1##KD& b) {
  return function_name<-1>(a,b);
}
}
}
%}
%enddef

%define IMP_SWIG_ALGEBRA_FUNCTION_DS_DN(ReturnType, function_name, Argument0, Argument1)
%inline %{
  namespace IMP {
    namespace algebra {
ReturnType##1Ds function_name(const Argument0##1D& a, const Argument1& b) {
  return function_name<1>(a,b);
}
ReturnType##2Ds function_name(const Argument0##2D& a, const Argument1& b) {
  return function_name<2>(a,b);
}
ReturnType##3Ds function_name(const Argument0##3D& a, const Argument1& b) {
  return function_name<3>(a,b);
}
ReturnType##4Ds function_name(const Argument0##4D& a, const Argument1& b) {
  return function_name<4>(a,b);
}
ReturnType##5Ds function_name(const Argument0##5D& a, const Argument1& b) {
  return function_name<5>(a,b);
}
ReturnType##6Ds function_name(const Argument0##6D& a, const Argument1& b) {
  return function_name<6>(a,b);
}
ReturnType##KDs function_name(const Argument0##KD& a, const Argument1& b) {
  return function_name<-1>(a,b);
}
    }
  }
%}
%enddef


%define IMP_SWIG_ALGEBRA_FUNCTION_TEMPLATE_D(function_name)
namespace IMP {
namespace algebra {
%template(function_name##_1d) function_name##_d<1>;
%template(function_name##_2d) function_name##_d<2>;
%template(function_name##_3d) function_name##_d<3>;
%template(function_name##_4d) function_name##_d<4>;
%template(function_name##_5d) function_name##_d<5>;
%template(function_name##_6d) function_name##_d<6>;
}
}
%enddef
