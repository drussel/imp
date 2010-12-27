/**
 *  \file example/ExampleTemplateClass.h
 *  \brief Show how to manage a template class with python.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPEXAMPLE_EXAMPLE_TEMPLATE_CLASS_H
#define IMPEXAMPLE_EXAMPLE_TEMPLATE_CLASS_H

#include "example_config.h"

#include <IMP/utility.h>
#include <IMP/algebra/VectorD.h>

IMPEXAMPLE_BEGIN_NAMESPACE

//! A line segment templated on the dimension.
/** Like the underlying
    algebra::VectorD data, is not initialized when the default
    constructor is used. Since it is a small class, it is designed to
    be allocated on the stack and to be passed by value (or
    const &).

    The class should be named SegmentD, but it is an demostration.

    The following command indicates to doxygen to mark the class
    as being one whose members are left uninitialized by the
    default constructor:
    \ingroup uninitialized_default

    The source code is as follows:
    \include ExampleTemplateClass.h
*/
template <unsigned int D>
class ExampleTemplateClassD
{
  IMP::algebra::VectorD<D> eps_[2];
public:
  ExampleTemplateClassD(){}
  /** Since it is a simple object, there is no reason to provide
      methods to change the data.
  */
  ExampleTemplateClassD(const IMP::algebra::VectorD<D> &a,
                       const IMP::algebra::VectorD<D> &b){
    eps_[0]= a;
    eps_[1]= b;
  }
  //! Get one of the endpoints
  const IMP::algebra::VectorD<D>& get_point(unsigned int i) const {
    IMP_USAGE_CHECK(i < 2, "The endpoint index can only be 0 or 1");
    return eps_[i];
  }

  IMP_SHOWABLE_INLINE(ExampleTemplateClassD, out << eps_[0] << " " << eps_[1];);
};

typedef ExampleTemplateClassD<3> ExampleTemplateClass3D;
typedef std::vector<ExampleTemplateClassD<3> > ExampleTemplateClass3Ds;

// Make it so the C++ operator<< can be used. The _D means that it is
// is templated on the dimension. See the docs for other, related macros.
IMP_OUTPUT_OPERATOR_D(ExampleTemplateClassD);

IMPEXAMPLE_END_NAMESPACE

#endif  /* IMPEXAMPLE_EXAMPLE_TEMPLATE_CLASS_H */
