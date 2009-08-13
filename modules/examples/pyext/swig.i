/* Wrap our own classes */
%include "IMP/examples/ExampleRestraint.h"
%include "IMP/examples/ExampleDecorator.h"
%include "IMP/examples/ExampleUnaryFunction.h"
%include "IMP/examples/ExampleRefCounted.h"

namespace IMP {
  namespace examples {
   IMP_DECORATORS(ExampleDecorator, ExampleDecorators, Particles)
  }
}