%module(directors="1") IMPEM
%{
#include "IMP.h"
#include "../IMPParticlesAccessPoint.h"
#include "../EMFitRestraint.h"
%}

/* Ignore shared object import/export stuff */
#define IMPDLLEXPORT
#define IMPDLLLOCAL

%include "std_vector.i"
%include "std_string.i"
%include "std_except.i"

%feature("director");

namespace IMP {
  class Model;
  typedef float Float;
  class FloatKey;
  class DerivativeAccumulator;
  class Restraint;
}

%include "../IMPParticlesAccessPoint.h"
%include "../EMFitRestraint.h"
