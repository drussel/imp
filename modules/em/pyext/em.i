%module(directors="1") "IMP.em"

%{
#include "IMP.h"
#include "IMP/em.h"
#include "IMP/algebra.h"
#include "IMP/core.h"
#include "IMP/atom.h"
%}

%include "kernel/pyext/IMP_macros.i"

/* Ignore shared object import/export stuff */
#define EMDLLEXPORT
#define EMDLLLOCAL

/* Ignore things to prevent SWIG warning about them */
%ignore DensityMap::operator =;

%include "em_config.i"

%include "std_vector.i"
%include "std_string.i"
%include "std_except.i"

/* Make selected classes extensible in Python */
%feature("director") IMP::em::IMPParticlesAccessPoint;

/* Get definitions of kernel base classes (but do not wrap) */
%import "kernel/pyext/IMP.i"

/* Get definitions of algebra base classes (but do not wrap) */
%import "modules/algebra/pyext/algebra.i"
%import "modules/core/pyext/core.i"
%import "modules/atom/pyext/atom.i"

/* Get definitions of EMBED base classes (but do not wrap) */
%import "ParticlesAccessPoint.h"
%import "MapReaderWriter.h"
%import "DensityMap.h"

/* Don't use the exception handlers defined in the kernel */
%exception;

/* Wrap our own classes */
%include "IMP/em/IMPParticlesAccessPoint.h"
%include "IMP/em/FitRestraint.h"
%include "IMP/em/converters.h"
%include "IMP/em/EMproject.h"
%include "IMP/em/SpiderHeader.h"
%include "IMP/em/ImageHeader.h"
%include "IMP/em/ImageReaderWriter.h"
%include "IMP/em/SpiderReaderWriter.h"
%include "IMP/em/Volume.h"
%include "IMP/em/Image.h"

/* Allow runtime casting of Restraint* objects to FitRestraint* */
namespace IMP {
  namespace em {
    %extend FitRestraint {
      static FitRestraint* cast(Restraint *r) {
        return dynamic_cast<IMP::em::FitRestraint *>(r);
      }
    }

    %template(project) ::IMP::em::project<float>;
    %template(imageReaderWriter) ::IMP::em::ImageReaderWriter<float>;
    %template(spiderImageReaderWriter)
                          ::IMP::em::SpiderImageReaderWriter<float>;
    %template(image) ::IMP::em::Image<float>;
    %template(volume) ::IMP::em::Volume<float>;

  }
}
