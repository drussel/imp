%module(directors="1") IMP_misc

%{
#include "IMP.h"
#include "IMP/misc.h"
%}

%include "kernel/pyext/IMP_macros.i"
%include "kernel/pyext/IMP_exceptions.i"

/* Ignore shared object import/export stuff */
#define IMPMISCEXPORT
#define IMPMISCLOCAL

%include "std_vector.i"
%include "std_string.i"
%include "std_except.i"

/* Get definitions of kernel base classes (but do not wrap) */
%import "kernel/pyext/IMP.i"

namespace IMP::misc {
  IMP_OWN_FIRST_CONSTRUCTOR(BondCoverPairScore)
  IMP_OWN_FIRST_SECOND_CONSTRUCTOR(RefineOncePairScore)
  IMP_OWN_FIRST_CONSTRUCTOR(TunnelSingletonScore)
}

/* Wrap our own classes */
%include "IMP/misc/WormLikeChain.h"
%include "IMP/misc/BondCoverParticleRefiner.h"
%include "IMP/misc/ChildrenParticleRefiner.h"
%include "IMP/misc/RefineOncePairScore.h"
%include "IMP/misc/TunnelSingletonScore.h"
%include "IMP/misc/CoverBondsScoreState.h"
