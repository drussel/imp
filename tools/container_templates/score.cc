/**
 *  \file CLASSNAMEScore.cpp  \brief Define CLASSNAMEScore
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 */

#include <IMP/CLASSNAMEScore.h>
#include <IMP/internal/utility.h>
#include <IMP/Restraint.h>
#include <IMP/macros.h>
#include <IMP/internal/container_helpers.h>
IMP_BEGIN_NAMESPACE

CLASSNAMEScore::CLASSNAMEScore(std::string name):
  Object(name)
{
  /* Implemented here rather than in the header so that PairScore
     symbols are present in the kernel DSO */
}

CLASSNAMEScoreRestraint::CLASSNAMEScoreRestraint(std::string name):
  Restraint(name){}

CLASSNAMEsScoreRestraint::CLASSNAMEsScoreRestraint(std::string name):
  Restraint(name){}


namespace {


  class CLASSNAMERestraint :
    public CLASSNAMEScoreRestraint
  {
    IMP::OwnerPointer<CLASSNAMEScore> ss_;
    STORAGETYPE v_;
  public:
    //! Create the restraint.
    /** This function takes the function to apply to the
        stored CLASSNAME and the CLASSNAME.
    */
    CLASSNAMERestraint(CLASSNAMEScore *ss,
                       ARGUMENTTYPE vt,
                       std::string name);

    CLASSNAMEScore* get_score() const {
      return ss_;
    }
    VARIABLETYPE get_argument() const {
      return v_;
    }

    IMP_RESTRAINT(CLASSNAMERestraint);
  };

  CLASSNAMERestraint
  ::CLASSNAMERestraint(CLASSNAMEScore *ss,
                       ARGUMENTTYPE vt,
                       std::string name):
    CLASSNAMEScoreRestraint(name),
    ss_(ss),
    v_(vt)
  {
  }

  double CLASSNAMERestraint
  ::unprotected_evaluate(DerivativeAccumulator *accum) const
  {
    IMP_OBJECT_LOG;
    IMP_CHECK_OBJECT(ss_);
    return ss_->evaluate(v_, accum);
  }

  ParticlesTemp CLASSNAMERestraint::get_input_particles() const
  {
    return IMP::internal::get_input_particles(ss_.get(), v_);
  }

  ContainersTemp CLASSNAMERestraint::get_input_containers() const
  {
    return IMP::internal::get_input_containers(ss_.get(), v_);
  }

  void CLASSNAMERestraint::do_show(std::ostream& out) const
  {
    out << "score " << ss_->get_name() << std::endl;
    out << "data " << IMP::internal::streamable(v_) << std::endl;
  }
}


Restraints
CLASSNAMEScore
::create_current_decomposition(ARGUMENTTYPE vt) const {
  return Restraints(1,
                    new CLASSNAMERestraint(const_cast<CLASSNAMEScore*>(this),
                                           vt,
                                           get_name()));
}

IMP_END_NAMESPACE
