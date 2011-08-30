/**
 *  \file SingletonScore.cpp  \brief Define SingletonScore
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 */

#include <IMP/SingletonScore.h>
#include <IMP/internal/utility.h>
#include <IMP/Restraint.h>
#include <IMP/macros.h>
#include <IMP/SingletonScore.h>
#include <IMP/internal/container_helpers.h>
IMP_BEGIN_NAMESPACE

SingletonScore::SingletonScore(std::string name):
  Object(name)
{
  /* Implemented here rather than in the header so that PairScore
     symbols are present in the kernel DSO */
}

SingletonScoreRestraint::SingletonScoreRestraint(std::string name):
  Restraint(name){}

SingletonsScoreRestraint::SingletonsScoreRestraint(std::string name):
  Restraint(name){}


namespace {


  class SingletonRestraint :
    public SingletonScoreRestraint
  {
    IMP::OwnerPointer<SingletonScore> ss_;
    Pointer<Particle> v_;
  public:
    //! Create the restraint.
    /** This function takes the function to apply to the
        stored Singleton and the Singleton.
    */
    SingletonRestraint(SingletonScore *ss,
                       Particle* vt,
                       std::string name);

    SingletonScore* get_score() const {
      return ss_;
    }
    Particle* get_argument() const {
      return v_;
    }

    IMP_RESTRAINT(SingletonRestraint);
  };

  SingletonRestraint
  ::SingletonRestraint(SingletonScore *ss,
                       Particle* vt,
                       std::string name):
    SingletonScoreRestraint(name),
    ss_(ss),
    v_(vt)
  {
  }

  double SingletonRestraint
  ::unprotected_evaluate(DerivativeAccumulator *accum) const
  {
    IMP_OBJECT_LOG;
    IMP_CHECK_OBJECT(ss_);
    return ss_->evaluate(v_, accum);
  }

  ParticlesTemp SingletonRestraint::get_input_particles() const
  {
    return IMP::internal::get_input_particles(ss_.get(), v_);
  }

  ContainersTemp SingletonRestraint::get_input_containers() const
  {
    return IMP::internal::get_input_containers(ss_.get(), v_);
  }

  void SingletonRestraint::do_show(std::ostream& out) const
  {
    out << "score " << ss_->get_name() << std::endl;
    out << "data " << IMP::internal::streamable(v_) << std::endl;
  }
}


Restraints
SingletonScore
::create_current_decomposition(Particle* vt) const {
  return Restraints(1,
                    new SingletonRestraint(const_cast<SingletonScore*>(this),
                                           vt,
                                           get_name()));
}

IMP_END_NAMESPACE
