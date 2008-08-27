%module(directors="1") IMP

%{
#include "IMP.h"
%}

%include "std_vector.i"
%include "std_map.i"
%include "std_string.i"
%include "std_pair.i"

%include "IMP_macros.i"
%include "IMP_exceptions.i"
%include "IMP_keys.i"

%include "typemaps.i"

namespace IMP {
  %typemap(out) std::pair<Float,Float> {
     PyObject *tup= PyTuple_New(2);
     PyTuple_SetItem(tup, 0, PyFloat_FromDouble($1.first));
     PyTuple_SetItem(tup, 1, PyFloat_FromDouble($1.second));
     $result= tup;
  }
}


%pythoncode %{
def check_particle(p, a):
   if (not p.get_is_active()):
      raise ValueError("Inactive Particle")
   if (type(a)() == a):
      raise IndexError("Cannot use default Index")
   if (not p.has_attribute(a)):
      raise IndexError("Particle does not have attribute")
%}

namespace IMP {
  // need to special case particle so can't add this to macro
  IMP_OWN_FIRST_CONSTRUCTOR(DistanceRestraint)
  IMP_OWN_FIRST_CONSTRUCTOR(AngleRestraint)
  IMP_OWN_FIRST_CONSTRUCTOR(DihedralRestraint)
  IMP_OWN_FIRST_CONSTRUCTOR(TorusRestraint)
  IMP_OWN_FIRST_CONSTRUCTOR(NonbondedRestraint)
  IMP_OWN_FIRST_CONSTRUCTOR(BondDecoratorRestraint)
  IMP_OWN_FIRST_CONSTRUCTOR(SingletonListRestraint)
  IMP_OWN_FIRST_CONSTRUCTOR(PairListRestraint)
  IMP_OWN_FIRST_CONSTRUCTOR(TripletChainRestraint)
  IMP_OWN_FIRST_CONSTRUCTOR(PairChainRestraint)
  IMP_OWN_FIRST_CONSTRUCTOR(ConnectivityRestraint)
  IMP_OWN_FIRST_CONSTRUCTOR(DistancePairScore)
  IMP_OWN_FIRST_CONSTRUCTOR(TransformedDistancePairScore)
  IMP_OWN_FIRST_CONSTRUCTOR(BondCoverPairScore)
  IMP_OWN_FIRST_CONSTRUCTOR(SphereDistancePairScore)
  IMP_OWN_FIRST_SECOND_CONSTRUCTOR(RefineOncePairScore)
  IMP_OWN_FIRST_CONSTRUCTOR(DistanceToSingletonScore)
  IMP_OWN_FIRST_CONSTRUCTOR(AttributeSingletonScore)
  IMP_OWN_FIRST_CONSTRUCTOR(TunnelSingletonScore)
  IMP_OWN_FIRST_CONSTRUCTOR(AngleTripletScore)
  IMP_SET_OBJECT(MonteCarlo, set_local_optimizer)
  IMP_SET_OBJECT(TypedPairScore, set_pair_score)

  %pythonprepend Particle::get_value %{
        check_particle(args[0], args[1])
  %}
  %pythonprepend Particle::get_is_optimized %{
        check_particle(args[0], args[1])
  %}
  %pythonprepend Particle::set_is_optimized %{
        check_particle(args[0], args[1])
  %}
  %pythonprepend Particle::set_value %{
        check_particle(args[0], args[1])
  %}
  %pythonprepend Particle::add_to_derivative %{
        check_particle(args[0], args[1])
  %}
  %pythonprepend Particle::get_derivative %{
        check_particle(args[0], args[1])
  %}
  %pythonprepend Particle::add_attribute %{
        # special case since we don't want to check that the attribute is there
        if (not args[0].get_is_active()):
           raise ValueError("Inactive Particle")
        elif (type(args[1])() == args[1]):
           raise IndexError("Cannot use default Index")
        elif (args[0].has_attribute(args[1])):
           raise IndexError("Particle already has attribute")

  %}

  // special case since particles are ref counted
  %extend Model {
    Particles get_particles() const {
      IMP::Particles ret(self->particles_begin(), self->particles_end());
      return ret;
    }
  }
  IMP_CONTAINER_SWIG(Model, ScoreState, score_state)
  IMP_CONTAINER_SWIG(Model, Restraint, restraint)
  IMP_CONTAINER_SWIG(RestraintSet, Restraint, restraint)
  IMP_CONTAINER_SWIG(Optimizer, OptimizerState, optimizer_state)
  IMP_ADD_OBJECT(MonteCarlo, add_mover)
  IMP_ADD_OBJECTS(MonteCarlo, add_movers)
  IMP_ADD_OBJECT(NonbondedListScoreState, add_bonded_list)
  IMP_ADD_OBJECTS(NonbondedListScoreState, add_bonded_lists)
}

%feature("ref")   Particle "$this->ref();"
%feature("unref") Particle "$this->unref(); if (! $this->get_has_ref()) delete $this;"


/* Don't wrap internal functions */
%ignore IMP::internal::evaluate_distance_pair_score;
%ignore IMP::internal::check_particles_active;

/* Make selected classes extensible in Python */
%feature("director") IMP::UnaryFunction;
%feature("director") IMP::Restraint;
%feature("director") IMP::ScoreState;
%feature("director") IMP::OptimizerState;
%feature("director") IMP::SingletonScore;
%feature("director") IMP::PairScore;
%feature("director") IMP::TripletScore;
%feature("director") IMP::Optimizer;
%feature("director") IMP::ParticleRefiner;

%include "IMP/base_types.h"
%include "IMP/Object.h"
%include "IMP/RefCountedObject.h"
%include "IMP/Index.h"
%include "IMP/VersionInfo.h"
%include "IMP/UnaryFunction.h"
%include "IMP/unary_functions/Harmonic.h"
%include "IMP/unary_functions/HarmonicLowerBound.h"
%include "IMP/unary_functions/HarmonicUpperBound.h"
%include "IMP/unary_functions/OpenCubicSpline.h"
%include "IMP/unary_functions/ClosedCubicSpline.h"
%include "IMP/unary_functions/Cosine.h"
%include "IMP/unary_functions/Linear.h"
%include "IMP/unary_functions/WormLikeChain.h"
%include "IMP/DerivativeAccumulator.h"
%include "Restraint.i"
%include "IMP/ScoreState.h"
%include "IMP/OptimizerState.h"
%include "IMP/log.h"
%include "IMP/Model.h"
%include "IMP/PairScore.h"
%include "IMP/ParticleRefiner.h"
%include "IMP/SingletonScore.h"
%include "IMP/TripletScore.h"
%include "Particle.i"
%include "Vector3D.i"
%include "IMP/DecoratorBase.h"
%include "IMP/decorators/bond_decorators.h"
%include "IMP/decorators/HierarchyDecorator.h"
%include "IMP/decorators/MolecularHierarchyDecorator.h"
%include "IMP/decorators/NameDecorator.h"
%include "IMP/decorators/ResidueDecorator.h"
%include "IMP/decorators/XYZDecorator.h"
%include "IMP/decorators/AtomDecorator.h"
%include "IMP/ParticleRefiner.h"
%include "IMP/particle_refiners/BondCoverParticleRefiner.h"
%include "IMP/particle_refiners/ChildrenParticleRefiner.h"
%include "IMP/Optimizer.h"
%include "IMP/optimizers/SteepestDescent.h"
%include "IMP/optimizers/ConjugateGradients.h"
%include "IMP/optimizers/MolecularDynamics.h"
%include "IMP/optimizers/BrownianDynamics.h"
%include "IMP/optimizers/Mover.h"
%include "IMP/optimizers/MoverBase.h"
%include "IMP/optimizers/MonteCarlo.h"
%include "IMP/optimizers/movers/BallMover.h"
%include "IMP/optimizers/movers/NormalMover.h"
%include "IMP/optimizers/states/VRMLLogOptimizerState.h"
%include "IMP/optimizers/states/CMMLogOptimizerState.h"
%include "IMP/optimizers/states/VelocityScalingOptimizerState.h"
%include "IMP/pair_scores/DistancePairScore.h"
%include "IMP/pair_scores/RefineOncePairScore.h"
%include "IMP/pair_scores/SphereDistancePairScore.h"
%include "IMP/pair_scores/TypedPairScore.h"
%include "IMP/pair_scores/TransformedDistancePairScore.h"
%include "IMP/particle_refiners/BondCoverParticleRefiner.h"
%include "IMP/particle_refiners/ChildrenParticleRefiner.h"
%include "IMP/singleton_scores/DistanceToSingletonScore.h"
%include "IMP/singleton_scores/AttributeSingletonScore.h"
%include "IMP/singleton_scores/TunnelSingletonScore.h"
%include "IMP/triplet_scores/AngleTripletScore.h"
%include "IMP/score_states/BondedListScoreState.h"
%include "IMP/score_states/MaxChangeScoreState.h"
%include "IMP/score_states/NonbondedListScoreState.h"
%include "IMP/score_states/AllNonbondedListScoreState.h"
%include "IMP/score_states/BondDecoratorListScoreState.h"
%include "IMP/score_states/BipartiteNonbondedListScoreState.h"
%include "IMP/score_states/GravityCenterScoreState.h"
%include "IMP/score_states/CoverBondsScoreState.h"
%include "IMP/restraints/AngleRestraint.h"
%include "IMP/restraints/BondDecoratorRestraint.h"
%include "IMP/restraints/ConnectivityRestraint.h"
%include "IMP/restraints/ConstantRestraint.h"
%include "IMP/restraints/DihedralRestraint.h"
%include "IMP/restraints/DistanceRestraint.h"
%include "IMP/restraints/NonbondedRestraint.h"
%include "IMP/restraints/PairChainRestraint.h"
%include "IMP/restraints/PairListRestraint.h"
%include "IMP/restraints/RestraintSet.h"
%include "IMP/restraints/SingletonListRestraint.h"
%include "IMP/restraints/TripletChainRestraint.h"

namespace IMP {
  %template(ParticleIndex) Index<ParticleTag>;
  %template(RestraintIndex) Index<RestraintTag>;
  %template(ScoreStateIndex) Index<ScoreStateTag>;
  %template(OptimizerStateIndex) Index<OptimizerStateTag>;
  %template(MoverIndex) Index<Mover>;
  %template(BondedListIndex) Index<BondedListScoreState>;
  %template(show_named_hierarchy) show<NameDecorator>;
  %template(show_molecular_hierarchy) show<MolecularHierarchyDecorator>;
  %template(Particles) ::std::vector<Particle*>;
  %template(ParticlesList) ::std::vector<Particles>;
  %template(ParticlePair) ::std::pair<IMP::Particle*, IMP::Particle*>;
  %template(ParticlePairs) ::std::vector<ParticlePair>;
  %template(Restraints) ::std::vector<Restraint*>;
  %template(ScoreStates) ::std::vector<ScoreState*>;
  %template(OptimizerStates) ::std::vector<OptimizerState*>;
  %template(ParticleIndexes) ::std::vector<ParticleIndex>;
  %template(BondDecorators) ::std::vector<BondDecorator>;
  %template(MolecularHiearchyDecorators) ::std::vector<MolecularHierarchyDecorator>;
  %template(FloatKeys) ::std::vector<FloatKey>;
  %template(StringKeys) ::std::vector<StringKey>;
  %template(IntKeys) ::std::vector<IntKey>;
  %template(ParticleKeys) ::std::vector<ParticleKey>;
  %template(Floats) ::std::vector<Float>;
  %template(Strings) ::std::vector<String>;
  %template(Ints) ::std::vector<Int>;
}
