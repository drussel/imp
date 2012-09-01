/**
 *  \file internal/swig.cpp
 *  \brief Functions for use in swig wrappers
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */
#include <IMP/internal/swig.h>
#include <IMP/internal/pdb.h>
IMP_BEGIN_INTERNAL_NAMESPACE


double _ConstRestraint
::unprotected_evaluate(IMP::DerivativeAccumulator *) const {
  return v_;
}
void _ConstRestraint::do_show(std::ostream &out) const {
  out << "value: " << v_ << std::endl;
}
ContainersTemp _ConstRestraint::get_input_containers() const {
  return ContainersTemp();
}
ParticlesTemp _ConstRestraint::get_input_particles() const {
  return ps_;
}

Restraints _ConstRestraint::do_create_decomposition() const {
  Restraints ret;
  for (unsigned int i=0; i< ps_.size(); ++i) {
    ret.push_back(new _ConstRestraint(v_/ps_.size(),
                                      ParticlesTemp(1,ps_[i])));
    ret.back()->set_last_score(v_/ps_.size());
  }
  return ret;
}



double _ConstSingletonScore::evaluate(Particle *,
                                      IMP::DerivativeAccumulator *) const {
  return v_;
}
void _ConstSingletonScore::do_show(std::ostream &out) const {
  out << "value: " << v_ << std::endl;
}
ContainersTemp
_ConstSingletonScore::get_input_containers(Particle *) const {
  return ContainersTemp();
}
ParticlesTemp _ConstSingletonScore::get_input_particles(Particle *p) const {
  return ParticlesTemp(1,p);
}



double _ConstPairScore::evaluate(const ParticlePair &,
                                 IMP::DerivativeAccumulator *) const {
  return v_;
}
void _ConstPairScore::do_show(std::ostream &out) const {
  out << "value: " << v_ << std::endl;
}
ContainersTemp
_ConstPairScore::get_input_containers(Particle*) const {
  return ContainersTemp();
}
ParticlesTemp _ConstPairScore::get_input_particles(Particle *p) const {
  return ParticlesTemp(1,p);
}


void _TrivialDecorator::show(std::ostream &out) const {
  out << "trivial decorator " << get_particle()->get_name();
}
void _TrivialDerivedDecorator::show(std::ostream &out) const {
  out << "trivial derived decorator " << get_particle()->get_name();
}

void _TrivialTraitsDecorator::show(std::ostream &out) const {
  out << "trivial traits decorator "<< get_particle()->get_name()
      << " with " << get_decorator_traits();
}



double _ConstOptimizer::do_optimize(unsigned int n) {
  for (unsigned int i=0; i < n; ++i) {
    get_model()->evaluate(false);
    update_states();
  }
  return get_model()->evaluate(false);
}

void _ConstOptimizer::do_show(std::ostream &) const {
}

int _overloaded_decorator(_TrivialDecorator) {
  return 0;
}
int _overloaded_decorator(_TrivialDerivedDecorator) {
  return 1;
}



std::string _test_ifile(base::TextInput a) {
  std::string read;
  while (true) {
    std::string cur;
    a.get_stream() >> cur;
    if (!a) break;
    read= read+cur;
  }
  std::cout << read;
  return read;
}
std::string _test_ofile(base::TextOutput a) {
  static_cast<std::ostream &>(a) << "hi\n"
                                 << " there, how are things"<< std::endl;
  return "hi\n";
}



std::string _test_ifile_overloaded(base::TextInput a, std::string) {
  return _test_ifile(a);
}
std::string _test_ofile_overloaded(base::TextOutput a, std::string) {
  return _test_ofile(a);
}
std::string _test_ifile_overloaded(base::TextInput a, int) {
  return _test_ifile(a);
}
std::string _test_ofile_overloaded(base::TextOutput a, int) {
  return _test_ofile(a);
}

IMPEXPORT ModelObjectsTemp
_pass_model_objects(const ModelObjectsTemp &p) {
  return p;
}



void _decorator_test(Particle*p) {
  std::cout << "hi " << p->get_name() << std::endl;
}

unsigned int _take_particles(const Particles &ps) {
  for (unsigned int i=0; i< ps.size(); ++i) {
    IMP_CHECK_OBJECT(ps[i]);
  }
  return ps.size();
}

unsigned int _take_particles(Model *, const Particles &ps) {
  for (unsigned int i=0; i< ps.size(); ++i) {
    IMP_CHECK_OBJECT(ps[i]);
  }
  return ps.size();
}

unsigned int _take_particles(Model *, const Particles &ps,
                             base::TextOutput) {
  for (unsigned int i=0; i< ps.size(); ++i) {
    IMP_CHECK_OBJECT(ps[i]);
  }
  return ps.size();
}
const Particles& _give_particles(Model *m) {
  static Particles ret;
  while(ret.size() <10) {
    ret.push_back(new Particle(m));
  }
  return ret;
}
const Particles& _pass_particles(const Particles &ps) {
  return ps;
}
Particle* _pass_particle(Particle* ps) {
  return ps;
}
const ParticlePair& _pass_particle_pair(const ParticlePair &pp) {
  for (unsigned int i=0; i< 2; ++i) {
    std::cout << pp[i]->get_name() << " ";
  }
  std::cout << std::endl;
  return pp;
}
Particles _give_particles_copy(Model *m) {
  Particles ret;
  while(ret.size() <10) {
    ret.push_back(new Particle(m));
  }
  return ret;
}
FloatKeys _pass_float_keys(const FloatKeys& in) {
  for (unsigned int i=0; i< in.size(); ++i) {
    std::cout << in[i] << " ";
  }
  return in;
}
Floats _pass_floats(const Floats& in) {
  for (unsigned int i=0; i< in.size(); ++i) {
    std::cout << in[i] << " ";
  }
  return in;
}
Ints _pass_ints( Ints in) {
  for (unsigned int i=0; i< in.size(); ++i) {
    std::cout << in[i] << " ";
  }
  return in;
}
const Strings& _pass_strings(const Strings& in) {
  std::cout << in << std::endl;
  return in;
}

const Particles &_pass(const Particles &p) {
  std::cout << p << std::endl;
  return p;
}
const Restraints &_pass(const Restraints &p) {
  std::cout << p << std::endl;
  return p;
}

const _TrivialDecorators &
_pass_decorators(const internal::_TrivialDecorators &p) {
  std::cout << p << std::endl;
  return p;
}

const _TrivialTraitsDecorators &
_pass_decorator_traits(const _TrivialTraitsDecorators &p) {
  std::cout << p << std::endl;
  return p;
}

ParticlePairsTemp
_pass_particle_pairs(const ParticlePairsTemp &p) {
  std::cout << p << std::endl;
  return get_as<ParticlePairsTemp>(p);
}

ParticleIndexPairs
_pass_particle_index_pairs(const ParticleIndexPairs &p) {
  std::cout << p << std::endl;
  return p;
}


DerivativePair
_pass_pair(const DerivativePair &p) {
  std::cout << p.first << " " << p.second << std::endl;
  return p;
}
IntsList _pass_ints_list(const IntsList &in) {
  std::cout << "IntsList of length " << in.size();
  return in;
}
IntsLists _pass_ints_lists(const IntsLists &in) {
  std::cout << "IntsLists of length " << in.size();
  return in;
}
std::pair<double, double>
_pass_plain_pair(std::pair<double, double> p) {
  std::cout << p.first << " " << p.second << std::endl;
  return p;
}


int _test_overload(const Particles &) {
  return 0;
}

int _test_overload(const Restraints &) {
  return 1;
}

int _test_intranges(const IntRanges &ips) {
  return ips.size();
}


IntRange _test_intrange(const IntRange &ips) {
  return ips;
}

IntRange _test_intrange() {
  return IntRange(-1,-1);
}

namespace {
  void test_log_1() {
    IMP_FUNCTION_LOG;
    IMP_LOG(SILENT, "Hi" << std::endl);
  }
  void test_log_0() {
    IMP_FUNCTION_LOG;
    test_log_1();
  }
}

void _test_log() {
  IMP_FUNCTION_LOG;
  test_log_0();
}

IMPEXPORT ParticlesTemp _create_particles_from_pdb(std::string name, Model*m) {
  return create_particles_from_pdb(name, m);
}


void _LogPairScore::do_show(std::ostream &) const {
}

Float _LogPairScore::evaluate(const ParticlePair &pp,
                                    DerivativeAccumulator *) const {
  if (map_.find(pp) == map_.end()) {
    map_[pp]=0;
  }
  ++map_[pp];
  return 0.;
}

 //! Get a list of all pairs (without multiplicity)
ParticlePairsTemp _LogPairScore::get_particle_pairs() const {
  ParticlePairsTemp ret;
  for (compatibility::map<ParticlePair, unsigned int>::const_iterator
           it = map_.begin(); it != map_.end(); ++it) {
    ret.push_back(it->first);
  }
  return ret;
}

IMP_END_INTERNAL_NAMESPACE
