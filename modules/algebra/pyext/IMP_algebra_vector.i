
/* Provide our own implementations for some operators */
%ignore IMP::algebra::VectorD::operator[];
%ignore IMP::algebra::VectorD::operator+=;
%ignore IMP::algebra::VectorD::operator*=;
%ignore IMP::algebra::VectorD::operator/=;
%ignore IMP::algebra::VectorD::operator-=;

/* Make sure that we return the original Python object from C++ inplace
   operators (not a new Python proxy around the same C++ object) */
namespace IMP {
 namespace algebra {
  %feature("shadow") VectorD::__iadd__(const IMP::algebra::VectorD<D> &) %{
    def __iadd__(self, *args):
        $action(self, *args)
        return self
  %}
  %feature("shadow") VectorD::__imul__(double) %{
    def __imul__(self, *args):
        $action(self, *args)
        return self
  %}
  %feature("shadow") VectorD::__idiv__(double) %{
    def __idiv__(self, *args):
        $action(self, *args)
        return self
  %}
  %feature("shadow") VectorD::__isub__(const IMP::algebra::VectorD<D> &) %{
    def __isub__(self, *args):
        $action(self, *args)
        return self
  %}
 }
}

%extend IMP::algebra::VectorD {
  double __getitem__(unsigned int index) const {
    if (index >= D) throw IMP::IndexException("");
    return self->operator[](index);
  }
  void __setitem__(unsigned int index, double val) {
    self->operator[](index) = val;
  }
  /* Ignore C++ return value from inplace operators, so that SWIG does not
     generate a new SWIG wrapper for the return value (see above). */
  void __iadd__(const IMP::algebra::VectorD<D> &o) { self->operator+=(o); }
  void __imul__(double f) { self->operator*=(f); }
  void __idiv__(double f) { self->operator/=(f); }
  void __isub__(const IMP::algebra::VectorD<D> &o) { self->operator-=(o); }
  unsigned int __len__() { return self->get_dimension(); }
  const IMP::algebra::VectorD<D> __rmul__(double f) const {return self->operator*(f);}
};
