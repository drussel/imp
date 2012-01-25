/**
 *  \file display_macros.h
 *  \brief macros for display classes
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */

#ifndef IMPDISPLAY_MACROS_H
#define IMPDISPLAY_MACROS_H

//! Define information for an TextWriter object
/** This macro declares the methods do_open, do_close, add_geometry
    and show, and defines the destructor and get_version_info.
*/
#define IMP_TEXT_WRITER(Name)                                           \
  Name(base::TextOutput of): TextWriter(of)                             \
  {do_open();}                                                          \
  Name(std::string name): TextWriter(name){                             \
    if (name.find("%1%") == std::string::npos) {                        \
      TextWriter::open();                                               \
    }                                                                   \
  }                                                                     \
  Name(const char* name): TextWriter(std::string(name)){                \
    if (std::string(name).find("%1%") == std::string::npos) {           \
      TextWriter::open();                                               \
    }                                                                   \
  }                                                                     \
  IMP_OBJECT_INLINE(Name,if (0) out << "Hi",do_close());                \
protected:                                                              \
 IMP_IMPLEMENT(virtual void do_open());                                 \
 IMP_IMPLEMENT(virtual void do_close())


#define IMP_WRITER(Name)                                                \
  IMP_OBJECT_INLINE(Name,if (0) out << "Hi",do_close());                \
protected:                                                              \
 IMP_IMPLEMENT(virtual void do_open());                                 \
 IMP_IMPLEMENT(virtual void do_close())


//! Define information for an Geometry object
#define IMP_GEOMETRY(Name)                              \
  IMP_IMPLEMENT(IMP::display::Geometries get_components() const);       \
  IMP_OBJECT(Name)




#define IMP_DISPLAY_GEOMETRY_DEF(Name, Type)                            \
  Name::Name(const Type &v): display::Geometry(#Name), v_(v){}          \
  Name::Name(const Type &v, const Color &c):                            \
      display::Geometry(c, #Name), v_(v) {}                             \
  Name::Name(const Type &v, const std::string n):                       \
      display::Geometry(n), v_(v) {}                                    \
  Name::Name(const Type &v, const Color &c, std::string n):             \
      display::Geometry(c,n), v_(v) {}                                  \
  IMP_IMPLEMENT_INLINE(display::Geometries Name::get_components() const, { \
    return display::Geometries(1, const_cast<Name*>(this));             \
    })                                                                  \
  IMP_IMPLEMENT_INLINE(void Name::do_show(std::ostream &out) const, {   \
    out << #Name << "Geometry: " << get_geometry();                     \
    });                                                                 \
  IMP_REQUIRE_SEMICOLON_NAMESPACE

#if defined(IMP_DOXYGEN) || defined(SWIG)
//! Define a geometric object using an IMP::algebra one
#define IMP_DISPLAY_GEOMETRY_DECL(Name, Type)                           \
  /** Display a geometric object.*/                                     \
  class IMPDISPLAYEXPORT Name: public display::Geometry {               \
  public:                                                               \
    Name(const Type &v);                                                \
    Name(const Type &v, const display::Color &c);                       \
    Name(const Type &v, const std::string n);                           \
    Name(const Type &v, const display::Color &c, std::string n);        \
    const Type& get_geometry() const {return v_;}                       \
    IMP_GEOMETRY(Name);                                                 \
  }                                                                     \


#define IMP_DISPLAY_GEOMETRY_DECOMPOSABLE_DECL(Name, Type)              \
  /** Display a compound geometric object.*/                            \
  class IMPDISPLAYEXPORT Name: public display::Geometry {               \
  public:                                                               \
    Name(const Type &v);                                                \
    Name(const Type &v, const display::Color &c);                       \
    Name(const Type &v, const std::string n);                           \
    Name(const Type &v, const display::Color &c, std::string n);        \
    IMP_GEOMETRY(Name);                                                 \
  }

#else

//! Define a geometric object using an IMP::algebra one
#define IMP_DISPLAY_GEOMETRY_DECL(Name, Type)                           \
  /** Display a geometric object.*/                                     \
  class IMPDISPLAYEXPORT Name: public display::Geometry {               \
    Type v_;                                                            \
  public:                                                               \
    Name(const Type &v);                                                \
    Name(const Type &v, const display::Color &c);                       \
    Name(const Type &v, const std::string n);                           \
    Name(const Type &v, const display::Color &c, std::string n);        \
    const Type& get_geometry() const {return v_;}                       \
    IMP_GEOMETRY(Name);                                                 \
  }                                                                     \


#define IMP_DISPLAY_GEOMETRY_DECOMPOSABLE_DECL(Name, Type)              \
  /** Display a compound geometric object.*/                            \
  class IMPDISPLAYEXPORT Name: public display::Geometry {               \
    Type v_;                                                            \
  public:                                                               \
    Name(const Type &v);                                                \
    Name(const Type &v, const display::Color &c);                       \
    Name(const Type &v, const std::string n);                           \
    Name(const Type &v, const display::Color &c, std::string n);        \
    const Type& get_geometry() const {return v_;}                       \
    IMP_GEOMETRY(Name);                                                 \
  }
#endif


#define IMP_DISPLAY_GEOMETRY_DECOMPOSABLE_DEF(Name, Type, decomp)       \
  Name::Name(const Type &v): display::Geometry(#Name), v_(v){}          \
  Name::Name(const Type &v, const display::Color &c):                   \
      display::Geometry(c, #Name), v_(v) {}                             \
  Name::Name(const Type &v, const std::string n):                       \
      display::Geometry(n), v_(v) {}                                    \
  Name::Name(const Type &v, const display::Color &c, std::string n):    \
      display::Geometry(c,n), v_(v) {}                                  \
  IMP_IMPLEMENT_INLINE(void Name::do_show(std::ostream &out) const, {   \
    out << #Name << "Geometry: "                                        \
        << get_geometry();                                              \
    });                                                                 \
  IMP_IMPLEMENT_INLINE(display::Geometries Name::get_components() const, { \
    display::Geometries ret;                                            \
    decomp;                                                             \
    return ret;                                                         \
    });                                                                 \
  IMP_REQUIRE_SEMICOLON_NAMESPACE

#define IMP_PARTICLE_GEOMETRY(Name, Decorator, action)                  \
  /** Display a particle.*/                                             \
  class Name##Geometry: public display::SingletonGeometry {             \
  public:                                                               \
  Name##Geometry(Particle* p): display::SingletonGeometry(p){}          \
  Name##Geometry(Decorator d): display::SingletonGeometry(d){}          \
  IMP_IMPLEMENT_INLINE(display::Geometries get_components() const, {    \
    display::Geometries ret;                                            \
    Decorator d(get_particle());                                        \
    action;                                                             \
    return ret;                                                         \
    });                                                                 \
  IMP_OBJECT_INLINE(Name##Geometry,                                     \
                    out <<  Decorator(get_particle())<< std::endl;,{}); \
  };                                                                    \
  /** Display multiple particles.*/                                     \
  class Name##sGeometry: public display::SingletonsGeometry {           \
  public:                                                               \
  Name##sGeometry(SingletonContainer* sc): display::SingletonsGeometry(sc){} \
  IMP_IMPLEMENT_INLINE(display::Geometries get_components() const, {    \
    display::Geometries ret;                                            \
    IMP_FOREACH_SINGLETON(get_container(), {                            \
        Decorator d(_1);                                                \
        action;                                                         \
      });                                                               \
    return ret;                                                         \
    });                                                                 \
  IMP_OBJECT_INLINE(Name##sGeometry,                                    \
                    out <<  get_container() << std::endl;,{});          \
  }


#define IMP_PARTICLE_PAIR_GEOMETRY(Name, Decorator, action)             \
  /** Display a pair of particles.*/                                    \
  class Name##Geometry: public display::PairGeometry {                  \
  public:                                                               \
  Name##Geometry(const ParticlePair &pp):                               \
  display::PairGeometry(pp){}                                           \
  IMP_IMPLEMENT_INLINE(display::Geometries get_components() const, {    \
    display::Geometries ret;                                            \
    Decorator d0(get_particle_pair()[0]);                               \
    Decorator d1(get_particle_pair()[1]);                               \
    action;                                                             \
    return ret;                                                         \
    })                                                                  \
  IMP_OBJECT_INLINE(Name##Geometry,                                     \
                    out <<  Decorator(get_particle_pair()[0])           \
                    << " " << Decorator(get_particle_pair()[1])         \
                    << std::endl;,{});                                  \
  };                                                                    \
  /** Display multiple pairs of particles.*/                            \
  class Name##sGeometry: public display::PairsGeometry {                \
  public:                                                               \
  Name##sGeometry(PairContainer* sc): display::PairsGeometry(sc){}      \
  IMP_IMPLEMENT_INLINE(display::Geometries get_components() const, {    \
    display::Geometries ret;                                            \
    IMP_FOREACH_PAIR(get_container(),{                                  \
        Decorator d0(_1[0]);                                            \
        Decorator d1(_1[1]);                                            \
        action;                                                         \
      });                                                               \
    return ret;                                                         \
    });                                                                 \
  IMP_OBJECT_INLINE(Name##sGeometry,                                    \
                    out <<  get_container() << std::endl;,{});          \
  }

#endif /* IMPDISPLAY_MACROS_H */
