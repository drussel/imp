/**
 *  \file display_macros.h
 *  \brief macros for display classes
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 */

#ifndef IMPDISPLAY_MACROS_H
#define IMPDISPLAY_MACROS_H

//! Define information for an TextWriter object
/** This macro declares the methods do_open, do_close, add_geometry
    and show, and defines the destructor and get_version_info.
*/
#define IMP_TEXT_WRITER(Name)                                           \
  Name(TextOutput of): TextWriter(of)                                   \
  {do_open();}                                                          \
  Name(std::string name): TextWriter(name){do_open();}                  \
  IMP_OBJECT_INLINE(Name,if (0) out << "Hi",do_close());                \
protected:                                                              \
 using Writer::handle;                                                  \
 virtual void do_open();                                                \
 virtual void do_close()


#define IMP_WRITER(Name)                                                \
  IMP_OBJECT_INLINE(Name,if (0) out << "Hi",do_close());                \
protected:                                                              \
 using Writer::handle;                                                  \
 virtual void do_open();                                                \
 virtual void do_close()


//! Define information for an Geometry object
#define IMP_GEOMETRY(Name)                              \
  IMP::display::Geometries get_components() const;      \
  IMP_OBJECT(Name)




#define IMP_DISPLAY_GEOMETRY_DEF(Name, Type)                            \
  Name::Name(const Type &v): Geometry(#Name), v_(v){}                   \
  Name::Name(const Type &v, const Color &c):                            \
    Geometry(c, #Name), v_(v) {}                                        \
  Name::Name(const Type &v, const std::string n):                       \
    Geometry(n), v_(v) {}                                               \
  Name::Name(const Type &v, const Color &c, std::string n):             \
    Geometry(c,n), v_(v) {}                                             \
  Geometries Name::get_components() const {                             \
    return Geometries(1, const_cast<Name*>(this));                      \
  }                                                                     \
  void Name::do_show(std::ostream &out) const {                         \
    out << #Name << "Geometry: " << get_geometry();                     \
  }                                                                     \
  IMP_REQUIRE_SEMICOLON_NAMESPACE

#if defined(IMP_DOXYGEN) || defined(SWIG)
//! Define a geometric object using an IMP::algebra one
#define IMP_DISPLAY_GEOMETRY_DECL(Name, Type)                           \
  /** Display a geometric object.*/                                     \
  class IMPDISPLAYEXPORT Name: public Geometry {                        \
  public:                                                               \
    Name(const Type &v);                                                \
    Name(const Type &v, const Color &c);                                \
    Name(const Type &v, const std::string n);                           \
    Name(const Type &v, const Color &c, std::string n);                 \
    const Type& get_geometry() const {return v_;}                       \
    IMP_GEOMETRY(Name);                                                 \
  }                                                                     \


#define IMP_DISPLAY_GEOMETRY_DECOMPOSABLE_DECL(Name, Type)              \
  /** Display a compound geometric object.*/                            \
  class IMPDISPLAYEXPORT Name: public Geometry {                        \
  public:                                                               \
    Name(const Type &v);                                                \
    Name(const Type &v, const Color &c);                                \
    Name(const Type &v, const std::string n);                           \
    Name(const Type &v, const Color &c, std::string n);                 \
    IMP_GEOMETRY(Name);                                                 \
  }

#else

//! Define a geometric object using an IMP::algebra one
#define IMP_DISPLAY_GEOMETRY_DECL(Name, Type)                           \
  /** Display a geometric object.*/                                     \
  class IMPDISPLAYEXPORT Name: public Geometry {                        \
    Type v_;                                                            \
  public:                                                               \
    Name(const Type &v);                                                \
    Name(const Type &v, const Color &c);                                \
    Name(const Type &v, const std::string n);                           \
    Name(const Type &v, const Color &c, std::string n);                 \
    const Type& get_geometry() const {return v_;}                       \
    IMP_GEOMETRY(Name);                                                 \
  }                                                                     \


#define IMP_DISPLAY_GEOMETRY_DECOMPOSABLE_DECL(Name, Type)              \
  /** Display a compound geometric object.*/                            \
  class IMPDISPLAYEXPORT Name: public Geometry {                        \
    Type v_;                                                            \
  public:                                                               \
    Name(const Type &v);                                                \
    Name(const Type &v, const Color &c);                                \
    Name(const Type &v, const std::string n);                           \
    Name(const Type &v, const Color &c, std::string n);                 \
    const Type& get_geometry() const {return v_;}                       \
    IMP_GEOMETRY(Name);                                                 \
  }
#endif


#define IMP_DISPLAY_GEOMETRY_DECOMPOSABLE_DEF(Name, Type, decomp)       \
  Name::Name(const Type &v): Geometry(#Name), v_(v){}                   \
  Name::Name(const Type &v, const Color &c):                            \
    Geometry(c, #Name), v_(v) {}                                        \
  Name::Name(const Type &v, const std::string n):                       \
    Geometry(n), v_(v) {}                                               \
  Name::Name(const Type &v, const Color &c, std::string n):             \
    Geometry(c,n), v_(v) {}                                             \
  void Name::do_show(std::ostream &out) const {                         \
    out << #Name << "Geometry: "                                        \
        << get_geometry();                                              \
  }                                                                     \
  Geometries Name::get_components() const {                             \
    Geometries ret;                                                     \
    decomp;                                                             \
    return ret;                                                         \
  }                                                                     \
  IMP_REQUIRE_SEMICOLON_NAMESPACE

#define IMP_PARTICLE_GEOMETRY(Name, Decorator, action)                  \
  /** Display a particle.*/                                             \
  class Name##Geometry: public SingletonGeometry {                      \
  public:                                                               \
  Name##Geometry(Particle* p): SingletonGeometry(p){}                   \
  Name##Geometry(Decorator d): SingletonGeometry(d){}                   \
  Geometries get_components() const {                                   \
    Geometries ret;                                                     \
    Decorator d(get_particle());                                        \
    action;                                                             \
    return ret;                                                         \
  }                                                                     \
  IMP_OBJECT_INLINE(Name##Geometry,                                     \
                    out <<  Decorator(get_particle())<< std::endl;,{}); \
  };                                                                    \
  /** Display multiple particles.*/                                     \
  class Name##sGeometry: public SingletonsGeometry {                    \
  public:                                                               \
  Name##sGeometry(SingletonContainer* sc): SingletonsGeometry(sc){}     \
  Geometries get_components() const {                                   \
    Geometries ret;                                                     \
    for (unsigned int i=0;                                              \
         i< get_container()->get_number_of_particles();                 \
         ++i) {                                                         \
      Decorator d(get_container()->get_particle(i));                    \
      action;                                                           \
    }                                                                   \
    return ret;                                                         \
  }                                                                     \
  IMP_OBJECT_INLINE(Name##sGeometry,                                    \
                    out <<  get_container() << std::endl;,{});          \
  }


#define IMP_PARTICLE_TRAITS_GEOMETRY(Name, Decorator, TraitsName,       \
                                     traits_name,action)                \
  /** Display a particle.*/                                             \
  class Name##Geometry: public SingletonGeometry {                      \
    TraitsName traits_;                                                 \
  public:                                                               \
  Name##Geometry(Decorator d): SingletonGeometry(d),                    \
    traits_(d.get_##traits_name()){}                                    \
  Geometries get_components() const {                                   \
    Geometries ret;                                                     \
    Decorator d(get_particle(), traits_);                               \
    action;                                                             \
    return ret;                                                         \
  }                                                                     \
  IMP_OBJECT_INLINE(Name##Geometry,                                     \
                    out <<  Decorator(get_particle(), traits_)          \
                    << std::endl;,{});                                  \
  };                                                                    \
  /** Display multiple particles.*/                                     \
  class Name##sGeometry: public SingletonsGeometry {                    \
    TraitsName traits_;                                                 \
  public:                                                               \
  Name##sGeometry(SingletonContainer* sc, TraitsName tr):               \
  SingletonsGeometry(sc), traits_(tr){}                                 \
  Geometries get_components() const {                                   \
    Geometries ret;                                                     \
    for (unsigned int i=0;                                              \
         i< get_container()->get_number_of_particles();                 \
         ++i) {                                                         \
      Decorator d(get_container()->get_particle(i), traits_);           \
      action;                                                           \
    }                                                                   \
    return ret;                                                         \
  }                                                                     \
  IMP_OBJECT_INLINE(Name##sGeometry,                                    \
                    out <<  get_container() << std::endl;,{});          \
  }


#define IMP_PARTICLE_PAIR_GEOMETRY(Name, Decorator, action)             \
  /** Display a pair of particles.*/                                    \
  class Name##Geometry: public PairGeometry {                           \
  public:                                                               \
  Name##Geometry(const ParticlePair &pp):                               \
  PairGeometry(pp){}                                                    \
  Geometries get_components() const {                                   \
    Geometries ret;                                                     \
    Decorator d0(get_particle_pair()[0]);                               \
    Decorator d1(get_particle_pair()[1]);                               \
    action;                                                             \
    return ret;                                                         \
  }                                                                     \
  IMP_OBJECT_INLINE(Name##Geometry,                                     \
                    out <<  Decorator(get_particle_pair()[0])           \
                    << " " << Decorator(get_particle_pair()[1])         \
                    << std::endl;,{});                                  \
  };                                                                    \
  /** Display multiple pairs of particles.*/                            \
  class Name##sGeometry: public PairsGeometry {                         \
  public:                                                               \
  Name##sGeometry(PairContainer* sc): PairsGeometry(sc){}               \
  Geometries get_components() const {                                   \
    Geometries ret;                                                     \
    for (unsigned int i=0;                                              \
         i< get_container()->get_number_of_particle_pairs();            \
         ++i) {                                                         \
      Decorator d0(get_container()->get_particle_pair(i)[0]);           \
      Decorator d1(get_container()->get_particle_pair(i)[1]);           \
      action;                                                           \
    }                                                                   \
    return ret;                                                         \
  }                                                                     \
  IMP_OBJECT_INLINE(Name##sGeometry,                                    \
                    out <<  get_container() << std::endl;,{});          \
  }

#endif /* IMPDISPLAY_MACROS_H */
