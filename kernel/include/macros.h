/**
 *  \file IMP/macros.h    \brief Various general useful macros for IMP.
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 *
 */

#ifndef IMP_MACROS_H
#define IMP_MACROS_H

#ifdef IMP_DOXYGEN
//! Hide something from doxygen
#define IMP_NO_DOXYGEN(x)
//! Only show something to doxygen
#define IMP_ONLY_DOXYGEN(x) x
#else
#define IMP_NO_DOXYGEN(x) x
#define IMP_ONLY_DOXYGEN(x)
#endif

#if defined(IMP_SWIG_WRAPPER) || defined(SWIG)
#define IMP_NO_SWIG(x)
#else
//! Hide the line when SWIG is compiled or parses it
#define IMP_NO_SWIG(x) x
#endif

#ifdef IMP_DOXYGEN
//! Implement comparison in a class using a compare function
/** The macro requires that This be defined as the type of the current class.
    The compare function should take a const This & and return -1, 0, 1 as
    appropriate.
 */
#define IMP_COMPARISONS
#else
#define IMP_COMPARISONS                                                 \
  bool operator==(const This &o) const {                                \
    return (compare(o) == 0);                                           \
  }                                                                     \
  bool operator!=(const This &o) const {                                \
    return (compare(o) != 0);                                           \
  }                                                                     \
  bool operator<(const This &o) const {                                 \
    return (compare(o) <0);                                             \
  }                                                                     \
  bool operator>(const This &o) const {                                 \
    return (compare(o) > 0);                                            \
  }                                                                     \
  bool operator>=(const This &o) const {                                \
    return !(compare(o) < 0);                                           \
  }                                                                     \
  bool operator<=(const This &o) const {                                \
    return !(compare(o) > 0);                                           \
  }                                                                     \
  IMP_NO_SWIG(template <class T> friend int compare(const T&a, const T&b);
#endif

#ifdef IMP_DOXYGEN                                                         \
//! Implement comparison in a class using field as the variable to compare
/** The macro requires that This be defined as the type of the current class.
 */
#define IMP_COMPARISONS_1(field)
#else
#define IMP_COMPARISONS_1(field)                                        \
  bool operator==(const This &o) const {                                \
    return (field== o.field);                                           \
  }                                                                     \
  bool operator!=(const This &o) const {                                \
    return (field!= o.field);                                           \
  }                                                                     \
  bool operator<(const This &o) const {                                 \
    return (field< o.field);                                            \
  }                                                                     \
  bool operator>(const This &o) const {                                 \
    return (field> o.field);                                            \
  }                                                                     \
  bool operator>=(const This &o) const {                                \
    return (field>= o.field);                                           \
  }                                                                     \
  bool operator<=(const This &o) const {                                \
    return (field<= o.field);                                           \
  }                                                                     \
  int compare(const This &o) const {                                    \
    if (operator<(o)) return -1;                                        \
    else if (operator>(o)) return 1;                                    \
    else return 0;                                                      \
  }

#endif

#ifdef IMP_DOXYGEN
//! Implement comparison in a class using field as the variable to compare
/** The macro requires that This be defined as the type of the current class.
 */
#define IMP_COMPARISONS_2(f0, f1)
#else
#define IMP_COMPARISONS_2(f0, f1)                                       \
  bool operator==(const This &o) const {                                \
    return (f0== o.f0 && f1==o.f1);                                     \
  }                                                                     \
  bool operator!=(const This &o) const {                                \
    return (f0!= o.f0 || f1 != o.f1);                                   \
  }                                                                     \
  bool operator<(const This &o) const {                                 \
    if (f0< o.f0) return true;                                          \
    else if (f0 > o.f0) return false;                                   \
    else return f1 < o.f1;                                              \
  }                                                                     \
  bool operator>(const This &o) const {                                 \
    if (f0 > o.f0) return true;                                         \
    else if (f0 < o.f0) return false;                                   \
    else return f1 > o.f1;                                              \
  }                                                                     \
  bool operator>=(const This &o) const {                                \
    return operator>(o) || operator==(o);                               \
  }                                                                     \
  bool operator<=(const This &o) const {                                \
    return operator<(o) || operator==(o);                               \
  }                                                                     \
  int compare(const This &o) const {                                    \
    if (operator<(o)) return -1;                                        \
    else if (operator>(o)) return 1;                                    \
    else return 0;                                                      \
  }
#endif


#ifdef IMP_DOXYGEN
//! Implement comparison in a class using field as the variable to compare
/** The macro requires that This be defined as the type of the current class.
 */
#define IMP_COMPARISONS_3(f0, f1, f2)
#else
#define IMP_COMPARISONS_3(f0, f1, f2)                                   \
  bool operator==(const This &o) const {                                \
    return (f0== o.f0 && f1==o.f1 && f2 == o.f2);                       \
  }                                                                     \
  bool operator!=(const This &o) const {                                \
    return (f0!= o.f0 || f1 != o.f1 || f2 != o.f2);                     \
  }                                                                     \
  bool operator<(const This &o) const {                                 \
    if (f0< o.f0) return true;                                          \
    else if (f0 > o.f0) return false;                                   \
    if (f1< o.f1) return true;                                          \
    else if (f1 > o.f1) return false;                                   \
    else return f2 < o.f2;                                              \
  }                                                                     \
  bool operator>(const This &o) const {                                 \
    if (f0 > o.f0) return true;                                         \
    else if (f0 < o.f0) return false;                                   \
    if (f1 > o.f1) return true;                                         \
    else if (f1 < o.f1) return false;                                   \
    else return f2 > o.f2;                                              \
  }                                                                     \
  bool operator>=(const This &o) const {                                \
    return operator>(o) || operator==(o);                               \
  }                                                                     \
  bool operator<=(const This &o) const {                                \
    return operator<(o) || operator==(o);                               \
  }                                                                     \
  int compare(const This &o) const {                                    \
    if (operator<(o)) return -1;                                        \
    else if (operator>(o)) return 1;                                    \
    else return 0;                                                      \
  }
#endif

#ifdef IMP_DOXYGEN                                                         \
//! Implement operator<< on class name, assuming it has one template argument
/** \copydetails IMP_OUTPUT_OPERATOR
 */
#define IMP_OUTPUT_OPERATOR_1(name)
#else
#define IMP_OUTPUT_OPERATOR_1(name)                                     \
  template <class L>                                                    \
  inline std::ostream& operator<<(std::ostream &out, const name<L> &i)  \
  {                                                                     \
    i.show(out);                                                        \
    return out;                                                         \
  }
#endif

#ifdef IMP_DOXYGEN
//! Implement operator<< on class name, assuming it has two template arguments
/** \copydetails IMP_OUTPUT_OPERATOR
 */
#define IMP_OUTPUT_OPERATOR_2(name)
#else
#define IMP_OUTPUT_OPERATOR_2(name)                                     \
  template <class L, class M>                                           \
  inline std::ostream& operator<<(std::ostream &out, const name<L, M> &i) \
  {                                                                     \
    i.show(out);                                                        \
    return out;                                                         \
  }
#endif

#ifdef IMP_DOXYGEN
//! Implement operator<< on class name
/** The class named should define the method
    \c void \c show(std::ostream&).
 */
#define IMP_OUTPUT_OPERATOR(name)
#else
#define IMP_OUTPUT_OPERATOR(name)                                       \
  inline std::ostream &operator<<(std::ostream &out, const name &i)     \
  {                                                                     \
    i.show(out);                                                 \
    return out;                                                  \
  }
#endif



//! Use the swap_with member function to swap two objects
/** The two objects mustbe of the same type (Name) and define
    the method \c swap_with().
*/
#define IMP_SWAP(Name) \
  IMP_NO_DOXYGEN(inline void swap(Name &a, Name &b) {a.swap_with(b);})

#define IMP_SWAP_2(Name)                          \
  template <class A, class B>                     \
  inline void swap(Name<A,B> &a, Name<A,B> &b) {  \
    a.swap_with(b);                               \
  }

#define IMP_SWAP_3(Name)                                \
  template <class A, class B, class C>                  \
  inline void swap(Name<A,B,C> &a, Name<A,B,C> &b) {    \
    a.swap_with(b);                                     \
  }


//! swap two member variables assuming the other object is called o
/** Swap the member \c var_name of the two objects (this and o).
 */
#define IMP_SWAP_MEMBER(var_name) \
  std::swap(var_name, o.var_name);



//! use a copy_from method to create a copy constructor and operator=
/** This macro is there to aid with classes which require a custom
    copy constructor. It simply forwards \c operator= and the copy
    constructor to a method \c copy_from() which should do the copying.

    You should think very hard before implementing a class which
    requires a custom copy custructor as it is easy to get wrong
    and you can easily wrap most resources with RIIA objects
    (\external{en.wikipedia.org/wiki/Resource_Acquisition_Is_Initialization,
    wikipedia entry}).

 */
#define IMP_COPY_CONSTRUCTOR(TC) TC(const TC &o){copy_from(o);}  \
  TC& operator=(const TC &o) {copy_from(o); return *this;}



#ifdef IMP_DOXYGEN
//! Ref counted objects should have private destructors
/** This macro defines a private destructor and adds the appropriate
    friend methods so that the class can be used with ref counting.
    By defining a private destructor, you make it so that the object
    cannot be declared on the stack and so must be ref counted.
    \see IMP::RefCounted
 */
#define IMP_REF_COUNTED_DESTRUCTOR
#else

#ifdef _MSC_VER
// VC doesn't understand friends properly
#define IMP_REF_COUNTED_DESTRUCTOR(Classname)                   \
public:                                                         \
 virtual ~Classname(){}
#else

#if defined(SWIG) || defined(IMP_SWIG_WRAPPER)
// SWIG doesn't do friends right either, but we don't care as much
#define IMP_REF_COUNTED_DESTRUCTOR(Classname)                   \
public:                                                      \
 virtual ~Classname(){}
#else
/* The destructor is unprotected for SWIG since if it is protected
    SWIG does not wrap the python proxy distruction and so does not
    dereference the ref counted pointer. Swig also gets confused
    on template friends.
 */
#define IMP_REF_COUNTED_DESTRUCTOR(Classname)                           \
  protected:                                                            \
  IMP_NO_DOXYGEN(template <class T> friend void IMP::internal::unref(T*);) \
  IMP_NO_DOXYGEN(virtual ~Classname(){})
#endif // SWIG
#endif // _MSC_VER
#endif // doxygen


/** \name Macros to aid with implementing decorators

    These macros are here to aid in implementation of IMP::Decorator
    objects.  The first two declare/define the expected methods. The
    remainder help implement basic functions.

    @{
 */

//! Define the basic things needed by a Decorator.
/** The key things this defines are
    - a default constructor,
    - a static cast function,
    - a method get_particle(),
    - a method get_model()
    - comparisons.

    \param[in] Name is the name of the decorator, such as XYZR
    \param[in] Parent The class name for the parent of this class,
    typically Decorator

    It requires that the implementer of the Decorator implement the static
    methods:

    - bool is_instance_of(Particle *p) which checks if a particle has
    needed attributes.
    - create(Particle *p, other args) which adds the needed attributes
    to a particle

    In addition, the macro declares a show(std::ostream &out,
    std::string prefix) method which should be implemented elsewhere.

    You also implement static methods \c get_x_key() to return each of the
    keys used. These static methods, which must be defined in the \c .cpp
    file should declare the key itself as a \c static member variable to
    avoid initializing the key if the decorator is not used.

    See \ref decorators "the decorators page" for a more detailed description
    of decorators.

    \see IMP_DECORATOR_TRAITS()
 */
#define IMP_DECORATOR(Name, Parent)                                     \
public:                                                                 \
/* Should be private but SWIG accesses it through the comparison
    macros*/                                                            \
IMP_NO_DOXYGEN(typedef Name This);                                      \
/** \brief Create null decorator. Such a decorator is like a NULL
    pointer in C++. */                                                  \
 Name(): Parent(){}                                                     \
/** \brief Create a decorator wrapping a particle which already has
    had create() called on it. */                                       \
explicit Name(::IMP::Particle *p): Parent(p) {                          \
   IMP_assert(is_instance_of(p),                                        \
              "Particle missing required attributes for decorator "     \
              << #Name << *p << std::endl);                             \
 }                                                                      \
 static Name cast(::IMP::Particle *p) {                                 \
   IMP_CHECK_OBJECT(p);                                                 \
   if (!is_instance_of(p)) {                                            \
     return Name();                                                     \
   }                                                                    \
   return Name(p);                                                      \
 }                                                                      \
 void show(std::ostream &out=std::cout,                                 \
           std::string prefix=std::string()) const;                     \


//! Define the basic things needed by a Decorator which has a traits object.
/** This macro is the same as IMP_DECORATOR() except that an extra object
    of type TraitsType is passed after the particle to
    Decorator::is_instance_of(), Decorator::create() and Decorator::cast().
    As in the IMP::core::XYZR or IMP::core::Hierarchy,
    this object can be used to parameterize the Decorator. The traits
    object is stored in the decorator and made accessible through
    the get_traits() method.
 */
#define IMP_DECORATOR_TRAITS(Name, Parent, TraitsType, traits_name,     \
default_traits)                                                         \
  private:                                                              \
  TraitsType traits_name##_;                                            \
public:                                                                 \
IMP_NO_DOXYGEN(typedef Name This;)                                      \
 /** \brief Create null decorator. Such a decorator is like a NULL
     pointer in C++. */                                                 \
Name(): Parent(){}                                                      \
/** \brief Create a decorator wrapping a particle which already has
    had create() called on it with the passed traits. */                \
Name(::IMP::Particle *p, const TraitsType &tr=default_traits): Parent(p), \
                                                   traits_name##_(tr) { \
  IMP_assert(is_instance_of(p, tr),                                     \
             "Particle missing required attributes for decorator "      \
             << #Name << *p << std::endl);                              \
}                                                                       \
static Name cast(::IMP::Particle *p, const TraitsType &tr=default_traits) { \
  if (!is_instance_of(p, tr)) return Name();                            \
  else return Name(p, tr);                                              \
}                                                                       \
void show(std::ostream &out=std::cout,                                  \
          std::string prefix=std::string()) const;                      \
/** Get the traits object */                                            \
const TraitsType &get_##traits_name() const {                           \
  return traits_name##_;                                                \
}                                                                       \


//! Perform actions dependent on whether a particle has an attribute.
/** A common pattern is to check if a particle has a particular attribute,
    do one thing if it does and another if it does not. This macro implements
   that pattern. It requires that the method get_particle() return the
   particle being used.

   \param[in] AttributeKey The key for the attribute
   \param[in] Type The type for the attribute ("Int", "Float", "String")
   \param[in] has_action The action to take if the Particle has the attribute.
                         The attribute value is stored in the variable VALUE.
   \param[in] not_has_action The action to take if the Particle does not have
                             the attribute.
   \see IMP_DECORATOR_GET()
   \see IMP_DECORATOR_GET_SET()

 */
#define IMP_DECORATOR_GET(AttributeKey, Type, has_action, not_has_action) \
  if (get_particle()->has_attribute(AttributeKey)) {                    \
    Type VALUE =  get_particle()->get_value(AttributeKey);              \
    has_action;                                                         \
  } else {                                                              \
    not_has_action;                                                     \
  }


//! Set an attribute, creating it if it does not already exist.
/** Another common pattern is to have an assumed value if the attribute
    is not there. Then, you sometimes need to set the value whether it
    is there or not.
   \see IMP_DECORATOR_GET()
   \see IMP_DECORATOR_GET_SET()
 */
#define IMP_DECORATOR_SET(AttributeKey, value)          \
  if (get_particle()->has_attribute(AttributeKey)) {    \
    get_particle()->set_value(AttributeKey, value)  ;   \
  } else {                                              \
    get_particle()->add_attribute(AttributeKey, value); \
  }

//! define methods for getting and setting a particular simple field
/**
   This macros defines methods to get an set a particular attribute.

   \param[in] name The lower case name of the attribute
   \param[in] AttributeKey The AttributeKey object controlling
                           the attribute.
   \param[in] Type The type of the attribute (upper case).
   \param[in] ReturnType The type to return from the get.
   \see IMP_DECORATOR_GET()
   \see IMP_DECORATOR_SET()
*/
#define IMP_DECORATOR_GET_SET(name, AttributeKey, Type, ReturnType)     \
  ReturnType get_##name() const {                                       \
    return static_cast<ReturnType>(get_particle()->get_value(AttributeKey)); \
  }                                                                     \
  void set_##name(ReturnType t) {                                             \
    get_particle()->set_value(AttributeKey, t);                         \
  }

//! Define methods for getting and setting an optional simple field.
/**
   See IMP_DECORATOR_GET_SET(). The difference is that here you can provide
   a default value to use if the decorator does not have the attribute.

   \param[in] name The lower case name of the attribute
   \param[in] AttributeKey The expression to get the required attribute key.
   \param[in] Type The type of the attribute (upper case).
   \param[in] ReturnType The type to return from the get.
   \param[in] default_value The value returned if the attribute is missing.
 */
#define IMP_DECORATOR_GET_SET_OPT(name, AttributeKey, Type,             \
                                  ReturnType, default_value)            \
  ReturnType get_##name() const {                                       \
    IMP_DECORATOR_GET(AttributeKey, Type,                               \
                      return static_cast<ReturnType>(VALUE),            \
                      return default_value);                            \
  }                                                                     \
  void set_##name(ReturnType t) {                                       \
    IMP_DECORATOR_SET(AttributeKey, t);                                 \
  }


//! Define a set of attributes which form an array
/**
   The macro defines a set of functions for using the array:
   - get_name(unsigned int)

   - get_number_of_name()

   - add_name(ExternalType)

   - add_name_at(ExternalType, unsigned int)

   - remove_name(unsigned int)

   in addition it defines the private methods
   - add_required_attributes_for_name(Particle *)

   \param[in] protection Whether it should be public, protected or private
   \param[in] Class The name of the wrapping class
   \param[in] Name The capitalized name of the prefix to use
   \param[in] name the name prefix to use, see the above method names
   \param[in] plural the plural form of the name
   \param[in] traits the traits object to use to manipulate things. This should
   inherit from or implement the interface of internal::ArrayOnAttributesHelper
   \param[in] ExternalType The name of the type to wrap the return type with.
 */
#define IMP_DECORATOR_ARRAY_DECL(protection, Class,                     \
                                 Name, name, plural,                    \
                                 traits, ExternalType)                  \
private:                                                                \
 template <class T>                                                     \
 static bool has_required_attributes_for_##name(Particle *p,            \
                                                const T &traits) {      \
  return traits.has_required_attributes(p);                             \
 }                                                                      \
 template <class T>                                                     \
 static void add_required_attributes_for_##name(Particle *p,            \
                                                const T &traits) {      \
   return traits.add_required_attributes(p);                            \
 }                                                                      \
 struct Name##AttrArrayAccessor {                                       \
   const Class &d_;                                                     \
   Name##AttrArrayAccessor(const Class &d): d_(d){}                     \
   typedef ExternalType result_type;                                    \
   result_type operator()(unsigned int i) const {                       \
     return d_.get_##name(i);                                           \
   }                                                                    \
   bool operator==(const Name##AttrArrayAccessor &o) const {            \
     return d_== o.d_;                                                  \
   }                                                                    \
 };                                                                     \
protection:                                                             \
 IMP_NO_SWIG(typedef IMP::internal                                      \
             ::IndexingIterator<Name##AttrArrayAccessor>                \
             Name##Iterator;)                                           \
 IMP_NO_SWIG(Name##Iterator plural##_begin() const {                    \
     return Name##Iterator(Name##AttrArrayAccessor(*this));             \
 }                                                                      \
 Name##Iterator plural##_end() const {                                  \
   return Name##Iterator(Name##AttrArrayAccessor(*this),                \
                         get_number_of_##plural());                     \
 })                                                                     \
 ExternalType get_##name(unsigned int i) const {                        \
   return traits.wrap(traits.get_value(get_particle(), i));             \
 }                                                                      \
 unsigned int get_number_of_##plural() const {                          \
   return traits.get_size(get_particle());                              \
 }                                                                      \
 unsigned int add_##name(ExternalType t) {                              \
   traits.audit_value(t);                                               \
   unsigned int i= traits.push_back(get_particle(),                     \
                                    traits.get_value(t));               \
   traits.on_add(get_particle(), t, i);                                 \
   return i;                                                            \
 }                                                                      \
 void add_##name##_at(ExternalType t, unsigned int idx) {               \
   traits.audit_value(t);                                               \
   traits.insert(get_particle(),                                        \
                 idx,                                                   \
                 traits.get_value(t));                                  \
   traits.on_add(get_particle(), t, idx);                               \
   for (unsigned int i= idx+1; i < get_number_of_##plural(); ++i) {     \
     traits.on_change(get_particle(),                                   \
                      traits.get_value( get_particle(), i),             \
                      i-1, i);                                          \
   }                                                                    \
 }                                                                      \
 void remove_##name(ExternalType t) {                                   \
   traits.audit_value(t);                                               \
   unsigned int idx= traits.get_index(get_particle(), t);               \
   traits.on_remove(get_particle(), t);                                 \
   traits.erase(get_particle(),                                         \
                      idx);                                             \
   for (unsigned int i= idx; i < get_number_of_##plural(); ++i) {       \
     traits.on_change(get_particle(),                                   \
                       traits.get_value(get_particle(), i),             \
                       i+1, i);                                         \
   }                                                                    \
 }                                                                      \
 void clear_##plural() {                                                \
   traits.clear(get_particle());                                        \
 }

//! @}

/*
 Particles get_##name##_particles() const {                             \
   Particles ret(get_number_of_##plural());                             \
   for (unsigned int i=0; i< ret.size(); ++i) {                         \
     ret[i]= get_##name(i).get_particle();                              \
   }                                                                    \
   return ret;                                                          \
 }                                                                      \
*/


/** \name Macros to aid with implementation classes

    These macros are here to aid with implementing classes that inherit
    from the various abstract base classes in the kernel. Each macro,
    which should be used in the body of the class,
    declares the set of needed functions. The functions should be defined
    in the associated \c .cpp file. By using the macros, you ensure
    that your class gets the names of the functions correct and it
    makes it easier to update your class if the functions should change.

    @{
 */


//! Define the basic things you need for a Restraint.
/** These are: show, evaluate, get_version_info and a empty destructor
    \param[in] Name The class name
    \param[in] version_info The version info object to return.
    \relatesalso IMP::Restraint
*/
#define IMP_RESTRAINT(Name, version_info)                                \
  virtual Float evaluate(DerivativeAccumulator *accum);                 \
  virtual void show(std::ostream &out=std::cout) const;                 \
  virtual IMP::VersionInfo get_version_info() const { return version_info; }\
  IMP_REF_COUNTED_DESTRUCTOR(Name)                                      \
  public:


//! Define the basic things you need for an optimizer.
/** These are: optimize, get_version_info
    \param[in] version_info The version info object to return.
*/
#define IMP_OPTIMIZER(version_info)                                     \
  /** \short Optimize the model.
      \param[in] max_steps The maximum number of steps to take.
      \return The final score.
   */                                                                   \
  virtual Float optimize(unsigned int max_steps);                       \
  /** \return version and authorship information */                     \
  virtual IMP::VersionInfo get_version_info() const { return version_info; }


//! Define the basics needed for an OptimizerState
/** This macro declares the required functions
    - void update()
    - void show(std::ostream &out) const
    and defines the function
    - get_version_info
    \param[in] Name The name of the class
    \param[in] version_info The version info object to return.
*/
#define IMP_OPTIMIZER_STATE(Name, version_info)                         \
  virtual void update();                                                \
  virtual void show(std::ostream &out=std::cout) const;                 \
  virtual IMP::VersionInfo get_version_info() const { return version_info; }\
  IMP_REF_COUNTED_DESTRUCTOR(Name)

//! Define the basics needed for a ScoreState
/** This macro declares the required functions
    - void do_before_evaluate()
    - void show(std::ostream &out) const
    and defines the function
    - get_version_info
    - an empty destructor

    \relatesalso IMP::ScoreState

    \param[in] Name the class name
    \param[in] version_info The version info object to return.
*/
#define IMP_SCORE_STATE(Name, version_info)                              \
protected:                                                              \
 virtual void do_before_evaluate();                                     \
 virtual void do_after_evaluate(DerivativeAccumulator *da);             \
 IMP_REF_COUNTED_DESTRUCTOR(Name)                                      \
public:                                                                 \
 virtual void show(std::ostream &out=std::cout) const;                  \
  virtual IMP::VersionInfo get_version_info() const { return version_info; }

//! Define the basics needed for a particle refiner
/** This macro declares the following functions
    - bool get_can_refine(Particle*) const;
    - unsigned int get_number_of_refined(Particle *) const;
    - Particle* get_refined(Particle *, unsigned int) const;
    - const Particles get_refined(Particle *) const;
    - void show(std::ostream &out) const;
    - an empty, private destructor

    \param[in] Name The name of the class which this is adding methods to.
    \param[in] version_info The version info object to return

 */
#define IMP_REFINER(Name, version_info)                                 \
  public:                                                               \
  virtual bool get_can_refine(Particle*) const;                         \
  virtual void show(std::ostream &out) const;                           \
  virtual Particle* get_refined(Particle *, unsigned int) const;        \
  virtual const ParticlesTemp get_refined(Particle *) const;       \
  virtual unsigned int get_number_of_refined(Particle *) const;         \
  virtual IMP::VersionInfo get_version_info() const {                   \
    return version_info;                                                \
  }                                                                     \
  IMP_REF_COUNTED_DESTRUCTOR(Name)


//! Declare the functions needed for a SingletonScore
/** The declared functions are:
    - void show(std::ostream &) const
    - double evaluate(Particle *a, DerivativeAccumulator *da) const
    The defined function is
    - VersionInfo get_version_info() const
    - The destructor
 */
#define IMP_SINGLETON_SCORE(ClassName, version)  \
  VersionInfo get_version_info() const {return version;}         \
  void show(std::ostream &out= std::cout) const;                 \
  double evaluate(Particle *a, DerivativeAccumulator *da) const; \
  IMP_REF_COUNTED_DESTRUCTOR(ClassName)


//! Declare the functions needed for a PairScore
/** The declared functions are:
    - void show(std::ostream &) const
    - double evaluate(Particle *a, Particle *b, DerivativeAccumulator *da) const
    The defined function is
    - VersionInfo get_version_info() const
    - The destructor
 */
#define IMP_PAIR_SCORE(ClassName, version)  \
  VersionInfo get_version_info() const {return version;}         \
  void show(std::ostream &out= std::cout) const;                 \
  double evaluate(Particle *a, Particle *b,                      \
                  DerivativeAccumulator *da) const;              \
  IMP_REF_COUNTED_DESTRUCTOR(ClassName)


//! Declare the functions needed for a SingletonModifier
/** The declared functins are:
    - void show(std::ostream &) const
    - void apply(Particle *) const
    The defined functions are:
    - get_version_info() const
    - void apply(Particle *a, DerivativeAccumulator&) const

    \see IMP_SINGLETON_MODIFIER_DA
 */
#define IMP_SINGLETON_MODIFIER(version)                          \
  VersionInfo get_version_info() const {return version;}         \
  void show(std::ostream &out= std::cout) const;                 \
  void apply(Particle *a) const;                                 \
  void apply(Particle *a, DerivativeAccumulator&) const{         \
    apply(a);                                                    \
  }



//! Declare the functions needed for a PairModifier
/** The declared functins are:
    - void show(std::ostream &) const
    - void apply(Particle *, Particle *b) const
    The defined functions are:
    - get_version_info() const
    - void apply(Particle *, Particle *, DerivativeAccumulator&) const
    \see IMP_PAIR_MODIFIER_DA
 */
#define IMP_PAIR_MODIFIER(version)                                      \
  VersionInfo get_version_info() const {return version;}                \
  void show(std::ostream &out= std::cout) const;                        \
  void apply(Particle *a, Particle *b) const;                           \
  void apply(Particle *a, Particle *b, DerivativeAccumulator&) const{     \
    apply(a,b);                                                         \
  }


//! Declare the functions needed for a SingletonModifier
/** This version takes a derivative accumulator.
   \see IMP_SINGLETON_MODIFIER
 */
#define IMP_SINGLETON_MODIFIER_DA(version)                              \
  VersionInfo get_version_info() const {return version;}                \
  void show(std::ostream &out= std::cout) const;                        \
  void apply(Particle *a, DerivativeAccumulator &da) const;             \
  void apply(Particle *) const{                                         \
    IMP_LOG(VERBOSE, "This modifier requires a derivative accumulator " \
            << *this << std::endl);                                     \
 }


//! Declare the functions needed for a PairModifier
/** This version takes a derivative accumulator.
    \see IMP_PAIR_MODIFIER
 */
#define IMP_PAIR_MODIFIER_DA(version) \
  VersionInfo get_version_info() const {return version;}                \
 void show(std::ostream &out= std::cout) const;                         \
 void apply(Particle *a, Particle *b, DerivativeAccumulator *da) const; \
 void apply(Particle *, Particle *) const{                              \
   IMP_LOG(VERBOSE, "This modifier requires a derivative accumulator "  \
         << *this << std::endl);                                        \
 }
//! Declare the needed functions for a SingletonContainer
/** You need to implement
    - get_contains_particle
    - get_number_of_particles
    - get_particle
    - show
    A private, empty destructor is provided.
    \relatesalso IMP::SingletonContainer
*/
#define IMP_SINGLETON_CONTAINER(Name, version_info)                  \
  bool get_contains_particle(Particle* p) const;                    \
  unsigned int get_number_of_particles() const;                     \
  Particle* get_particle(unsigned int i) const;                     \
  void show(std::ostream &out= std::cout) const;                        \
  IMP::VersionInfo get_version_info() const { return version_info; }\
  void apply(const SingletonModifier *sm);                          \
  void apply(const SingletonModifier *sm,                            \
             DerivativeAccumulator *da);                            \
  double evaluate(const SingletonScore *s,                          \
                  DerivativeAccumulator *da) const;                 \
  Particles get_particles() const;                                  \
  unsigned int get_revision() const;                                \
  IMP_REF_COUNTED_DESTRUCTOR(Name)                                  \
  public:

//! Declare the needed functions for a PairContainer
/** \relatesalso IMP::PairContainer
    See IMP_SINGLETON_CONTAINER() for full documentation.
 */
#define IMP_PAIR_CONTAINER(Name, version_info)                           \
  bool get_contains_particle_pair(ParticlePair p) const;                \
  unsigned int get_number_of_particle_pairs() const;                    \
  ParticlePair get_particle_pair(unsigned int i) const;                 \
  void show(std::ostream &out= std::cout) const;                        \
  IMP::VersionInfo get_version_info() const { return version_info; }    \
  void apply(const PairModifier *sm);                                   \
  void apply(const PairModifier *sm,                                     \
             DerivativeAccumulator *da);                                \
  double evaluate(const PairScore *s,                                   \
                  DerivativeAccumulator *da) const;                     \
  unsigned int get_revision() const;                                    \
  ParticlePairs get_particle_pairs() const;                             \
  IMP_REF_COUNTED_DESTRUCTOR(Name)                                      \
  public:



//! Declare the needed functions for a UnaryFunction
/** \relatesalso IMP::UnaryFunction
    This macro declares the methods:
    - Object::show()
    - UnaryFunction::evaluate_with_derivative()
    - UnaryFunction::evaluate()

    It also defines the methods
    - Object::get_version_info()

    \see IMP_UNARY_FUNCTION_INLINE
*/
#define IMP_UNARY_FUNCTION(Name, version_info)                          \
  virtual DerivativePair evaluate_with_derivative(double feature) const; \
  virtual double evaluate(double feature) const;                        \
  virtual void show(std::ostream &out=std::cout) const;                 \
  IMP::VersionInfo get_version_info() const { return version_info; }    \
  IMP_REF_COUNTED_DESTRUCTOR(Name)                                      \
  public:

//! Declare the needed functions for a UnaryFunction which evaluates inline
/** \relatesalso IMP::UnaryFunction

    This macro declares all the functions needed for an IMP::UnaryFunction
    inline in the class. There is no need for an associated \c .cpp file.

    The last three arguments are expressions that evaluate to the
    unary function value and derivative and are sent to the stream in the
    show function, respectively. The input to the function is called
    \c feature.

    \see IMP_UNARY_FUNCTION
*/
#define IMP_UNARY_FUNCTION_INLINE(Name, version_info, value_expression, \
                           derivative_expression, show_expression)      \
  virtual DerivativePair evaluate_with_derivative(double feature) const { \
    return DerivativePair((value_expression), (derivative_expression)); \
  }                                                                     \
  virtual double evaluate(double feature) const {                       \
    return (value_expression);                                          \
  }                                                                     \
  virtual void show(std::ostream &out=std::cout) const {                \
    out << show_expression;                                             \
  }                                                                     \
  IMP::VersionInfo get_version_info() const { return version_info; }    \
  IMP_REF_COUNTED_DESTRUCTOR(Name)                                      \
  public:


//! Declare a IMP::FailureHandler
/** The method bool handle_failure(std::string, const Exception &) needs to
    be implemented as well as the usual show method.
*/
#define IMP_FAILURE_HANDLER(Name, version_info)                 \
  VersionInfo get_version_info() const {return version_info;}   \
  void show(std::ostream &out=std::cout) const;                 \
  void handle_failure();                                        \
  IMP_REF_COUNTED_DESTRUCTOR(Name)

//! @}

#ifndef IMP_DOXYGEN

//! Declare a ref counted pointer to a new object
/** \param[in] Typename The namespace qualified type being declared
    \param[in] varname The name for the ref counted pointer
    \param[in] args The paretesized arguments to the constructor, or ()
    if there are none.
    \see IMP::Pointer
    \code
    IMP_NEW(m, IMP::Model, ());
    \endcode
 */
#define IMP_NEW(Typename, varname, args)        \
  Pointer<Typename> varname(new Typename args)



//! Use this to label a function with no side effects
#ifdef __GNU__
#define IMP_NO_SIDEEFFECTS __attribute__ ((pure))
#else
#define IMP_NO_SIDEEFFECTS
#endif


//! Use this to make the compiler (possibly) warn if the result is not used
#ifdef __GNU__
#define IMP_WARN_UNUSED_RESULT __attribute__ ((warn_unused_result))
#else
#define IMP_WARN_UNUSED_RESULT
#endif

//! Time the given command and assign the time of one iteration to the variable
/** The units for the time are in seconds. */
#define IMP_TIME(block, timev)                  \
  {                                             \
    boost::timer imp_timer;                     \
    unsigned int imp_reps=0;                    \
    do {                                        \
      block;                                    \
      ++imp_reps;                               \
    } while (imp_timer.elapsed() < 2.5);        \
    timev= imp_timer.elapsed()/imp_reps;        \
  }

#endif

#endif  /* IMP_MACROS_H */
