/**
 *  \file core/macros.h    \brief Various important macros
 *                           for implementing decorators.
 *
 *  Copyright 2007-8 Sali Lab. All rights reserved.
 *
 */

#ifndef IMPCORE_DECORATOR_MACROS_H
#define IMPCORE_DECORATOR_MACROS_H

#include <sstream>

//! Define the basic things needed by a Decorator.
/** The key things this defines are a default constructor, a static create
    function, a static cast function, a method get_particle(), a method
    get_model() and comparisons.

    \param[in] Name is the name of the decorator, such as NameDecorator
    \param[in] Parent The class name for the parent of this class,
    typically DecoratorBase
    \param[in] check_required is code which returns a bool which checks
    if a Particle *p has the required fields
    \param[in] add_required is code which adds the required fields

    It requires that the implementer of the Decorator implement the static
    method:

    - void initialize_static_data() which initializes static data such as
      AttributeKey instances. Ideally, this should internally make sure it
      is only done once.
 */
#define IMP_DECORATOR(Name, Parent, check_required, add_required)       \
  private:                                                              \
  static bool decorator_keys_initialized_;                              \
protected:                                                              \
 /** goes away once DecoratorBase patched */                            \
 static bool has_required_attributes(::IMP::Particle *p) {              \
   return is_instance_of(p);                                            \
 }                                                                      \
 /** Initialize the data needed for these decorators */                 \
  static void decorator_initialize_static_data();                       \
  static void add_required_attributes(::IMP::Particle *p) {             \
    if (!internal::parent_instance<Parent>(p)) {                        \
      Parent::add_required_attributes(p);                               \
    }                                                                   \
    add_required;                                                       \
  }                                                                     \
  friend class DecoratorBase;                                           \
public:                                                                 \
/** \note Should be private but SWIG accesses it through the comparison
    macros*/                                                            \
 typedef Name This;                                                     \
 /** \short The default constructor. This is used as a null value */    \
 Name(): Parent(){}                                                     \
 /** \short Construct from a Particle which has all needed attributes */\
 Name(::IMP::Particle *p): Parent(p) {                                  \
   decorator_initialize_static_data();                                  \
   IMP_assert(decorator_keys_initialized_,                              \
              #Name "::create or ::cast must be called before the "     \
              << "constructor is used.");                               \
   IMP_assert(is_instance_of(p),                                        \
              "Particle missing required attributes for decorator "     \
              << #Name << *p << std::endl);                             \
 }                                                                      \
 /** Add the necessary attributes to p and return a decorator. */       \
 static Name create(::IMP::Particle *p) {                               \
   return IMP::DecoratorBase::create<Name>(p);                          \
 }                                                                      \
 /** Check that p has the necessary attributes and return a decorator.
     \throws InvalidStateException if some required attributes are
     missing
 */                                                                     \
 static Name cast(::IMP::Particle *p) {                                 \
   return IMP::DecoratorBase::cast<Name>(p);                            \
 }                                                                      \
/** Check if the particle has the needed attributes for a
    cast to succeed */                                                  \
 static bool is_instance_of(::IMP::Particle *p) {                       \
   decorator_initialize_static_data();                                  \
   if (!internal::parent_instance<Parent>(p)) return false;             \
   check_required;                                                      \
 }                                                                      \
 /** Write information about this decorator to out. Each line should
     prefixed by prefix*/                                               \
 void show(std::ostream &out=std::cout,                                 \
           std::string prefix=std::string()) const;


/**
   Put this macro in to the .cpp with code to initialize the keys
   used by the decorator. This will make sure that the keys are
   initialized before use and initialized exactly once.
   \param[in] Name the name of the decorate
   \param[in] Parent the name of the parent decorator to make sure its keys
   are initalized
   \param[in] work The list of statements to initialize the keys.
   this should probably look something like {a_key_=IntKey("Foo");...}
 */
#define IMP_DECORATOR_INITIALIZE(Name, Parent, work)\
  bool Name::decorator_keys_initialized_=false;     \
  void Name::decorator_initialize_static_data() {   \
    if (decorator_keys_initialized_) return;        \
    else {                                          \
      Parent::decorator_initialize_static_data();   \
      work;                                         \
      decorator_keys_initialized_=true;             \
    }                                               \
  }

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
 */
#define IMP_DECORATOR_GET(AttributeKey, Type, has_action, not_has_action) \
  if (get_particle()->has_attribute(AttributeKey)) {                    \
    Type VALUE =  get_particle()->get_value(AttributeKey);              \
    has_action;                                                         \
  } else {                                                              \
    not_has_action;                                                     \
  }


//! Set an attribute, creating it if it does not already exist.
/** Another commont pattern is to have an assumed value if the attribute
    is not there. Then, you sometimes need to set the value whether it
    is there or not.
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
*/
#define IMP_DECORATOR_GET_SET(name, AttributeKey, Type, ReturnType)     \
  /** \return the attribute value  */                                   \
  ReturnType get_##name() const {                                       \
    return static_cast<ReturnType>(get_particle()->get_value(AttributeKey)); \
  }                                                                     \
  /** Set the attribute \param[in] t the value    */                    \
  void set_##name(ReturnType t) {                                             \
    get_particle()->set_value(AttributeKey, t);                         \
  }

//! Define methods for getting and setting an optional simple field.
/**
   See IMP_DECORATOR_GET_SET.

   \param[in] name The lower case name of the attribute
   \param[in] AttributeKey The AttributeKey object controlling
                           the attribute.
   \param[in] Type The type of the attribute (upper case).
   \param[in] ReturnType The type to return from the get.
   \param[in] default_value The value returned if the attribute is missing.
 */
#define IMP_DECORATOR_GET_SET_OPT(name, AttributeKey, Type,             \
                                  ReturnType, default_value)            \
  /** \return the attribute value*/                                     \
  ReturnType get_##name() const {                                       \
    IMP_DECORATOR_GET(AttributeKey, Type,                               \
                      return static_cast<ReturnType>(VALUE),            \
                      return default_value);                            \
  }                                                                     \
  /** \param[in] t the value to set the attribute to*/                  \
  void set_##name(ReturnType t) {                                       \
    IMP_DECORATOR_SET(AttributeKey, t);                                 \
  }

              /* static internal::ArrayOnAttributesHelper<Traits::Key,  \
                 Traits::Value> name##_data_;  */

//! Define a set of attributes which form an array
/**
   This macro should go in the header and IMP_DECORATOR_ARRAY_CPP into the .cpp
   and IMP_DECORATOR_ARRAY_INIT in the initialize_static_data function

   The macro defines a set of functions for using the array:
   - get_name(unsigned int)

   - get_number_of_name()

   - add_name(ExternalType)

   - add_name_at(ExternalType, unsigned int)

   - remove_name(unsigned int)

   in addition it defines the private methods
   - add_required_attributes_for_name(Particle *)

   \param[in] protection Whether it should be public, protected or private
   \param[in] name the name prefix to use, see the above method names
   \param[in] plural the plural form of the name
   \param[in] traits the traits object to use to manipulate things. This should
   inherit from or implement the interface of internal::ArrayOnAttributesHelper
   \param[in] ExternalType The name of the type to wrap the return type with.
   \param[in] helper The internal::ArrayOnAttributesHelper instance to use.
 */
#define IMP_DECORATOR_ARRAY_DECL(protection, name, plural,              \
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
protection:                                                             \
 /** \brief Get the ith member*/                                        \
 ExternalType get_##name(unsigned int i) const {                        \
   return traits.wrap(traits.get_value(get_particle(), i));             \
 }                                                                      \
 /** \brief Get the total number of them*/                              \
 unsigned int get_number_of_##plural() const {                          \
   return traits.get_size(get_particle());                              \
 }                                                                      \
 /** \brief Add t at the end */                                         \
 unsigned int add_##name(ExternalType t) {                              \
   traits.audit_value(t);                                               \
   unsigned int i= traits.push_back(get_particle(),                     \
                                    traits.get_value(t));               \
   traits.on_add(get_particle(), t, i);                                 \
   return i;                                                            \
 }                                                                      \
 /** Add t at a certain position */                                     \
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
 /** Remove t from the array */                                         \
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
 }


//! add a method to get a key
/** One has to make sure to call the
    decorator_initialize_static_data method first
 */
#define IMP_DECORATOR_GET_KEY(KeyType, key_name, variable_name)\
  static KeyType get_##key_name() {                            \
  decorator_initialize_static_data();                          \
  return variable_name;                                        \
  }

#define IMP_ATOM_TYPE_INDEX 8974343
#define IMP_RESIDUE_TYPE_INDEX 90784334

#endif  /* IMPCORE_DECORATOR_MACROS_H */
