/**
 *  \file decorators/utility.h    \brief Various important functionality
 *                                       for implementing decorators.
 *
 *  Copyright 2007 Sali Lab. All rights reserved.
 *
 */

#ifndef __IMP_DECORATOR_UTILITY_H
#define __IMP_DECORATOR_UTILITY_H

//! Define the basic things needed by a Decorator.
/** The key things this defines are a default constructor, a static create
    function, a static cast function, a method get_particle(), a method
    get_model() and comparisons. 

    \param[in] Name is the name of the decorator, such as NameDecorator
    \param[in] check_required is code which returns a bool which checks
    if a Particle *p has the required fields
    \param[in] add_required is code which adds the required fields

    It requires that the implementer of the Decorator implement the static
    method:

    - void initialize_static_data() which initializes static data such as
      AttributeKey instances. Ideally, this should internally make sure it
      is only done once.
 */
#define IMP_DECORATOR(Name, check_required, add_required)               \
  protected:                                                            \
  Particle *particle_;                                                  \
  bool is_default() const {return particle_==NULL;}                     \
  Name(Particle* p): particle_(p) {                                     \
    IMP_assert(has_required_attributes(p),                              \
               "This is not a hierarchy particle " << *p);              \
  }                                                                     \
  static void initialize_static_data();                                 \
  static bool has_required_attributes(Particle *p) {                    \
    check_required;                                                     \
  }                                                                     \
  static void add_required_attributes(Particle *p) {                    \
    add_required;                                                       \
  }                                                                     \
public:                                                                 \
 typedef Name This;                                                     \
 /** The default constructor. This is used as a null value */           \
 Name(): particle_(NULL){}                                              \
/** Add the necessary attributes to p and return a decorator. */        \
 static Name create(Particle *p) {                                      \
   initialize_static_data();                                            \
   add_required_attributes(p);                                          \
   return Name(p);                                                      \
 }                                                                      \
 /** Check that p has the necessary attributes and return a decorator. */\
 static Name cast(Particle *p) {                                        \
   initialize_static_data();                                            \
   if (!has_required_attributes(p)) return Name();                      \
   else return Name(p);                                                 \
 }                                                                      \
 IMP_COMPARISONS_1(particle_)                                           \
 /** \return the particle wrapped by this decorator*/                   \
 Particle *get_particle() const {return particle_;}                     \
/** \return the Model containing the particle */                        \
 Model *get_model() const {return particle_->get_model();}              \
/** Write information about this decorator to out*/                     \
 void show(std::ostream &out) const;                                    \
private:


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
      has_action;                                                       \
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
  void set_##name(Type t) {                                             \
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
    IMP_DECORATOR_GET(AttributeKey, Type, return ReturnType(VALUE),     \
                      return default_value);                            \
  }                                                                     \
  /** \param[in] t the value to set the attribute to*/                  \
  void set_##name(Type t) {                                             \
    IMP_DECORATOR_SET(AttributeKey, t);                                 \
  }


//! Define a set of attributes which form an array
/**
   This macro should go in the header and IMP_DECORATOR_ARRAY_CPP into the .cpp
   and IMP_DECORATOR_ARRAY_INIT in the initialize_static_data function

   To use the array, use functions 

   - Type internal_get_name(unsigned int i) 

   - void internal_add_name(Type) 

   - unsigned int internal_get_number_of_name() const 
 */
#define IMP_DECORATOR_ARRAY_DECL(name, Type)                            \
  protected:                                                            \
  static IntKey number_of_##name##_key_;                                \
  static std::vector<Type##Key> name##_keys_;                           \
  static void generate_##name##_keys(unsigned int i);                   \
  static const Type##Key get_##name##_key(unsigned int i) {             \
    if (i >= name##_keys_.size()) generate_##name##_keys(i);            \
    return name##_keys_[i];                                             \
  }                                                                     \
  Type internal_get_##name(unsigned int i) const{                       \
    IMP_DECORATOR_GET(get_##name##_key(i), Type,                        \
                      return VALUE,                                     \
                      throw IndexException(); return Type());           \
  }                                                                     \
  int internal_add_##name(Type t);                                      \
  unsigned int internal_get_number_of_##name() const {                  \
    IMP_DECORATOR_GET(number_of_##name##_key_,                          \
                      Int, return VALUE, return 0);                     \
  }                                                                     \

//! See IMP_DECORATOR_ARRAY_DECL
#define IMP_DECORATOR_ARRAY_DEF(DecoratorType, name, Type)              \
  IntKey DecoratorType##Decorator::number_of_##name##_key_;             \
  std::vector<Type##Key> DecoratorType##Decorator::name##_keys_;        \
  void DecoratorType##Decorator::generate_##name##_keys(unsigned int i) \
  {                                                                     \
    while (!(i < name##_keys_.size())) {                                \
      std::ostringstream oss;                                           \
      oss << #DecoratorType " " #name " " << name##_keys_.size();       \
      name##_keys_.push_back(Type##Key(oss.str().c_str()));             \
    }                                                                   \
  }                                                                     \
  int DecoratorType##Decorator::internal_add_##name(Type t) {           \
    int nc= internal_get_number_of_##name();                            \
    get_particle()->add_attribute(get_##name##_key(nc), t);             \
    IMP_DECORATOR_SET(number_of_##name##_key_, nc+1);                   \
    return nc;                                                          \
  }

//! See IMP_DECORATOR_ARRAY_DECL
#define IMP_DECORATOR_ARRAY_INIT(DecoratorType, name, Type)       \
  number_of_##name##_key_= IntKey(#DecoratorType " num " #name);

#endif  /* __IMP_DECORATOR_UTILITY_H */
