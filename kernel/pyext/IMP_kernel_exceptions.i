/* IMP exception handling */

/* Runtime check functions from exception.h */
namespace IMP
{
enum CheckLevel {NONE=0, USAGE=1, USAGE_AND_INTERNAL=2};
void set_check_level(CheckLevel tf);
CheckLevel get_check_level();
void set_print_exceptions(bool tf);
}

/* Python prints exception messages */                        
%pythoncode %{                                                
set_print_exceptions(False)
%}

/* Create Python exception classes at startup to mirror C++ classes, if we're
   building the kernel. If we're building a module, import these classes from
   the kernel. */
%define CREATE_EXCEPTION_CLASS(VAR, CNAME)
#if defined(IMP_SWIG_KERNEL)
VAR = PyErr_NewException((char *)"_IMP.CNAME", imp_exception, NULL);
Py_INCREF(VAR);
PyModule_AddObject(m, "CNAME", VAR)
#else
VAR = PyDict_GetItemString(kernel_dict, "CNAME")
#endif
%enddef

/* PyErr_NewException only takes a tuple in Python 2.5 or later. In older
   Pythons we will have to reassign to __bases__ after the class is created
   (but this in turn does not work in newer Pythons, JPython, IronPython etc.)
*/
%define CREATE_EXCEPTION_CLASS_PYTHON(VAR, CNAME, PYNAME)
#if !defined(IMP_SWIG_KERNEL)
CREATE_EXCEPTION_CLASS(VAR, CNAME)
#else
%#if PY_VERSION_HEX >= 0x02050000
do {
  PyObject *base_tuple = PyTuple_Pack(2, imp_exception, PyExc_##PYNAME);
  VAR = PyErr_NewException((char *)"_IMP.CNAME", base_tuple, NULL);
  Py_INCREF(VAR);
  Py_DECREF(base_tuple);
  PyModule_AddObject(m, "CNAME", VAR);
} while(0)
%#else
CREATE_EXCEPTION_CLASS(VAR, CNAME)
%#endif
#endif
%enddef

%init {
  {
    /* Create or load base exception class */
#ifdef IMP_SWIG_KERNEL
    imp_exception = PyErr_NewException((char *)"_IMP.Exception", NULL, NULL);
    Py_INCREF(imp_exception);
    PyModule_AddObject(m, "Exception", imp_exception);
#else
    PyObject *kernel = PyImport_ImportModule("_IMP");
    PyObject *kernel_dict = PyModule_GetDict(kernel);
    imp_exception = PyDict_GetItemString(kernel_dict, "Exception");
#endif

    /* Create or load exception subclasses */
    CREATE_EXCEPTION_CLASS(imp_internal_exception, InternalException);
    CREATE_EXCEPTION_CLASS(imp_model_exception, ModelException);
    CREATE_EXCEPTION_CLASS(imp_usage_exception, UsageException);

    /* Create or load subclasses that also derive from Python classes */
    CREATE_EXCEPTION_CLASS_PYTHON(imp_index_exception, IndexException,
                                  IndexError);
    CREATE_EXCEPTION_CLASS_PYTHON(imp_io_exception, IOException, IOError);
    CREATE_EXCEPTION_CLASS_PYTHON(imp_value_exception, ValueException,
                                  ValueError);

#ifndef IMP_SWIG_KERNEL
    Py_DECREF(kernel);
#endif
  }
}

/* Make sure that exception classes are visible to Python, and make certain
   subclasses also derive from Python builtins, in Pythons older than 2.5. */
#ifdef IMP_SWIG_KERNEL
%pythoncode %{
from _IMP import Exception, InternalException, ModelException
from _IMP import UsageException, IndexException, IOException, ValueException

import sys
if sys.version_info[:2] < (2,5):
    IndexException.__bases__ += (IndexError,)
    IOException.__bases__ += (IOError,)
    ValueException.__bases__ += (ValueError,)
%}
#endif

%{
static PyObject *imp_exception, *imp_internal_exception, *imp_model_exception,
                *imp_usage_exception, *imp_index_exception, *imp_io_exception,
                *imp_value_exception;
%}

%{
#ifdef IMP_KERNEL_USE_BOOST_FILESYSTEM
#if !defined(BOOST_FILESYSTEM_VERSION)
#define BOOST_FILESYSTEM_VERSION 2
#endif
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/exception.hpp>
#endif

  /* Code to convert C++ exceptions into scripting language errors. Saves
     having lots of catch statements in every single wrapper. */
  static void handle_imp_exception(void)
  {
    try {
      throw;
    /* Map std:: exceptions to IMP equivalents */
    } catch (const std::out_of_range &e) {
      PyErr_SetString(imp_index_exception, e.what());
    } catch (const std::domain_error &e) {
      PyErr_SetString(imp_value_exception, e.what());
    } catch (const std::ios::failure &e) {
      PyErr_SetString(imp_io_exception, e.what());
    } catch (const std::length_error &e) {
      /* Internal error, such as attempt to resize a vector beyond max size */
      PyErr_SetString(imp_internal_exception, e.what());
    /* Map IMP exceptions to Python objects */
    } catch (const IMP::IndexException &e) {
      PyErr_SetString(imp_index_exception, e.what());
    } catch (const IMP::ValueException &e) {
      PyErr_SetString(imp_value_exception, e.what());
    } catch (const IMP::InternalException &e) {
      PyErr_SetString(imp_internal_exception, e.what());
    } catch (const IMP::ModelException &e) {
      PyErr_SetString(imp_model_exception, e.what());
    } catch (const IMP::UsageException &e) {
      PyErr_SetString(imp_usage_exception, e.what());
    } catch (const IMP::IOException &e) {
      PyErr_SetString(imp_io_exception, e.what());
    } catch (const IMP::Exception &e) {
      PyErr_SetString(imp_exception, e.what());
    /* Map Boost exceptions to Python exceptions */
#ifdef IMP_KERNEL_USE_BOOST_FILESYSTEM
    } catch (boost::filesystem::filesystem_error &e) {
      PyErr_SetString(imp_io_exception, e.what());
#endif
    /* Catch memory allocation errors, if raised */
    } catch (const std::bad_alloc &e) {
      PyErr_SetString(PyExc_MemoryError, e.what());
    /* Catch any other exceptions raised */
    } catch (const std::exception &e) {
      PyErr_SetString(PyExc_RuntimeError,
                      e.what());
#if BOOST_VERSION > 103600
    } catch (const boost::exception &e) {
      PyErr_SetString(PyExc_RuntimeError,
                      "Unknown error in boost caught by Python wrapper");
#endif
    } catch (...) {
      PyErr_SetString(PyExc_RuntimeError,
                      "Unknown error caught by Python wrapper");
    }
  /* SWIG_exception contains "goto fail" so make sure the label is defined */
  fail:
    return;
  }
%}

%exception {
  try {
    $action
  } catch (...) {
    // If Python error indicator is set (e.g. from a failed director method),
    // it will be reraised at the end of the method
    if (!PyErr_Occurred()) {
      handle_imp_exception();
    }
    SWIG_fail;
  }
}

// If Python exceptions are raised in a director method, temporarily reraise
// them as C++ exceptions (will be finally handled as Python exceptions
// again by %exception)
%feature("director:except") {
  if ($error != NULL) {
    throw Swig::DirectorMethodException();
  }
}
