// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from kachaka_interfaces:msg/LocationList.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "kachaka_interfaces/msg/detail/location_list__struct.h"
#include "kachaka_interfaces/msg/detail/location_list__functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"

// Nested array functions includes
#include "kachaka_interfaces/msg/detail/location__functions.h"
// end nested array functions include
bool kachaka_interfaces__msg__location__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * kachaka_interfaces__msg__location__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool kachaka_interfaces__msg__location_list__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[51];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("kachaka_interfaces.msg._location_list.LocationList", full_classname_dest, 50) == 0);
  }
  kachaka_interfaces__msg__LocationList * ros_message = _ros_message;
  {  // locations
    PyObject * field = PyObject_GetAttrString(_pymsg, "locations");
    if (!field) {
      return false;
    }
    PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'locations'");
    if (!seq_field) {
      Py_DECREF(field);
      return false;
    }
    Py_ssize_t size = PySequence_Size(field);
    if (-1 == size) {
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    if (!kachaka_interfaces__msg__Location__Sequence__init(&(ros_message->locations), size)) {
      PyErr_SetString(PyExc_RuntimeError, "unable to create kachaka_interfaces__msg__Location__Sequence ros_message");
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    kachaka_interfaces__msg__Location * dest = ros_message->locations.data;
    for (Py_ssize_t i = 0; i < size; ++i) {
      if (!kachaka_interfaces__msg__location__convert_from_py(PySequence_Fast_GET_ITEM(seq_field, i), &dest[i])) {
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
    }
    Py_DECREF(seq_field);
    Py_DECREF(field);
  }
  {  // default_location_id
    PyObject * field = PyObject_GetAttrString(_pymsg, "default_location_id");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->default_location_id, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * kachaka_interfaces__msg__location_list__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of LocationList */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("kachaka_interfaces.msg._location_list");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "LocationList");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  kachaka_interfaces__msg__LocationList * ros_message = (kachaka_interfaces__msg__LocationList *)raw_ros_message;
  {  // locations
    PyObject * field = NULL;
    size_t size = ros_message->locations.size;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    kachaka_interfaces__msg__Location * item;
    for (size_t i = 0; i < size; ++i) {
      item = &(ros_message->locations.data[i]);
      PyObject * pyitem = kachaka_interfaces__msg__location__convert_to_py(item);
      if (!pyitem) {
        Py_DECREF(field);
        return NULL;
      }
      int rc = PyList_SetItem(field, i, pyitem);
      (void)rc;
      assert(rc == 0);
    }
    assert(PySequence_Check(field));
    {
      int rc = PyObject_SetAttrString(_pymessage, "locations", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // default_location_id
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->default_location_id.data,
      strlen(ros_message->default_location_id.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "default_location_id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
