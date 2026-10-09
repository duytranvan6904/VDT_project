// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from vdt_msgs:msg/RcFsmInput.idl
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
#include "vdt_msgs/msg/detail/rc_fsm_input__struct.h"
#include "vdt_msgs/msg/detail/rc_fsm_input__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool vdt_msgs__msg__rc_fsm_input__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[38];
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
    assert(strncmp("vdt_msgs.msg._rc_fsm_input.RcFsmInput", full_classname_dest, 37) == 0);
  }
  vdt_msgs__msg__RcFsmInput * ros_message = _ros_message;
  {  // land_switch
    PyObject * field = PyObject_GetAttrString(_pymsg, "land_switch");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->land_switch = (Py_True == field);
    Py_DECREF(field);
  }
  {  // kill_switch
    PyObject * field = PyObject_GetAttrString(_pymsg, "kill_switch");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->kill_switch = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * vdt_msgs__msg__rc_fsm_input__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of RcFsmInput */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("vdt_msgs.msg._rc_fsm_input");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "RcFsmInput");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  vdt_msgs__msg__RcFsmInput * ros_message = (vdt_msgs__msg__RcFsmInput *)raw_ros_message;
  {  // land_switch
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->land_switch ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "land_switch", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // kill_switch
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->kill_switch ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "kill_switch", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
