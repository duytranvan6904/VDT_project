// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from vdt_msgs:msg/OffboardStatus.idl
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
#include "vdt_msgs/msg/detail/offboard_status__struct.h"
#include "vdt_msgs/msg/detail/offboard_status__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool vdt_msgs__msg__offboard_status__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[45];
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
    assert(strncmp("vdt_msgs.msg._offboard_status.OffboardStatus", full_classname_dest, 44) == 0);
  }
  vdt_msgs__msg__OffboardStatus * ros_message = _ros_message;
  {  // offboard_active
    PyObject * field = PyObject_GetAttrString(_pymsg, "offboard_active");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->offboard_active = (Py_True == field);
    Py_DECREF(field);
  }
  {  // heartbeat_age_sec
    PyObject * field = PyObject_GetAttrString(_pymsg, "heartbeat_age_sec");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->heartbeat_age_sec = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * vdt_msgs__msg__offboard_status__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of OffboardStatus */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("vdt_msgs.msg._offboard_status");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "OffboardStatus");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  vdt_msgs__msg__OffboardStatus * ros_message = (vdt_msgs__msg__OffboardStatus *)raw_ros_message;
  {  // offboard_active
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->offboard_active ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "offboard_active", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // heartbeat_age_sec
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->heartbeat_age_sec);
    {
      int rc = PyObject_SetAttrString(_pymessage, "heartbeat_age_sec", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
