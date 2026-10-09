// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from vdt_msgs:msg/InputSnapshot.idl
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
#include "vdt_msgs/msg/detail/input_snapshot__struct.h"
#include "vdt_msgs/msg/detail/input_snapshot__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool vdt_msgs__msg__input_snapshot__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[43];
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
    assert(strncmp("vdt_msgs.msg._input_snapshot.InputSnapshot", full_classname_dest, 42) == 0);
  }
  vdt_msgs__msg__InputSnapshot * ros_message = _ros_message;
  {  // valid
    PyObject * field = PyObject_GetAttrString(_pymsg, "valid");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->valid = (Py_True == field);
    Py_DECREF(field);
  }
  {  // marker_detected
    PyObject * field = PyObject_GetAttrString(_pymsg, "marker_detected");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->marker_detected = (Py_True == field);
    Py_DECREF(field);
  }
  {  // align_error
    PyObject * field = PyObject_GetAttrString(_pymsg, "align_error");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->align_error = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // altitude
    PyObject * field = PyObject_GetAttrString(_pymsg, "altitude");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->altitude = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // delta_h
    PyObject * field = PyObject_GetAttrString(_pymsg, "delta_h");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->delta_h = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // d_horiz
    PyObject * field = PyObject_GetAttrString(_pymsg, "d_horiz");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->d_horiz = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // touchdown
    PyObject * field = PyObject_GetAttrString(_pymsg, "touchdown");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->touchdown = (Py_True == field);
    Py_DECREF(field);
  }
  {  // planner_timeout
    PyObject * field = PyObject_GetAttrString(_pymsg, "planner_timeout");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->planner_timeout = (Py_True == field);
    Py_DECREF(field);
  }
  {  // yaw_rate
    PyObject * field = PyObject_GetAttrString(_pymsg, "yaw_rate");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->yaw_rate = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * vdt_msgs__msg__input_snapshot__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of InputSnapshot */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("vdt_msgs.msg._input_snapshot");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "InputSnapshot");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  vdt_msgs__msg__InputSnapshot * ros_message = (vdt_msgs__msg__InputSnapshot *)raw_ros_message;
  {  // valid
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->valid ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "valid", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // marker_detected
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->marker_detected ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "marker_detected", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // align_error
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->align_error);
    {
      int rc = PyObject_SetAttrString(_pymessage, "align_error", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // altitude
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->altitude);
    {
      int rc = PyObject_SetAttrString(_pymessage, "altitude", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // delta_h
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->delta_h);
    {
      int rc = PyObject_SetAttrString(_pymessage, "delta_h", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // d_horiz
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->d_horiz);
    {
      int rc = PyObject_SetAttrString(_pymessage, "d_horiz", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // touchdown
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->touchdown ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "touchdown", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // planner_timeout
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->planner_timeout ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "planner_timeout", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // yaw_rate
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->yaw_rate);
    {
      int rc = PyObject_SetAttrString(_pymessage, "yaw_rate", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
