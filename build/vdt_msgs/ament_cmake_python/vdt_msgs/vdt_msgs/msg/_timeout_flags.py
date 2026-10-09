# generated from rosidl_generator_py/resource/_idl.py.em
# with input from vdt_msgs:msg/TimeoutFlags.idl
# generated code does not contain a copyright notice

# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
from os import getenv

ros_python_check_fields = getenv('ROS_PYTHON_CHECK_FIELDS', default='')


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_TimeoutFlags(type):
    """Metaclass of message 'TimeoutFlags'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('vdt_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'vdt_msgs.msg.TimeoutFlags')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__timeout_flags
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__timeout_flags
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__timeout_flags
            cls._TYPE_SUPPORT = module.type_support_msg__msg__timeout_flags
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__timeout_flags

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class TimeoutFlags(metaclass=Metaclass_TimeoutFlags):
    """Message class 'TimeoutFlags'."""

    __slots__ = [
        '_ekf_timeout',
        '_vision_timeout',
        '_alt_timeout',
        '_planner_timeout',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'ekf_timeout': 'boolean',
        'vision_timeout': 'boolean',
        'alt_timeout': 'boolean',
        'planner_timeout': 'boolean',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        if 'check_fields' in kwargs:
            self._check_fields = kwargs['check_fields']
        else:
            self._check_fields = ros_python_check_fields == '1'
        if self._check_fields:
            assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
                'Invalid arguments passed to constructor: %s' % \
                ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.ekf_timeout = kwargs.get('ekf_timeout', bool())
        self.vision_timeout = kwargs.get('vision_timeout', bool())
        self.alt_timeout = kwargs.get('alt_timeout', bool())
        self.planner_timeout = kwargs.get('planner_timeout', bool())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.get_fields_and_field_types().keys(), self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    if self._check_fields:
                        assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.ekf_timeout != other.ekf_timeout:
            return False
        if self.vision_timeout != other.vision_timeout:
            return False
        if self.alt_timeout != other.alt_timeout:
            return False
        if self.planner_timeout != other.planner_timeout:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def ekf_timeout(self):
        """Message field 'ekf_timeout'."""
        return self._ekf_timeout

    @ekf_timeout.setter
    def ekf_timeout(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'ekf_timeout' field must be of type 'bool'"
        self._ekf_timeout = value

    @builtins.property
    def vision_timeout(self):
        """Message field 'vision_timeout'."""
        return self._vision_timeout

    @vision_timeout.setter
    def vision_timeout(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'vision_timeout' field must be of type 'bool'"
        self._vision_timeout = value

    @builtins.property
    def alt_timeout(self):
        """Message field 'alt_timeout'."""
        return self._alt_timeout

    @alt_timeout.setter
    def alt_timeout(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'alt_timeout' field must be of type 'bool'"
        self._alt_timeout = value

    @builtins.property
    def planner_timeout(self):
        """Message field 'planner_timeout'."""
        return self._planner_timeout

    @planner_timeout.setter
    def planner_timeout(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'planner_timeout' field must be of type 'bool'"
        self._planner_timeout = value
