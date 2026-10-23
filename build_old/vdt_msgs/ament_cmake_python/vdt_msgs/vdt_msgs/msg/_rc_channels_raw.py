# generated from rosidl_generator_py/resource/_idl.py.em
# with input from vdt_msgs:msg/RcChannelsRaw.idl
# generated code does not contain a copyright notice

# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
from os import getenv

ros_python_check_fields = getenv('ROS_PYTHON_CHECK_FIELDS', default='')


# Import statements for member types

import builtins  # noqa: E402, I100

# Member 'ch'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_RcChannelsRaw(type):
    """Metaclass of message 'RcChannelsRaw'."""

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
                'vdt_msgs.msg.RcChannelsRaw')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__rc_channels_raw
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__rc_channels_raw
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__rc_channels_raw
            cls._TYPE_SUPPORT = module.type_support_msg__msg__rc_channels_raw
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__rc_channels_raw

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class RcChannelsRaw(metaclass=Metaclass_RcChannelsRaw):
    """Message class 'RcChannelsRaw'."""

    __slots__ = [
        '_ch',
        '_valid',
        '_failsafe',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'ch': 'int16[16]',
        'valid': 'boolean',
        'failsafe': 'boolean',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('int16'), 16),  # noqa: E501
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
        if 'ch' not in kwargs:
            self.ch = numpy.zeros(16, dtype=numpy.int16)
        else:
            self.ch = kwargs.get('ch')
        self.valid = kwargs.get('valid', bool())
        self.failsafe = kwargs.get('failsafe', bool())

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
        if any(self.ch != other.ch):
            return False
        if self.valid != other.valid:
            return False
        if self.failsafe != other.failsafe:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def ch(self):
        """Message field 'ch'."""
        return self._ch

    @ch.setter
    def ch(self, value):
        if self._check_fields:
            if isinstance(value, numpy.ndarray):
                assert value.dtype == numpy.int16, \
                    "The 'ch' numpy.ndarray() must have the dtype of 'numpy.int16'"
                assert value.size == 16, \
                    "The 'ch' numpy.ndarray() must have a size of 16"
                self._ch = value
                return
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 16 and
                 all(isinstance(v, int) for v in value) and
                 all(val >= -32768 and val < 32768 for val in value)), \
                "The 'ch' field must be a set or sequence with length 16 and each value of type 'int' and each integer in [-32768, 32767]"
        self._ch = numpy.array(value, dtype=numpy.int16)

    @builtins.property
    def valid(self):
        """Message field 'valid'."""
        return self._valid

    @valid.setter
    def valid(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'valid' field must be of type 'bool'"
        self._valid = value

    @builtins.property
    def failsafe(self):
        """Message field 'failsafe'."""
        return self._failsafe

    @failsafe.setter
    def failsafe(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'failsafe' field must be of type 'bool'"
        self._failsafe = value
