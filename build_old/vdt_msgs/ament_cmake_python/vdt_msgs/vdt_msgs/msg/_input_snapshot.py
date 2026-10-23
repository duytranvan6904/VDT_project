# generated from rosidl_generator_py/resource/_idl.py.em
# with input from vdt_msgs:msg/InputSnapshot.idl
# generated code does not contain a copyright notice

# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
from os import getenv

ros_python_check_fields = getenv('ROS_PYTHON_CHECK_FIELDS', default='')


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_InputSnapshot(type):
    """Metaclass of message 'InputSnapshot'."""

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
                'vdt_msgs.msg.InputSnapshot')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__input_snapshot
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__input_snapshot
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__input_snapshot
            cls._TYPE_SUPPORT = module.type_support_msg__msg__input_snapshot
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__input_snapshot

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class InputSnapshot(metaclass=Metaclass_InputSnapshot):
    """Message class 'InputSnapshot'."""

    __slots__ = [
        '_valid',
        '_marker_detected',
        '_align_error',
        '_altitude',
        '_delta_h',
        '_d_horiz',
        '_touchdown',
        '_planner_timeout',
        '_yaw_rate',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'valid': 'boolean',
        'marker_detected': 'boolean',
        'align_error': 'float',
        'altitude': 'float',
        'delta_h': 'float',
        'd_horiz': 'float',
        'touchdown': 'boolean',
        'planner_timeout': 'boolean',
        'yaw_rate': 'float',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
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
        self.valid = kwargs.get('valid', bool())
        self.marker_detected = kwargs.get('marker_detected', bool())
        self.align_error = kwargs.get('align_error', float())
        self.altitude = kwargs.get('altitude', float())
        self.delta_h = kwargs.get('delta_h', float())
        self.d_horiz = kwargs.get('d_horiz', float())
        self.touchdown = kwargs.get('touchdown', bool())
        self.planner_timeout = kwargs.get('planner_timeout', bool())
        self.yaw_rate = kwargs.get('yaw_rate', float())

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
        if self.valid != other.valid:
            return False
        if self.marker_detected != other.marker_detected:
            return False
        if self.align_error != other.align_error:
            return False
        if self.altitude != other.altitude:
            return False
        if self.delta_h != other.delta_h:
            return False
        if self.d_horiz != other.d_horiz:
            return False
        if self.touchdown != other.touchdown:
            return False
        if self.planner_timeout != other.planner_timeout:
            return False
        if self.yaw_rate != other.yaw_rate:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

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
    def marker_detected(self):
        """Message field 'marker_detected'."""
        return self._marker_detected

    @marker_detected.setter
    def marker_detected(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'marker_detected' field must be of type 'bool'"
        self._marker_detected = value

    @builtins.property
    def align_error(self):
        """Message field 'align_error'."""
        return self._align_error

    @align_error.setter
    def align_error(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'align_error' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'align_error' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._align_error = value

    @builtins.property
    def altitude(self):
        """Message field 'altitude'."""
        return self._altitude

    @altitude.setter
    def altitude(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'altitude' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'altitude' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._altitude = value

    @builtins.property
    def delta_h(self):
        """Message field 'delta_h'."""
        return self._delta_h

    @delta_h.setter
    def delta_h(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'delta_h' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'delta_h' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._delta_h = value

    @builtins.property
    def d_horiz(self):
        """Message field 'd_horiz'."""
        return self._d_horiz

    @d_horiz.setter
    def d_horiz(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'd_horiz' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'd_horiz' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._d_horiz = value

    @builtins.property
    def touchdown(self):
        """Message field 'touchdown'."""
        return self._touchdown

    @touchdown.setter
    def touchdown(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'touchdown' field must be of type 'bool'"
        self._touchdown = value

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

    @builtins.property
    def yaw_rate(self):
        """Message field 'yaw_rate'."""
        return self._yaw_rate

    @yaw_rate.setter
    def yaw_rate(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'yaw_rate' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'yaw_rate' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._yaw_rate = value
