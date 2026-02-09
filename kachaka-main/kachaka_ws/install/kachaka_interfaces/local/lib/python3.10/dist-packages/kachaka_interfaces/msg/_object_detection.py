# generated from rosidl_generator_py/resource/_idl.py.em
# with input from kachaka_interfaces:msg/ObjectDetection.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ObjectDetection(type):
    """Metaclass of message 'ObjectDetection'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'PERSON': 0,
        'SHELF': 1,
        'CHARGER': 2,
        'DOOR': 3,
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('kachaka_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'kachaka_interfaces.msg.ObjectDetection')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__object_detection
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__object_detection
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__object_detection
            cls._TYPE_SUPPORT = module.type_support_msg__msg__object_detection
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__object_detection

            from sensor_msgs.msg import RegionOfInterest
            if RegionOfInterest.__class__._TYPE_SUPPORT is None:
                RegionOfInterest.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'PERSON': cls.__constants['PERSON'],
            'SHELF': cls.__constants['SHELF'],
            'CHARGER': cls.__constants['CHARGER'],
            'DOOR': cls.__constants['DOOR'],
        }

    @property
    def PERSON(self):
        """Message constant 'PERSON'."""
        return Metaclass_ObjectDetection.__constants['PERSON']

    @property
    def SHELF(self):
        """Message constant 'SHELF'."""
        return Metaclass_ObjectDetection.__constants['SHELF']

    @property
    def CHARGER(self):
        """Message constant 'CHARGER'."""
        return Metaclass_ObjectDetection.__constants['CHARGER']

    @property
    def DOOR(self):
        """Message constant 'DOOR'."""
        return Metaclass_ObjectDetection.__constants['DOOR']


class ObjectDetection(metaclass=Metaclass_ObjectDetection):
    """
    Message class 'ObjectDetection'.

    Constants:
      PERSON
      SHELF
      CHARGER
      DOOR
    """

    __slots__ = [
        '_label',
        '_roi',
        '_score',
        '_distance_median',
    ]

    _fields_and_field_types = {
        'label': 'uint8',
        'roi': 'sensor_msgs/RegionOfInterest',
        'score': 'float',
        'distance_median': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['sensor_msgs', 'msg'], 'RegionOfInterest'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.label = kwargs.get('label', int())
        from sensor_msgs.msg import RegionOfInterest
        self.roi = kwargs.get('roi', RegionOfInterest())
        self.score = kwargs.get('score', float())
        self.distance_median = kwargs.get('distance_median', float())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
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
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.label != other.label:
            return False
        if self.roi != other.roi:
            return False
        if self.score != other.score:
            return False
        if self.distance_median != other.distance_median:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def label(self):
        """Message field 'label'."""
        return self._label

    @label.setter
    def label(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'label' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'label' field must be an unsigned integer in [0, 255]"
        self._label = value

    @builtins.property
    def roi(self):
        """Message field 'roi'."""
        return self._roi

    @roi.setter
    def roi(self, value):
        if __debug__:
            from sensor_msgs.msg import RegionOfInterest
            assert \
                isinstance(value, RegionOfInterest), \
                "The 'roi' field must be a sub message of type 'RegionOfInterest'"
        self._roi = value

    @builtins.property
    def score(self):
        """Message field 'score'."""
        return self._score

    @score.setter
    def score(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'score' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'score' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._score = value

    @builtins.property
    def distance_median(self):
        """Message field 'distance_median'."""
        return self._distance_median

    @distance_median.setter
    def distance_median(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'distance_median' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'distance_median' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._distance_median = value
