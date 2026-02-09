# generated from rosidl_generator_py/resource/_idl.py.em
# with input from kachaka_interfaces:msg/Shelf.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Shelf(type):
    """Metaclass of message 'Shelf'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'DEFAULT_SPEED_MODE': 0,
        'LOW_SPEED_MODE': 1,
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
                'kachaka_interfaces.msg.Shelf')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__shelf
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__shelf
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__shelf
            cls._TYPE_SUPPORT = module.type_support_msg__msg__shelf
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__shelf

            from geometry_msgs.msg import Pose2D
            if Pose2D.__class__._TYPE_SUPPORT is None:
                Pose2D.__class__.__import_type_support__()

            from kachaka_interfaces.msg import ShelfSize
            if ShelfSize.__class__._TYPE_SUPPORT is None:
                ShelfSize.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'DEFAULT_SPEED_MODE': cls.__constants['DEFAULT_SPEED_MODE'],
            'LOW_SPEED_MODE': cls.__constants['LOW_SPEED_MODE'],
        }

    @property
    def DEFAULT_SPEED_MODE(self):
        """Message constant 'DEFAULT_SPEED_MODE'."""
        return Metaclass_Shelf.__constants['DEFAULT_SPEED_MODE']

    @property
    def LOW_SPEED_MODE(self):
        """Message constant 'LOW_SPEED_MODE'."""
        return Metaclass_Shelf.__constants['LOW_SPEED_MODE']


class Shelf(metaclass=Metaclass_Shelf):
    """
    Message class 'Shelf'.

    Constants:
      DEFAULT_SPEED_MODE
      LOW_SPEED_MODE
    """

    __slots__ = [
        '_id',
        '_name',
        '_pose',
        '_size',
        '_home_location_id',
        '_speed_mode',
    ]

    _fields_and_field_types = {
        'id': 'string',
        'name': 'string',
        'pose': 'geometry_msgs/Pose2D',
        'size': 'kachaka_interfaces/ShelfSize',
        'home_location_id': 'string',
        'speed_mode': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Pose2D'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['kachaka_interfaces', 'msg'], 'ShelfSize'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.id = kwargs.get('id', str())
        self.name = kwargs.get('name', str())
        from geometry_msgs.msg import Pose2D
        self.pose = kwargs.get('pose', Pose2D())
        from kachaka_interfaces.msg import ShelfSize
        self.size = kwargs.get('size', ShelfSize())
        self.home_location_id = kwargs.get('home_location_id', str())
        self.speed_mode = kwargs.get('speed_mode', int())

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
        if self.id != other.id:
            return False
        if self.name != other.name:
            return False
        if self.pose != other.pose:
            return False
        if self.size != other.size:
            return False
        if self.home_location_id != other.home_location_id:
            return False
        if self.speed_mode != other.speed_mode:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property  # noqa: A003
    def id(self):  # noqa: A003
        """Message field 'id'."""
        return self._id

    @id.setter  # noqa: A003
    def id(self, value):  # noqa: A003
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'id' field must be of type 'str'"
        self._id = value

    @builtins.property
    def name(self):
        """Message field 'name'."""
        return self._name

    @name.setter
    def name(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'name' field must be of type 'str'"
        self._name = value

    @builtins.property
    def pose(self):
        """Message field 'pose'."""
        return self._pose

    @pose.setter
    def pose(self, value):
        if __debug__:
            from geometry_msgs.msg import Pose2D
            assert \
                isinstance(value, Pose2D), \
                "The 'pose' field must be a sub message of type 'Pose2D'"
        self._pose = value

    @builtins.property
    def size(self):
        """Message field 'size'."""
        return self._size

    @size.setter
    def size(self, value):
        if __debug__:
            from kachaka_interfaces.msg import ShelfSize
            assert \
                isinstance(value, ShelfSize), \
                "The 'size' field must be a sub message of type 'ShelfSize'"
        self._size = value

    @builtins.property
    def home_location_id(self):
        """Message field 'home_location_id'."""
        return self._home_location_id

    @home_location_id.setter
    def home_location_id(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'home_location_id' field must be of type 'str'"
        self._home_location_id = value

    @builtins.property
    def speed_mode(self):
        """Message field 'speed_mode'."""
        return self._speed_mode

    @speed_mode.setter
    def speed_mode(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'speed_mode' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'speed_mode' field must be an unsigned integer in [0, 255]"
        self._speed_mode = value
