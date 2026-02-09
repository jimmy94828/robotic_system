# generated from rosidl_generator_py/resource/_idl.py.em
# with input from kachaka_interfaces:msg/KachakaCommand.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_KachakaCommand(type):
    """Metaclass of message 'KachakaCommand'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'COMMAND_TYPE_OTHER': 0,
        'MOVE_SHELF_COMMAND': 1,
        'RETURN_SHELF_COMMAND': 2,
        'UNDOCK_SHELF_COMMAND': 5,
        'MOVE_TO_LOCATION_COMMAND': 7,
        'RETURN_HOME_COMMAND': 8,
        'DOCK_SHELF_COMMAND': 9,
        'SPEAK_COMMAND': 12,
        'MOVE_TO_POSE_COMMAND': 13,
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
                'kachaka_interfaces.msg.KachakaCommand')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__kachaka_command
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__kachaka_command
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__kachaka_command
            cls._TYPE_SUPPORT = module.type_support_msg__msg__kachaka_command
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__kachaka_command

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'COMMAND_TYPE_OTHER': cls.__constants['COMMAND_TYPE_OTHER'],
            'MOVE_SHELF_COMMAND': cls.__constants['MOVE_SHELF_COMMAND'],
            'RETURN_SHELF_COMMAND': cls.__constants['RETURN_SHELF_COMMAND'],
            'UNDOCK_SHELF_COMMAND': cls.__constants['UNDOCK_SHELF_COMMAND'],
            'MOVE_TO_LOCATION_COMMAND': cls.__constants['MOVE_TO_LOCATION_COMMAND'],
            'RETURN_HOME_COMMAND': cls.__constants['RETURN_HOME_COMMAND'],
            'DOCK_SHELF_COMMAND': cls.__constants['DOCK_SHELF_COMMAND'],
            'SPEAK_COMMAND': cls.__constants['SPEAK_COMMAND'],
            'MOVE_TO_POSE_COMMAND': cls.__constants['MOVE_TO_POSE_COMMAND'],
        }

    @property
    def COMMAND_TYPE_OTHER(self):
        """Message constant 'COMMAND_TYPE_OTHER'."""
        return Metaclass_KachakaCommand.__constants['COMMAND_TYPE_OTHER']

    @property
    def MOVE_SHELF_COMMAND(self):
        """Message constant 'MOVE_SHELF_COMMAND'."""
        return Metaclass_KachakaCommand.__constants['MOVE_SHELF_COMMAND']

    @property
    def RETURN_SHELF_COMMAND(self):
        """Message constant 'RETURN_SHELF_COMMAND'."""
        return Metaclass_KachakaCommand.__constants['RETURN_SHELF_COMMAND']

    @property
    def UNDOCK_SHELF_COMMAND(self):
        """Message constant 'UNDOCK_SHELF_COMMAND'."""
        return Metaclass_KachakaCommand.__constants['UNDOCK_SHELF_COMMAND']

    @property
    def MOVE_TO_LOCATION_COMMAND(self):
        """Message constant 'MOVE_TO_LOCATION_COMMAND'."""
        return Metaclass_KachakaCommand.__constants['MOVE_TO_LOCATION_COMMAND']

    @property
    def RETURN_HOME_COMMAND(self):
        """Message constant 'RETURN_HOME_COMMAND'."""
        return Metaclass_KachakaCommand.__constants['RETURN_HOME_COMMAND']

    @property
    def DOCK_SHELF_COMMAND(self):
        """Message constant 'DOCK_SHELF_COMMAND'."""
        return Metaclass_KachakaCommand.__constants['DOCK_SHELF_COMMAND']

    @property
    def SPEAK_COMMAND(self):
        """Message constant 'SPEAK_COMMAND'."""
        return Metaclass_KachakaCommand.__constants['SPEAK_COMMAND']

    @property
    def MOVE_TO_POSE_COMMAND(self):
        """Message constant 'MOVE_TO_POSE_COMMAND'."""
        return Metaclass_KachakaCommand.__constants['MOVE_TO_POSE_COMMAND']


class KachakaCommand(metaclass=Metaclass_KachakaCommand):
    """
    Message class 'KachakaCommand'.

    Constants:
      COMMAND_TYPE_OTHER
      MOVE_SHELF_COMMAND
      RETURN_SHELF_COMMAND
      UNDOCK_SHELF_COMMAND
      MOVE_TO_LOCATION_COMMAND
      RETURN_HOME_COMMAND
      DOCK_SHELF_COMMAND
      SPEAK_COMMAND
      MOVE_TO_POSE_COMMAND
    """

    __slots__ = [
        '_command_type',
        '_move_shelf_command_target_shelf_id',
        '_move_shelf_command_destination_location_id',
        '_move_shelf_command_undock_on_destination',
        '_return_shelf_command_target_shelf_id',
        '_undock_shelf_command_target_shelf_id',
        '_move_to_location_command_target_location_id',
        '_return_home_command_silent',
        '_speak_command_text',
        '_move_to_pose_command_x',
        '_move_to_pose_command_y',
        '_move_to_pose_command_yaw',
    ]

    _fields_and_field_types = {
        'command_type': 'uint8',
        'move_shelf_command_target_shelf_id': 'string',
        'move_shelf_command_destination_location_id': 'string',
        'move_shelf_command_undock_on_destination': 'boolean',
        'return_shelf_command_target_shelf_id': 'string',
        'undock_shelf_command_target_shelf_id': 'string',
        'move_to_location_command_target_location_id': 'string',
        'return_home_command_silent': 'boolean',
        'speak_command_text': 'string',
        'move_to_pose_command_x': 'double',
        'move_to_pose_command_y': 'double',
        'move_to_pose_command_yaw': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.command_type = kwargs.get('command_type', int())
        self.move_shelf_command_target_shelf_id = kwargs.get('move_shelf_command_target_shelf_id', str())
        self.move_shelf_command_destination_location_id = kwargs.get('move_shelf_command_destination_location_id', str())
        self.move_shelf_command_undock_on_destination = kwargs.get('move_shelf_command_undock_on_destination', bool())
        self.return_shelf_command_target_shelf_id = kwargs.get('return_shelf_command_target_shelf_id', str())
        self.undock_shelf_command_target_shelf_id = kwargs.get('undock_shelf_command_target_shelf_id', str())
        self.move_to_location_command_target_location_id = kwargs.get('move_to_location_command_target_location_id', str())
        self.return_home_command_silent = kwargs.get('return_home_command_silent', bool())
        self.speak_command_text = kwargs.get('speak_command_text', str())
        self.move_to_pose_command_x = kwargs.get('move_to_pose_command_x', float())
        self.move_to_pose_command_y = kwargs.get('move_to_pose_command_y', float())
        self.move_to_pose_command_yaw = kwargs.get('move_to_pose_command_yaw', float())

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
        if self.command_type != other.command_type:
            return False
        if self.move_shelf_command_target_shelf_id != other.move_shelf_command_target_shelf_id:
            return False
        if self.move_shelf_command_destination_location_id != other.move_shelf_command_destination_location_id:
            return False
        if self.move_shelf_command_undock_on_destination != other.move_shelf_command_undock_on_destination:
            return False
        if self.return_shelf_command_target_shelf_id != other.return_shelf_command_target_shelf_id:
            return False
        if self.undock_shelf_command_target_shelf_id != other.undock_shelf_command_target_shelf_id:
            return False
        if self.move_to_location_command_target_location_id != other.move_to_location_command_target_location_id:
            return False
        if self.return_home_command_silent != other.return_home_command_silent:
            return False
        if self.speak_command_text != other.speak_command_text:
            return False
        if self.move_to_pose_command_x != other.move_to_pose_command_x:
            return False
        if self.move_to_pose_command_y != other.move_to_pose_command_y:
            return False
        if self.move_to_pose_command_yaw != other.move_to_pose_command_yaw:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def command_type(self):
        """Message field 'command_type'."""
        return self._command_type

    @command_type.setter
    def command_type(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'command_type' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'command_type' field must be an unsigned integer in [0, 255]"
        self._command_type = value

    @builtins.property
    def move_shelf_command_target_shelf_id(self):
        """Message field 'move_shelf_command_target_shelf_id'."""
        return self._move_shelf_command_target_shelf_id

    @move_shelf_command_target_shelf_id.setter
    def move_shelf_command_target_shelf_id(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'move_shelf_command_target_shelf_id' field must be of type 'str'"
        self._move_shelf_command_target_shelf_id = value

    @builtins.property
    def move_shelf_command_destination_location_id(self):
        """Message field 'move_shelf_command_destination_location_id'."""
        return self._move_shelf_command_destination_location_id

    @move_shelf_command_destination_location_id.setter
    def move_shelf_command_destination_location_id(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'move_shelf_command_destination_location_id' field must be of type 'str'"
        self._move_shelf_command_destination_location_id = value

    @builtins.property
    def move_shelf_command_undock_on_destination(self):
        """Message field 'move_shelf_command_undock_on_destination'."""
        return self._move_shelf_command_undock_on_destination

    @move_shelf_command_undock_on_destination.setter
    def move_shelf_command_undock_on_destination(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'move_shelf_command_undock_on_destination' field must be of type 'bool'"
        self._move_shelf_command_undock_on_destination = value

    @builtins.property
    def return_shelf_command_target_shelf_id(self):
        """Message field 'return_shelf_command_target_shelf_id'."""
        return self._return_shelf_command_target_shelf_id

    @return_shelf_command_target_shelf_id.setter
    def return_shelf_command_target_shelf_id(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'return_shelf_command_target_shelf_id' field must be of type 'str'"
        self._return_shelf_command_target_shelf_id = value

    @builtins.property
    def undock_shelf_command_target_shelf_id(self):
        """Message field 'undock_shelf_command_target_shelf_id'."""
        return self._undock_shelf_command_target_shelf_id

    @undock_shelf_command_target_shelf_id.setter
    def undock_shelf_command_target_shelf_id(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'undock_shelf_command_target_shelf_id' field must be of type 'str'"
        self._undock_shelf_command_target_shelf_id = value

    @builtins.property
    def move_to_location_command_target_location_id(self):
        """Message field 'move_to_location_command_target_location_id'."""
        return self._move_to_location_command_target_location_id

    @move_to_location_command_target_location_id.setter
    def move_to_location_command_target_location_id(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'move_to_location_command_target_location_id' field must be of type 'str'"
        self._move_to_location_command_target_location_id = value

    @builtins.property
    def return_home_command_silent(self):
        """Message field 'return_home_command_silent'."""
        return self._return_home_command_silent

    @return_home_command_silent.setter
    def return_home_command_silent(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'return_home_command_silent' field must be of type 'bool'"
        self._return_home_command_silent = value

    @builtins.property
    def speak_command_text(self):
        """Message field 'speak_command_text'."""
        return self._speak_command_text

    @speak_command_text.setter
    def speak_command_text(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'speak_command_text' field must be of type 'str'"
        self._speak_command_text = value

    @builtins.property
    def move_to_pose_command_x(self):
        """Message field 'move_to_pose_command_x'."""
        return self._move_to_pose_command_x

    @move_to_pose_command_x.setter
    def move_to_pose_command_x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'move_to_pose_command_x' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'move_to_pose_command_x' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._move_to_pose_command_x = value

    @builtins.property
    def move_to_pose_command_y(self):
        """Message field 'move_to_pose_command_y'."""
        return self._move_to_pose_command_y

    @move_to_pose_command_y.setter
    def move_to_pose_command_y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'move_to_pose_command_y' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'move_to_pose_command_y' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._move_to_pose_command_y = value

    @builtins.property
    def move_to_pose_command_yaw(self):
        """Message field 'move_to_pose_command_yaw'."""
        return self._move_to_pose_command_yaw

    @move_to_pose_command_yaw.setter
    def move_to_pose_command_yaw(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'move_to_pose_command_yaw' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'move_to_pose_command_yaw' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._move_to_pose_command_yaw = value
