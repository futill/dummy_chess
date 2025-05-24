# generated from rosidl_generator_py/resource/_idl.py.em
# with input from my_robot_pkg_msg:msg/ChessMove.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ChessMove(type):
    """Metaclass of message 'ChessMove'."""

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
            module = import_type_support('my_robot_pkg_msg')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'my_robot_pkg_msg.msg.ChessMove')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__chess_move
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__chess_move
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__chess_move
            cls._TYPE_SUPPORT = module.type_support_msg__msg__chess_move
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__chess_move

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ChessMove(metaclass=Metaclass_ChessMove):
    """Message class 'ChessMove'."""

    __slots__ = [
        '_grid_index',
        '_color',
    ]

    _fields_and_field_types = {
        'grid_index': 'uint8',
        'color': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.grid_index = kwargs.get('grid_index', int())
        self.color = kwargs.get('color', str())

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
        if self.grid_index != other.grid_index:
            return False
        if self.color != other.color:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def grid_index(self):
        """Message field 'grid_index'."""
        return self._grid_index

    @grid_index.setter
    def grid_index(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'grid_index' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'grid_index' field must be an unsigned integer in [0, 255]"
        self._grid_index = value

    @builtins.property
    def color(self):
        """Message field 'color'."""
        return self._color

    @color.setter
    def color(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'color' field must be of type 'str'"
        self._color = value
