"""autogenerated by genpy from joint_movement/joint_state.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class joint_state(genpy.Message):
  _md5sum = "40b41a7926a59de667cedeb989b0c8d6"
  _type = "joint_movement/joint_state"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """float64 x
float64 y
float64 z
float64 xDot
float64 yDot
float64 zDot
float64 qx
float64 qy
float64 qz
float64 qw

"""
  __slots__ = ['x','y','z','xDot','yDot','zDot','qx','qy','qz','qw']
  _slot_types = ['float64','float64','float64','float64','float64','float64','float64','float64','float64','float64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       x,y,z,xDot,yDot,zDot,qx,qy,qz,qw

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(joint_state, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.x is None:
        self.x = 0.
      if self.y is None:
        self.y = 0.
      if self.z is None:
        self.z = 0.
      if self.xDot is None:
        self.xDot = 0.
      if self.yDot is None:
        self.yDot = 0.
      if self.zDot is None:
        self.zDot = 0.
      if self.qx is None:
        self.qx = 0.
      if self.qy is None:
        self.qy = 0.
      if self.qz is None:
        self.qz = 0.
      if self.qw is None:
        self.qw = 0.
    else:
      self.x = 0.
      self.y = 0.
      self.z = 0.
      self.xDot = 0.
      self.yDot = 0.
      self.zDot = 0.
      self.qx = 0.
      self.qy = 0.
      self.qz = 0.
      self.qw = 0.

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_struct_10d.pack(_x.x, _x.y, _x.z, _x.xDot, _x.yDot, _x.zDot, _x.qx, _x.qy, _x.qz, _x.qw))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      _x = self
      start = end
      end += 80
      (_x.x, _x.y, _x.z, _x.xDot, _x.yDot, _x.zDot, _x.qx, _x.qy, _x.qz, _x.qw,) = _struct_10d.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_struct_10d.pack(_x.x, _x.y, _x.z, _x.xDot, _x.yDot, _x.zDot, _x.qx, _x.qy, _x.qz, _x.qw))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 80
      (_x.x, _x.y, _x.z, _x.xDot, _x.yDot, _x.zDot, _x.qx, _x.qy, _x.qz, _x.qw,) = _struct_10d.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_10d = struct.Struct("<10d")
