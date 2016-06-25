# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from ucl_drone/Pose3D.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import std_msgs.msg

class Pose3D(genpy.Message):
  _md5sum = "61cfa76c32b638d5304ee331d87c6c2b"
  _type = "ucl_drone/Pose3D"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """
# This represents an estimate of a position and velocity in 3D space.
# The pose in this message should be specified in an absolute coordinate frame.

# ucl definition of a pose message.
# TODO: unites?
# Also add velocities for the controller purpose.
# TODO: why exclude acceleration?

Header header

float64 x
float64 y
float64 z

#float64 quatX
#float64 quatY
#float64 quatZ
#float64 quatW

float64 rotX
float64 rotY
float64 rotZ

float64 xvel
float64 yvel
float64 zvel

float64 rotXvel
float64 rotYvel
float64 rotZvel

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id
"""
  __slots__ = ['header','x','y','z','rotX','rotY','rotZ','xvel','yvel','zvel','rotXvel','rotYvel','rotZvel']
  _slot_types = ['std_msgs/Header','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,x,y,z,rotX,rotY,rotZ,xvel,yvel,zvel,rotXvel,rotYvel,rotZvel

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Pose3D, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.x is None:
        self.x = 0.
      if self.y is None:
        self.y = 0.
      if self.z is None:
        self.z = 0.
      if self.rotX is None:
        self.rotX = 0.
      if self.rotY is None:
        self.rotY = 0.
      if self.rotZ is None:
        self.rotZ = 0.
      if self.xvel is None:
        self.xvel = 0.
      if self.yvel is None:
        self.yvel = 0.
      if self.zvel is None:
        self.zvel = 0.
      if self.rotXvel is None:
        self.rotXvel = 0.
      if self.rotYvel is None:
        self.rotYvel = 0.
      if self.rotZvel is None:
        self.rotZvel = 0.
    else:
      self.header = std_msgs.msg.Header()
      self.x = 0.
      self.y = 0.
      self.z = 0.
      self.rotX = 0.
      self.rotY = 0.
      self.rotZ = 0.
      self.xvel = 0.
      self.yvel = 0.
      self.zvel = 0.
      self.rotXvel = 0.
      self.rotYvel = 0.
      self.rotZvel = 0.

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
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      if python3:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_12d.pack(_x.x, _x.y, _x.z, _x.rotX, _x.rotY, _x.rotZ, _x.xvel, _x.yvel, _x.zvel, _x.rotXvel, _x.rotYvel, _x.rotZvel))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 96
      (_x.x, _x.y, _x.z, _x.rotX, _x.rotY, _x.rotZ, _x.xvel, _x.yvel, _x.zvel, _x.rotXvel, _x.rotYvel, _x.rotZvel,) = _struct_12d.unpack(str[start:end])
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
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      if python3:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_12d.pack(_x.x, _x.y, _x.z, _x.rotX, _x.rotY, _x.rotZ, _x.xvel, _x.yvel, _x.zvel, _x.rotXvel, _x.rotYvel, _x.rotZvel))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 96
      (_x.x, _x.y, _x.z, _x.rotX, _x.rotY, _x.rotZ, _x.xvel, _x.yvel, _x.zvel, _x.rotXvel, _x.rotYvel, _x.rotZvel,) = _struct_12d.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_3I = struct.Struct("<3I")
_struct_12d = struct.Struct("<12d")
