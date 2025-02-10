# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from thi_vision/visionTracksRequest.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import thi_vision.msg

class visionTracksRequest(genpy.Message):
  _md5sum = "7310b90945a93b256f7864c27e961494"
  _type = "thi_vision/visionTracksRequest"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """string pointCloudFile
string fileType     # "ply" "pcd" "txt"
int32  workpiece    #工件号 0:上半间 1:下半件 右面
float32 press       #下压量
float32 width       #道宽 单位mm
float32 grooveWidth #预留的槽宽
float32 avoidLeft   #x轴正方向左侧避障弧长
float32 avoidRight  #x轴正方向右侧避障弧长
float32 length      #可打磨的最长区域
orientation angleLeft  #x轴正方向左侧避障固定姿态角度(a,b,c对应法兰盘rpy)
orientation angleRight #x轴正方向右侧避障固定姿态角度
pcArea[] listPcAreas

================================================================================
MSG: thi_vision/orientation
float32 a
float32 b
float32 c

================================================================================
MSG: thi_vision/pcArea
position[] listPcPoint

================================================================================
MSG: thi_vision/position
float32 x
float32 y
float32 z
"""
  __slots__ = ['pointCloudFile','fileType','workpiece','press','width','grooveWidth','avoidLeft','avoidRight','length','angleLeft','angleRight','listPcAreas']
  _slot_types = ['string','string','int32','float32','float32','float32','float32','float32','float32','thi_vision/orientation','thi_vision/orientation','thi_vision/pcArea[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       pointCloudFile,fileType,workpiece,press,width,grooveWidth,avoidLeft,avoidRight,length,angleLeft,angleRight,listPcAreas

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(visionTracksRequest, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.pointCloudFile is None:
        self.pointCloudFile = ''
      if self.fileType is None:
        self.fileType = ''
      if self.workpiece is None:
        self.workpiece = 0
      if self.press is None:
        self.press = 0.
      if self.width is None:
        self.width = 0.
      if self.grooveWidth is None:
        self.grooveWidth = 0.
      if self.avoidLeft is None:
        self.avoidLeft = 0.
      if self.avoidRight is None:
        self.avoidRight = 0.
      if self.length is None:
        self.length = 0.
      if self.angleLeft is None:
        self.angleLeft = thi_vision.msg.orientation()
      if self.angleRight is None:
        self.angleRight = thi_vision.msg.orientation()
      if self.listPcAreas is None:
        self.listPcAreas = []
    else:
      self.pointCloudFile = ''
      self.fileType = ''
      self.workpiece = 0
      self.press = 0.
      self.width = 0.
      self.grooveWidth = 0.
      self.avoidLeft = 0.
      self.avoidRight = 0.
      self.length = 0.
      self.angleLeft = thi_vision.msg.orientation()
      self.angleRight = thi_vision.msg.orientation()
      self.listPcAreas = []

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
      _x = self.pointCloudFile
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.fileType
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_i12f().pack(_x.workpiece, _x.press, _x.width, _x.grooveWidth, _x.avoidLeft, _x.avoidRight, _x.length, _x.angleLeft.a, _x.angleLeft.b, _x.angleLeft.c, _x.angleRight.a, _x.angleRight.b, _x.angleRight.c))
      length = len(self.listPcAreas)
      buff.write(_struct_I.pack(length))
      for val1 in self.listPcAreas:
        length = len(val1.listPcPoint)
        buff.write(_struct_I.pack(length))
        for val2 in val1.listPcPoint:
          _x = val2
          buff.write(_get_struct_3f().pack(_x.x, _x.y, _x.z))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.angleLeft is None:
        self.angleLeft = thi_vision.msg.orientation()
      if self.angleRight is None:
        self.angleRight = thi_vision.msg.orientation()
      if self.listPcAreas is None:
        self.listPcAreas = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.pointCloudFile = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.pointCloudFile = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.fileType = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.fileType = str[start:end]
      _x = self
      start = end
      end += 52
      (_x.workpiece, _x.press, _x.width, _x.grooveWidth, _x.avoidLeft, _x.avoidRight, _x.length, _x.angleLeft.a, _x.angleLeft.b, _x.angleLeft.c, _x.angleRight.a, _x.angleRight.b, _x.angleRight.c,) = _get_struct_i12f().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.listPcAreas = []
      for i in range(0, length):
        val1 = thi_vision.msg.pcArea()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        val1.listPcPoint = []
        for i in range(0, length):
          val2 = thi_vision.msg.position()
          _x = val2
          start = end
          end += 12
          (_x.x, _x.y, _x.z,) = _get_struct_3f().unpack(str[start:end])
          val1.listPcPoint.append(val2)
        self.listPcAreas.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self.pointCloudFile
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.fileType
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_i12f().pack(_x.workpiece, _x.press, _x.width, _x.grooveWidth, _x.avoidLeft, _x.avoidRight, _x.length, _x.angleLeft.a, _x.angleLeft.b, _x.angleLeft.c, _x.angleRight.a, _x.angleRight.b, _x.angleRight.c))
      length = len(self.listPcAreas)
      buff.write(_struct_I.pack(length))
      for val1 in self.listPcAreas:
        length = len(val1.listPcPoint)
        buff.write(_struct_I.pack(length))
        for val2 in val1.listPcPoint:
          _x = val2
          buff.write(_get_struct_3f().pack(_x.x, _x.y, _x.z))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.angleLeft is None:
        self.angleLeft = thi_vision.msg.orientation()
      if self.angleRight is None:
        self.angleRight = thi_vision.msg.orientation()
      if self.listPcAreas is None:
        self.listPcAreas = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.pointCloudFile = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.pointCloudFile = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.fileType = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.fileType = str[start:end]
      _x = self
      start = end
      end += 52
      (_x.workpiece, _x.press, _x.width, _x.grooveWidth, _x.avoidLeft, _x.avoidRight, _x.length, _x.angleLeft.a, _x.angleLeft.b, _x.angleLeft.c, _x.angleRight.a, _x.angleRight.b, _x.angleRight.c,) = _get_struct_i12f().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.listPcAreas = []
      for i in range(0, length):
        val1 = thi_vision.msg.pcArea()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        val1.listPcPoint = []
        for i in range(0, length):
          val2 = thi_vision.msg.position()
          _x = val2
          start = end
          end += 12
          (_x.x, _x.y, _x.z,) = _get_struct_3f().unpack(str[start:end])
          val1.listPcPoint.append(val2)
        self.listPcAreas.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_3f = None
def _get_struct_3f():
    global _struct_3f
    if _struct_3f is None:
        _struct_3f = struct.Struct("<3f")
    return _struct_3f
_struct_i12f = None
def _get_struct_i12f():
    global _struct_i12f
    if _struct_i12f is None:
        _struct_i12f = struct.Struct("<i12f")
    return _struct_i12f
# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from thi_vision/visionTracksResponse.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import thi_vision.msg

class visionTracksResponse(genpy.Message):
  _md5sum = "a33069b7a2e523cfca40ce4212d31d16"
  _type = "thi_vision/visionTracksResponse"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """track[] tracks
bool ok


================================================================================
MSG: thi_vision/track
pose[] track

================================================================================
MSG: thi_vision/pose
position point
orientation angle

================================================================================
MSG: thi_vision/position
float32 x
float32 y
float32 z

================================================================================
MSG: thi_vision/orientation
float32 a
float32 b
float32 c
"""
  __slots__ = ['tracks','ok']
  _slot_types = ['thi_vision/track[]','bool']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       tracks,ok

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(visionTracksResponse, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.tracks is None:
        self.tracks = []
      if self.ok is None:
        self.ok = False
    else:
      self.tracks = []
      self.ok = False

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
      length = len(self.tracks)
      buff.write(_struct_I.pack(length))
      for val1 in self.tracks:
        length = len(val1.track)
        buff.write(_struct_I.pack(length))
        for val2 in val1.track:
          _v1 = val2.point
          _x = _v1
          buff.write(_get_struct_3f().pack(_x.x, _x.y, _x.z))
          _v2 = val2.angle
          _x = _v2
          buff.write(_get_struct_3f().pack(_x.a, _x.b, _x.c))
      _x = self.ok
      buff.write(_get_struct_B().pack(_x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.tracks is None:
        self.tracks = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.tracks = []
      for i in range(0, length):
        val1 = thi_vision.msg.track()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        val1.track = []
        for i in range(0, length):
          val2 = thi_vision.msg.pose()
          _v3 = val2.point
          _x = _v3
          start = end
          end += 12
          (_x.x, _x.y, _x.z,) = _get_struct_3f().unpack(str[start:end])
          _v4 = val2.angle
          _x = _v4
          start = end
          end += 12
          (_x.a, _x.b, _x.c,) = _get_struct_3f().unpack(str[start:end])
          val1.track.append(val2)
        self.tracks.append(val1)
      start = end
      end += 1
      (self.ok,) = _get_struct_B().unpack(str[start:end])
      self.ok = bool(self.ok)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      length = len(self.tracks)
      buff.write(_struct_I.pack(length))
      for val1 in self.tracks:
        length = len(val1.track)
        buff.write(_struct_I.pack(length))
        for val2 in val1.track:
          _v5 = val2.point
          _x = _v5
          buff.write(_get_struct_3f().pack(_x.x, _x.y, _x.z))
          _v6 = val2.angle
          _x = _v6
          buff.write(_get_struct_3f().pack(_x.a, _x.b, _x.c))
      _x = self.ok
      buff.write(_get_struct_B().pack(_x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.tracks is None:
        self.tracks = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.tracks = []
      for i in range(0, length):
        val1 = thi_vision.msg.track()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        val1.track = []
        for i in range(0, length):
          val2 = thi_vision.msg.pose()
          _v7 = val2.point
          _x = _v7
          start = end
          end += 12
          (_x.x, _x.y, _x.z,) = _get_struct_3f().unpack(str[start:end])
          _v8 = val2.angle
          _x = _v8
          start = end
          end += 12
          (_x.a, _x.b, _x.c,) = _get_struct_3f().unpack(str[start:end])
          val1.track.append(val2)
        self.tracks.append(val1)
      start = end
      end += 1
      (self.ok,) = _get_struct_B().unpack(str[start:end])
      self.ok = bool(self.ok)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_3f = None
def _get_struct_3f():
    global _struct_3f
    if _struct_3f is None:
        _struct_3f = struct.Struct("<3f")
    return _struct_3f
_struct_B = None
def _get_struct_B():
    global _struct_B
    if _struct_B is None:
        _struct_B = struct.Struct("<B")
    return _struct_B
class visionTracks(object):
  _type          = 'thi_vision/visionTracks'
  _md5sum = '54640a39f52ffd596022a5e1d0dc6ffe'
  _request_class  = visionTracksRequest
  _response_class = visionTracksResponse
