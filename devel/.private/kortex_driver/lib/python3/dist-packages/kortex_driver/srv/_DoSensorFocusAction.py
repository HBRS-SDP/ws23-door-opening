# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from kortex_driver/DoSensorFocusActionRequest.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import kortex_driver.msg

class DoSensorFocusActionRequest(genpy.Message):
  _md5sum = "7045797c32f93f031d0933c5fd38ad58"
  _type = "kortex_driver/DoSensorFocusActionRequest"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """SensorFocusAction input

================================================================================
MSG: kortex_driver/SensorFocusAction

uint32 sensor
uint32 focus_action
SensorFocusAction_action_parameters oneof_action_parameters
================================================================================
MSG: kortex_driver/SensorFocusAction_action_parameters

FocusPoint[] focus_point
ManualFocus[] manual_focus
================================================================================
MSG: kortex_driver/FocusPoint

uint32 x
uint32 y
================================================================================
MSG: kortex_driver/ManualFocus

uint32 value"""
  __slots__ = ['input']
  _slot_types = ['kortex_driver/SensorFocusAction']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       input

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(DoSensorFocusActionRequest, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.input is None:
        self.input = kortex_driver.msg.SensorFocusAction()
    else:
      self.input = kortex_driver.msg.SensorFocusAction()

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
      buff.write(_get_struct_2I().pack(_x.input.sensor, _x.input.focus_action))
      length = len(self.input.oneof_action_parameters.focus_point)
      buff.write(_struct_I.pack(length))
      for val1 in self.input.oneof_action_parameters.focus_point:
        _x = val1
        buff.write(_get_struct_2I().pack(_x.x, _x.y))
      length = len(self.input.oneof_action_parameters.manual_focus)
      buff.write(_struct_I.pack(length))
      for val1 in self.input.oneof_action_parameters.manual_focus:
        _x = val1.value
        buff.write(_get_struct_I().pack(_x))
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
      if self.input is None:
        self.input = kortex_driver.msg.SensorFocusAction()
      end = 0
      _x = self
      start = end
      end += 8
      (_x.input.sensor, _x.input.focus_action,) = _get_struct_2I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.input.oneof_action_parameters.focus_point = []
      for i in range(0, length):
        val1 = kortex_driver.msg.FocusPoint()
        _x = val1
        start = end
        end += 8
        (_x.x, _x.y,) = _get_struct_2I().unpack(str[start:end])
        self.input.oneof_action_parameters.focus_point.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.input.oneof_action_parameters.manual_focus = []
      for i in range(0, length):
        val1 = kortex_driver.msg.ManualFocus()
        start = end
        end += 4
        (val1.value,) = _get_struct_I().unpack(str[start:end])
        self.input.oneof_action_parameters.manual_focus.append(val1)
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
      _x = self
      buff.write(_get_struct_2I().pack(_x.input.sensor, _x.input.focus_action))
      length = len(self.input.oneof_action_parameters.focus_point)
      buff.write(_struct_I.pack(length))
      for val1 in self.input.oneof_action_parameters.focus_point:
        _x = val1
        buff.write(_get_struct_2I().pack(_x.x, _x.y))
      length = len(self.input.oneof_action_parameters.manual_focus)
      buff.write(_struct_I.pack(length))
      for val1 in self.input.oneof_action_parameters.manual_focus:
        _x = val1.value
        buff.write(_get_struct_I().pack(_x))
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
      if self.input is None:
        self.input = kortex_driver.msg.SensorFocusAction()
      end = 0
      _x = self
      start = end
      end += 8
      (_x.input.sensor, _x.input.focus_action,) = _get_struct_2I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.input.oneof_action_parameters.focus_point = []
      for i in range(0, length):
        val1 = kortex_driver.msg.FocusPoint()
        _x = val1
        start = end
        end += 8
        (_x.x, _x.y,) = _get_struct_2I().unpack(str[start:end])
        self.input.oneof_action_parameters.focus_point.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.input.oneof_action_parameters.manual_focus = []
      for i in range(0, length):
        val1 = kortex_driver.msg.ManualFocus()
        start = end
        end += 4
        (val1.value,) = _get_struct_I().unpack(str[start:end])
        self.input.oneof_action_parameters.manual_focus.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_2I = None
def _get_struct_2I():
    global _struct_2I
    if _struct_2I is None:
        _struct_2I = struct.Struct("<2I")
    return _struct_2I
# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from kortex_driver/DoSensorFocusActionResponse.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import kortex_driver.msg

class DoSensorFocusActionResponse(genpy.Message):
  _md5sum = "c6c43d221c810050f75091660f63b0cd"
  _type = "kortex_driver/DoSensorFocusActionResponse"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """Empty output

================================================================================
MSG: kortex_driver/Empty
"""
  __slots__ = ['output']
  _slot_types = ['kortex_driver/Empty']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       output

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(DoSensorFocusActionResponse, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.output is None:
        self.output = kortex_driver.msg.Empty()
    else:
      self.output = kortex_driver.msg.Empty()

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
      pass
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
      if self.output is None:
        self.output = kortex_driver.msg.Empty()
      end = 0
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
      pass
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
      if self.output is None:
        self.output = kortex_driver.msg.Empty()
      end = 0
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
class DoSensorFocusAction(object):
  _type          = 'kortex_driver/DoSensorFocusAction'
  _md5sum = 'a727a16574ce67bd4de8fba7da857f9f'
  _request_class  = DoSensorFocusActionRequest
  _response_class = DoSensorFocusActionResponse
