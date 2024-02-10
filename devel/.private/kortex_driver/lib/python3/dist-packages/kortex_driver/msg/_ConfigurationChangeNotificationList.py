# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from kortex_driver/ConfigurationChangeNotificationList.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import kortex_driver.msg

class ConfigurationChangeNotificationList(genpy.Message):
  _md5sum = "bf8ec7af96a2de99f1b33e2bf4146af1"
  _type = "kortex_driver/ConfigurationChangeNotificationList"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """
ConfigurationChangeNotification[] notifications
================================================================================
MSG: kortex_driver/ConfigurationChangeNotification

uint32 event
Timestamp timestamp
UserProfileHandle user_handle
Connection connection
ConfigurationChangeNotification_configuration_change oneof_configuration_change
================================================================================
MSG: kortex_driver/Timestamp

uint32 sec
uint32 usec
================================================================================
MSG: kortex_driver/UserProfileHandle

uint32 identifier
uint32 permission
================================================================================
MSG: kortex_driver/Connection

UserProfileHandle user_handle
string connection_information
uint32 connection_identifier
================================================================================
MSG: kortex_driver/ConfigurationChangeNotification_configuration_change

SequenceHandle[] sequence_handle
ActionHandle[] action_handle
MappingHandle[] mapping_handle
MapGroupHandle[] map_group_handle
MapHandle[] map_handle
UserProfileHandle[] user_profile_handle
ProtectionZoneHandle[] protection_zone_handle
SafetyHandle[] safety_handle
NetworkHandle[] network_handle
Ssid[] ssid
ControllerHandle[] controller_handle
================================================================================
MSG: kortex_driver/SequenceHandle

uint32 identifier
uint32 permission
================================================================================
MSG: kortex_driver/ActionHandle

uint32 identifier
uint32 action_type
uint32 permission
================================================================================
MSG: kortex_driver/MappingHandle

uint32 identifier
uint32 permission
================================================================================
MSG: kortex_driver/MapGroupHandle

uint32 identifier
uint32 permission
================================================================================
MSG: kortex_driver/MapHandle

uint32 identifier
uint32 permission
================================================================================
MSG: kortex_driver/ProtectionZoneHandle

uint32 identifier
uint32 permission
================================================================================
MSG: kortex_driver/SafetyHandle

uint32 identifier
================================================================================
MSG: kortex_driver/NetworkHandle

uint32 type
================================================================================
MSG: kortex_driver/Ssid

string identifier
================================================================================
MSG: kortex_driver/ControllerHandle

uint32 type
uint32 controller_identifier"""
  __slots__ = ['notifications']
  _slot_types = ['kortex_driver/ConfigurationChangeNotification[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       notifications

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(ConfigurationChangeNotificationList, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.notifications is None:
        self.notifications = []
    else:
      self.notifications = []

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
      length = len(self.notifications)
      buff.write(_struct_I.pack(length))
      for val1 in self.notifications:
        _x = val1.event
        buff.write(_get_struct_I().pack(_x))
        _v1 = val1.timestamp
        _x = _v1
        buff.write(_get_struct_2I().pack(_x.sec, _x.usec))
        _v2 = val1.user_handle
        _x = _v2
        buff.write(_get_struct_2I().pack(_x.identifier, _x.permission))
        _v3 = val1.connection
        _v4 = _v3.user_handle
        _x = _v4
        buff.write(_get_struct_2I().pack(_x.identifier, _x.permission))
        _x = _v3.connection_information
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        _x = _v3.connection_identifier
        buff.write(_get_struct_I().pack(_x))
        _v5 = val1.oneof_configuration_change
        length = len(_v5.sequence_handle)
        buff.write(_struct_I.pack(length))
        for val3 in _v5.sequence_handle:
          _x = val3
          buff.write(_get_struct_2I().pack(_x.identifier, _x.permission))
        length = len(_v5.action_handle)
        buff.write(_struct_I.pack(length))
        for val3 in _v5.action_handle:
          _x = val3
          buff.write(_get_struct_3I().pack(_x.identifier, _x.action_type, _x.permission))
        length = len(_v5.mapping_handle)
        buff.write(_struct_I.pack(length))
        for val3 in _v5.mapping_handle:
          _x = val3
          buff.write(_get_struct_2I().pack(_x.identifier, _x.permission))
        length = len(_v5.map_group_handle)
        buff.write(_struct_I.pack(length))
        for val3 in _v5.map_group_handle:
          _x = val3
          buff.write(_get_struct_2I().pack(_x.identifier, _x.permission))
        length = len(_v5.map_handle)
        buff.write(_struct_I.pack(length))
        for val3 in _v5.map_handle:
          _x = val3
          buff.write(_get_struct_2I().pack(_x.identifier, _x.permission))
        length = len(_v5.user_profile_handle)
        buff.write(_struct_I.pack(length))
        for val3 in _v5.user_profile_handle:
          _x = val3
          buff.write(_get_struct_2I().pack(_x.identifier, _x.permission))
        length = len(_v5.protection_zone_handle)
        buff.write(_struct_I.pack(length))
        for val3 in _v5.protection_zone_handle:
          _x = val3
          buff.write(_get_struct_2I().pack(_x.identifier, _x.permission))
        length = len(_v5.safety_handle)
        buff.write(_struct_I.pack(length))
        for val3 in _v5.safety_handle:
          _x = val3.identifier
          buff.write(_get_struct_I().pack(_x))
        length = len(_v5.network_handle)
        buff.write(_struct_I.pack(length))
        for val3 in _v5.network_handle:
          _x = val3.type
          buff.write(_get_struct_I().pack(_x))
        length = len(_v5.ssid)
        buff.write(_struct_I.pack(length))
        for val3 in _v5.ssid:
          _x = val3.identifier
          length = len(_x)
          if python3 or type(_x) == unicode:
            _x = _x.encode('utf-8')
            length = len(_x)
          buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        length = len(_v5.controller_handle)
        buff.write(_struct_I.pack(length))
        for val3 in _v5.controller_handle:
          _x = val3
          buff.write(_get_struct_2I().pack(_x.type, _x.controller_identifier))
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
      if self.notifications is None:
        self.notifications = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.notifications = []
      for i in range(0, length):
        val1 = kortex_driver.msg.ConfigurationChangeNotification()
        start = end
        end += 4
        (val1.event,) = _get_struct_I().unpack(str[start:end])
        _v6 = val1.timestamp
        _x = _v6
        start = end
        end += 8
        (_x.sec, _x.usec,) = _get_struct_2I().unpack(str[start:end])
        _v7 = val1.user_handle
        _x = _v7
        start = end
        end += 8
        (_x.identifier, _x.permission,) = _get_struct_2I().unpack(str[start:end])
        _v8 = val1.connection
        _v9 = _v8.user_handle
        _x = _v9
        start = end
        end += 8
        (_x.identifier, _x.permission,) = _get_struct_2I().unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v8.connection_information = str[start:end].decode('utf-8', 'rosmsg')
        else:
          _v8.connection_information = str[start:end]
        start = end
        end += 4
        (_v8.connection_identifier,) = _get_struct_I().unpack(str[start:end])
        _v10 = val1.oneof_configuration_change
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        _v10.sequence_handle = []
        for i in range(0, length):
          val3 = kortex_driver.msg.SequenceHandle()
          _x = val3
          start = end
          end += 8
          (_x.identifier, _x.permission,) = _get_struct_2I().unpack(str[start:end])
          _v10.sequence_handle.append(val3)
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        _v10.action_handle = []
        for i in range(0, length):
          val3 = kortex_driver.msg.ActionHandle()
          _x = val3
          start = end
          end += 12
          (_x.identifier, _x.action_type, _x.permission,) = _get_struct_3I().unpack(str[start:end])
          _v10.action_handle.append(val3)
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        _v10.mapping_handle = []
        for i in range(0, length):
          val3 = kortex_driver.msg.MappingHandle()
          _x = val3
          start = end
          end += 8
          (_x.identifier, _x.permission,) = _get_struct_2I().unpack(str[start:end])
          _v10.mapping_handle.append(val3)
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        _v10.map_group_handle = []
        for i in range(0, length):
          val3 = kortex_driver.msg.MapGroupHandle()
          _x = val3
          start = end
          end += 8
          (_x.identifier, _x.permission,) = _get_struct_2I().unpack(str[start:end])
          _v10.map_group_handle.append(val3)
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        _v10.map_handle = []
        for i in range(0, length):
          val3 = kortex_driver.msg.MapHandle()
          _x = val3
          start = end
          end += 8
          (_x.identifier, _x.permission,) = _get_struct_2I().unpack(str[start:end])
          _v10.map_handle.append(val3)
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        _v10.user_profile_handle = []
        for i in range(0, length):
          val3 = kortex_driver.msg.UserProfileHandle()
          _x = val3
          start = end
          end += 8
          (_x.identifier, _x.permission,) = _get_struct_2I().unpack(str[start:end])
          _v10.user_profile_handle.append(val3)
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        _v10.protection_zone_handle = []
        for i in range(0, length):
          val3 = kortex_driver.msg.ProtectionZoneHandle()
          _x = val3
          start = end
          end += 8
          (_x.identifier, _x.permission,) = _get_struct_2I().unpack(str[start:end])
          _v10.protection_zone_handle.append(val3)
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        _v10.safety_handle = []
        for i in range(0, length):
          val3 = kortex_driver.msg.SafetyHandle()
          start = end
          end += 4
          (val3.identifier,) = _get_struct_I().unpack(str[start:end])
          _v10.safety_handle.append(val3)
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        _v10.network_handle = []
        for i in range(0, length):
          val3 = kortex_driver.msg.NetworkHandle()
          start = end
          end += 4
          (val3.type,) = _get_struct_I().unpack(str[start:end])
          _v10.network_handle.append(val3)
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        _v10.ssid = []
        for i in range(0, length):
          val3 = kortex_driver.msg.Ssid()
          start = end
          end += 4
          (length,) = _struct_I.unpack(str[start:end])
          start = end
          end += length
          if python3:
            val3.identifier = str[start:end].decode('utf-8', 'rosmsg')
          else:
            val3.identifier = str[start:end]
          _v10.ssid.append(val3)
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        _v10.controller_handle = []
        for i in range(0, length):
          val3 = kortex_driver.msg.ControllerHandle()
          _x = val3
          start = end
          end += 8
          (_x.type, _x.controller_identifier,) = _get_struct_2I().unpack(str[start:end])
          _v10.controller_handle.append(val3)
        self.notifications.append(val1)
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
      length = len(self.notifications)
      buff.write(_struct_I.pack(length))
      for val1 in self.notifications:
        _x = val1.event
        buff.write(_get_struct_I().pack(_x))
        _v11 = val1.timestamp
        _x = _v11
        buff.write(_get_struct_2I().pack(_x.sec, _x.usec))
        _v12 = val1.user_handle
        _x = _v12
        buff.write(_get_struct_2I().pack(_x.identifier, _x.permission))
        _v13 = val1.connection
        _v14 = _v13.user_handle
        _x = _v14
        buff.write(_get_struct_2I().pack(_x.identifier, _x.permission))
        _x = _v13.connection_information
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        _x = _v13.connection_identifier
        buff.write(_get_struct_I().pack(_x))
        _v15 = val1.oneof_configuration_change
        length = len(_v15.sequence_handle)
        buff.write(_struct_I.pack(length))
        for val3 in _v15.sequence_handle:
          _x = val3
          buff.write(_get_struct_2I().pack(_x.identifier, _x.permission))
        length = len(_v15.action_handle)
        buff.write(_struct_I.pack(length))
        for val3 in _v15.action_handle:
          _x = val3
          buff.write(_get_struct_3I().pack(_x.identifier, _x.action_type, _x.permission))
        length = len(_v15.mapping_handle)
        buff.write(_struct_I.pack(length))
        for val3 in _v15.mapping_handle:
          _x = val3
          buff.write(_get_struct_2I().pack(_x.identifier, _x.permission))
        length = len(_v15.map_group_handle)
        buff.write(_struct_I.pack(length))
        for val3 in _v15.map_group_handle:
          _x = val3
          buff.write(_get_struct_2I().pack(_x.identifier, _x.permission))
        length = len(_v15.map_handle)
        buff.write(_struct_I.pack(length))
        for val3 in _v15.map_handle:
          _x = val3
          buff.write(_get_struct_2I().pack(_x.identifier, _x.permission))
        length = len(_v15.user_profile_handle)
        buff.write(_struct_I.pack(length))
        for val3 in _v15.user_profile_handle:
          _x = val3
          buff.write(_get_struct_2I().pack(_x.identifier, _x.permission))
        length = len(_v15.protection_zone_handle)
        buff.write(_struct_I.pack(length))
        for val3 in _v15.protection_zone_handle:
          _x = val3
          buff.write(_get_struct_2I().pack(_x.identifier, _x.permission))
        length = len(_v15.safety_handle)
        buff.write(_struct_I.pack(length))
        for val3 in _v15.safety_handle:
          _x = val3.identifier
          buff.write(_get_struct_I().pack(_x))
        length = len(_v15.network_handle)
        buff.write(_struct_I.pack(length))
        for val3 in _v15.network_handle:
          _x = val3.type
          buff.write(_get_struct_I().pack(_x))
        length = len(_v15.ssid)
        buff.write(_struct_I.pack(length))
        for val3 in _v15.ssid:
          _x = val3.identifier
          length = len(_x)
          if python3 or type(_x) == unicode:
            _x = _x.encode('utf-8')
            length = len(_x)
          buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        length = len(_v15.controller_handle)
        buff.write(_struct_I.pack(length))
        for val3 in _v15.controller_handle:
          _x = val3
          buff.write(_get_struct_2I().pack(_x.type, _x.controller_identifier))
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
      if self.notifications is None:
        self.notifications = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.notifications = []
      for i in range(0, length):
        val1 = kortex_driver.msg.ConfigurationChangeNotification()
        start = end
        end += 4
        (val1.event,) = _get_struct_I().unpack(str[start:end])
        _v16 = val1.timestamp
        _x = _v16
        start = end
        end += 8
        (_x.sec, _x.usec,) = _get_struct_2I().unpack(str[start:end])
        _v17 = val1.user_handle
        _x = _v17
        start = end
        end += 8
        (_x.identifier, _x.permission,) = _get_struct_2I().unpack(str[start:end])
        _v18 = val1.connection
        _v19 = _v18.user_handle
        _x = _v19
        start = end
        end += 8
        (_x.identifier, _x.permission,) = _get_struct_2I().unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v18.connection_information = str[start:end].decode('utf-8', 'rosmsg')
        else:
          _v18.connection_information = str[start:end]
        start = end
        end += 4
        (_v18.connection_identifier,) = _get_struct_I().unpack(str[start:end])
        _v20 = val1.oneof_configuration_change
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        _v20.sequence_handle = []
        for i in range(0, length):
          val3 = kortex_driver.msg.SequenceHandle()
          _x = val3
          start = end
          end += 8
          (_x.identifier, _x.permission,) = _get_struct_2I().unpack(str[start:end])
          _v20.sequence_handle.append(val3)
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        _v20.action_handle = []
        for i in range(0, length):
          val3 = kortex_driver.msg.ActionHandle()
          _x = val3
          start = end
          end += 12
          (_x.identifier, _x.action_type, _x.permission,) = _get_struct_3I().unpack(str[start:end])
          _v20.action_handle.append(val3)
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        _v20.mapping_handle = []
        for i in range(0, length):
          val3 = kortex_driver.msg.MappingHandle()
          _x = val3
          start = end
          end += 8
          (_x.identifier, _x.permission,) = _get_struct_2I().unpack(str[start:end])
          _v20.mapping_handle.append(val3)
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        _v20.map_group_handle = []
        for i in range(0, length):
          val3 = kortex_driver.msg.MapGroupHandle()
          _x = val3
          start = end
          end += 8
          (_x.identifier, _x.permission,) = _get_struct_2I().unpack(str[start:end])
          _v20.map_group_handle.append(val3)
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        _v20.map_handle = []
        for i in range(0, length):
          val3 = kortex_driver.msg.MapHandle()
          _x = val3
          start = end
          end += 8
          (_x.identifier, _x.permission,) = _get_struct_2I().unpack(str[start:end])
          _v20.map_handle.append(val3)
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        _v20.user_profile_handle = []
        for i in range(0, length):
          val3 = kortex_driver.msg.UserProfileHandle()
          _x = val3
          start = end
          end += 8
          (_x.identifier, _x.permission,) = _get_struct_2I().unpack(str[start:end])
          _v20.user_profile_handle.append(val3)
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        _v20.protection_zone_handle = []
        for i in range(0, length):
          val3 = kortex_driver.msg.ProtectionZoneHandle()
          _x = val3
          start = end
          end += 8
          (_x.identifier, _x.permission,) = _get_struct_2I().unpack(str[start:end])
          _v20.protection_zone_handle.append(val3)
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        _v20.safety_handle = []
        for i in range(0, length):
          val3 = kortex_driver.msg.SafetyHandle()
          start = end
          end += 4
          (val3.identifier,) = _get_struct_I().unpack(str[start:end])
          _v20.safety_handle.append(val3)
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        _v20.network_handle = []
        for i in range(0, length):
          val3 = kortex_driver.msg.NetworkHandle()
          start = end
          end += 4
          (val3.type,) = _get_struct_I().unpack(str[start:end])
          _v20.network_handle.append(val3)
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        _v20.ssid = []
        for i in range(0, length):
          val3 = kortex_driver.msg.Ssid()
          start = end
          end += 4
          (length,) = _struct_I.unpack(str[start:end])
          start = end
          end += length
          if python3:
            val3.identifier = str[start:end].decode('utf-8', 'rosmsg')
          else:
            val3.identifier = str[start:end]
          _v20.ssid.append(val3)
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        _v20.controller_handle = []
        for i in range(0, length):
          val3 = kortex_driver.msg.ControllerHandle()
          _x = val3
          start = end
          end += 8
          (_x.type, _x.controller_identifier,) = _get_struct_2I().unpack(str[start:end])
          _v20.controller_handle.append(val3)
        self.notifications.append(val1)
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
_struct_3I = None
def _get_struct_3I():
    global _struct_3I
    if _struct_3I is None:
        _struct_3I = struct.Struct("<3I")
    return _struct_3I