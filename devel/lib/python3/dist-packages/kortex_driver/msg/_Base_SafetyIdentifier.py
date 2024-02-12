# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from kortex_driver/Base_SafetyIdentifier.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class Base_SafetyIdentifier(genpy.Message):
  _md5sum = "1af24619c0f6e3d61a2eb98a96e27e96"
  _type = "kortex_driver/Base_SafetyIdentifier"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """
uint32 UNSPECIFIED_BASE_SAFETY_IDENTIFIER = 0

uint32 FIRMWARE_UPDATE_FAILURE = 1

uint32 EXTERNAL_COMMUNICATION_ERROR = 2

uint32 MAXIMUM_AMBIENT_TEMPERATURE = 4

uint32 MAXIMUM_CORE_TEMPERATURE = 8

uint32 JOINT_FAULT = 16

uint32 CYCLIC_DATA_JITTER = 32

uint32 REACHED_MAXIMUM_EVENT_LOGS = 64

uint32 NO_KINEMATICS_SUPPORT = 128

uint32 BRAKE_REMOVAL_FAILURE = 256

uint32 NETWORK_ERROR = 512

uint32 UNABLE_TO_REACH_POSE = 1024

uint32 JOINT_DETECTION_ERROR = 2048

uint32 NETWORK_INITIALIZATION_ERROR = 4096

uint32 MAXIMUM_CURRENT = 8192

uint32 MAXIMUM_VOLTAGE = 16384

uint32 MINIMUM_VOLTAGE = 32768

uint32 MAXIMUM_END_EFFECTOR_TRANSLATION_VELOCITY = 65536

uint32 MAXIMUM_END_EFFECTOR_ORIENTATION_VELOCITY = 131072

uint32 MAXIMUM_END_EFFECTOR_TRANSLATION_ACCELERATION = 262144

uint32 MAXIMUM_END_EFFECTOR_ORIENTATION_ACCELERATION = 524288

uint32 MAXIMUM_END_EFFECTOR_TRANSLATION_FORCE = 1048576

uint32 MAXIMUM_END_EFFECTOR_ORIENTATION_FORCE = 2097152

uint32 MAXIMUM_END_EFFECTOR_PAYLOAD = 4194304

uint32 EMERGENCY_STOP_ACTIVATED = 8388608

uint32 EMERGENCY_LINE_ACTIVATED = 16777216

uint32 INRUSH_CURRENT_LIMITER_FAULT = 33554432

uint32 NVRAM_CORRUPTED = 67108864

uint32 INCOMPATIBLE_FIRMWARE_VERSION = 134217728

uint32 POWERON_SELF_TEST_FAILURE = 268435456

uint32 DISCRETE_INPUT_STUCK_ACTIVE = 536870912

uint32 ARM_INTO_ILLEGAL_POSITION = 1073741824
"""
  # Pseudo-constants
  UNSPECIFIED_BASE_SAFETY_IDENTIFIER = 0
  FIRMWARE_UPDATE_FAILURE = 1
  EXTERNAL_COMMUNICATION_ERROR = 2
  MAXIMUM_AMBIENT_TEMPERATURE = 4
  MAXIMUM_CORE_TEMPERATURE = 8
  JOINT_FAULT = 16
  CYCLIC_DATA_JITTER = 32
  REACHED_MAXIMUM_EVENT_LOGS = 64
  NO_KINEMATICS_SUPPORT = 128
  BRAKE_REMOVAL_FAILURE = 256
  NETWORK_ERROR = 512
  UNABLE_TO_REACH_POSE = 1024
  JOINT_DETECTION_ERROR = 2048
  NETWORK_INITIALIZATION_ERROR = 4096
  MAXIMUM_CURRENT = 8192
  MAXIMUM_VOLTAGE = 16384
  MINIMUM_VOLTAGE = 32768
  MAXIMUM_END_EFFECTOR_TRANSLATION_VELOCITY = 65536
  MAXIMUM_END_EFFECTOR_ORIENTATION_VELOCITY = 131072
  MAXIMUM_END_EFFECTOR_TRANSLATION_ACCELERATION = 262144
  MAXIMUM_END_EFFECTOR_ORIENTATION_ACCELERATION = 524288
  MAXIMUM_END_EFFECTOR_TRANSLATION_FORCE = 1048576
  MAXIMUM_END_EFFECTOR_ORIENTATION_FORCE = 2097152
  MAXIMUM_END_EFFECTOR_PAYLOAD = 4194304
  EMERGENCY_STOP_ACTIVATED = 8388608
  EMERGENCY_LINE_ACTIVATED = 16777216
  INRUSH_CURRENT_LIMITER_FAULT = 33554432
  NVRAM_CORRUPTED = 67108864
  INCOMPATIBLE_FIRMWARE_VERSION = 134217728
  POWERON_SELF_TEST_FAILURE = 268435456
  DISCRETE_INPUT_STUCK_ACTIVE = 536870912
  ARM_INTO_ILLEGAL_POSITION = 1073741824

  __slots__ = []
  _slot_types = []

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Base_SafetyIdentifier, self).__init__(*args, **kwds)

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
      end = 0
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I