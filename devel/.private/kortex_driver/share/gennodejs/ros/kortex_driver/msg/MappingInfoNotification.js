// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let MapHandle = require('./MapHandle.js');
let Timestamp = require('./Timestamp.js');
let UserProfileHandle = require('./UserProfileHandle.js');
let Connection = require('./Connection.js');
let MappingHandle = require('./MappingHandle.js');

//-----------------------------------------------------------

class MappingInfoNotification {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.controller_identifier = null;
      this.active_map_handle = null;
      this.timestamp = null;
      this.user_handle = null;
      this.connection = null;
      this.mapping_handle = null;
    }
    else {
      if (initObj.hasOwnProperty('controller_identifier')) {
        this.controller_identifier = initObj.controller_identifier
      }
      else {
        this.controller_identifier = 0;
      }
      if (initObj.hasOwnProperty('active_map_handle')) {
        this.active_map_handle = initObj.active_map_handle
      }
      else {
        this.active_map_handle = new MapHandle();
      }
      if (initObj.hasOwnProperty('timestamp')) {
        this.timestamp = initObj.timestamp
      }
      else {
        this.timestamp = new Timestamp();
      }
      if (initObj.hasOwnProperty('user_handle')) {
        this.user_handle = initObj.user_handle
      }
      else {
        this.user_handle = new UserProfileHandle();
      }
      if (initObj.hasOwnProperty('connection')) {
        this.connection = initObj.connection
      }
      else {
        this.connection = new Connection();
      }
      if (initObj.hasOwnProperty('mapping_handle')) {
        this.mapping_handle = initObj.mapping_handle
      }
      else {
        this.mapping_handle = new MappingHandle();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MappingInfoNotification
    // Serialize message field [controller_identifier]
    bufferOffset = _serializer.uint32(obj.controller_identifier, buffer, bufferOffset);
    // Serialize message field [active_map_handle]
    bufferOffset = MapHandle.serialize(obj.active_map_handle, buffer, bufferOffset);
    // Serialize message field [timestamp]
    bufferOffset = Timestamp.serialize(obj.timestamp, buffer, bufferOffset);
    // Serialize message field [user_handle]
    bufferOffset = UserProfileHandle.serialize(obj.user_handle, buffer, bufferOffset);
    // Serialize message field [connection]
    bufferOffset = Connection.serialize(obj.connection, buffer, bufferOffset);
    // Serialize message field [mapping_handle]
    bufferOffset = MappingHandle.serialize(obj.mapping_handle, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MappingInfoNotification
    let len;
    let data = new MappingInfoNotification(null);
    // Deserialize message field [controller_identifier]
    data.controller_identifier = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [active_map_handle]
    data.active_map_handle = MapHandle.deserialize(buffer, bufferOffset);
    // Deserialize message field [timestamp]
    data.timestamp = Timestamp.deserialize(buffer, bufferOffset);
    // Deserialize message field [user_handle]
    data.user_handle = UserProfileHandle.deserialize(buffer, bufferOffset);
    // Deserialize message field [connection]
    data.connection = Connection.deserialize(buffer, bufferOffset);
    // Deserialize message field [mapping_handle]
    data.mapping_handle = MappingHandle.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += Connection.getMessageSize(object.connection);
    return length + 36;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/MappingInfoNotification';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c0d2df9a9b1143d371ae7438ebf20d09';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 controller_identifier
    MapHandle active_map_handle
    Timestamp timestamp
    UserProfileHandle user_handle
    Connection connection
    MappingHandle mapping_handle
    ================================================================================
    MSG: kortex_driver/MapHandle
    
    uint32 identifier
    uint32 permission
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
    MSG: kortex_driver/MappingHandle
    
    uint32 identifier
    uint32 permission
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MappingInfoNotification(null);
    if (msg.controller_identifier !== undefined) {
      resolved.controller_identifier = msg.controller_identifier;
    }
    else {
      resolved.controller_identifier = 0
    }

    if (msg.active_map_handle !== undefined) {
      resolved.active_map_handle = MapHandle.Resolve(msg.active_map_handle)
    }
    else {
      resolved.active_map_handle = new MapHandle()
    }

    if (msg.timestamp !== undefined) {
      resolved.timestamp = Timestamp.Resolve(msg.timestamp)
    }
    else {
      resolved.timestamp = new Timestamp()
    }

    if (msg.user_handle !== undefined) {
      resolved.user_handle = UserProfileHandle.Resolve(msg.user_handle)
    }
    else {
      resolved.user_handle = new UserProfileHandle()
    }

    if (msg.connection !== undefined) {
      resolved.connection = Connection.Resolve(msg.connection)
    }
    else {
      resolved.connection = new Connection()
    }

    if (msg.mapping_handle !== undefined) {
      resolved.mapping_handle = MappingHandle.Resolve(msg.mapping_handle)
    }
    else {
      resolved.mapping_handle = new MappingHandle()
    }

    return resolved;
    }
};

module.exports = MappingInfoNotification;