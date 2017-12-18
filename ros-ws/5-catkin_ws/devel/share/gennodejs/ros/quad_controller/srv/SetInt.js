// Auto-generated. Do not edit!

// (in-package quad_controller.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class SetIntRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.data = null;
    }
    else {
      if (initObj.hasOwnProperty('data')) {
        this.data = initObj.data
      }
      else {
        this.data = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetIntRequest
    // Serialize message field [data]
    bufferOffset = _serializer.int32(obj.data, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetIntRequest
    let len;
    let data = new SetIntRequest(null);
    // Deserialize message field [data]
    data.data = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'quad_controller/SetIntRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'da5909fbe378aeaf85e547e830cc1bb7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 data
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetIntRequest(null);
    if (msg.data !== undefined) {
      resolved.data = msg.data;
    }
    else {
      resolved.data = 0
    }

    return resolved;
    }
};

class SetIntResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
      this.newData = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
      if (initObj.hasOwnProperty('newData')) {
        this.newData = initObj.newData
      }
      else {
        this.newData = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetIntResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [newData]
    bufferOffset = _serializer.int32(obj.newData, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetIntResponse
    let len;
    let data = new SetIntResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [newData]
    data.newData = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'quad_controller/SetIntResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2a5eb360a06563913251f51da65a2f34';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success
    int32 newData
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetIntResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    if (msg.newData !== undefined) {
      resolved.newData = msg.newData;
    }
    else {
      resolved.newData = 0
    }

    return resolved;
    }
};

module.exports = {
  Request: SetIntRequest,
  Response: SetIntResponse,
  md5sum() { return 'c6cf86a08ff7309c5d9f200d8652f57f'; },
  datatype() { return 'quad_controller/SetInt'; }
};
