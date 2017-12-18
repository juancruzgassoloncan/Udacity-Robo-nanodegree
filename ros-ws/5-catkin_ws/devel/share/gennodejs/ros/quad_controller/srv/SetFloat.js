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

class SetFloatRequest {
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
        this.data = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetFloatRequest
    // Serialize message field [data]
    bufferOffset = _serializer.float32(obj.data, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetFloatRequest
    let len;
    let data = new SetFloatRequest(null);
    // Deserialize message field [data]
    data.data = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'quad_controller/SetFloatRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '73fcbf46b49191e672908e50842a83d4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 data
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetFloatRequest(null);
    if (msg.data !== undefined) {
      resolved.data = msg.data;
    }
    else {
      resolved.data = 0.0
    }

    return resolved;
    }
};

class SetFloatResponse {
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
        this.newData = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetFloatResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [newData]
    bufferOffset = _serializer.float32(obj.newData, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetFloatResponse
    let len;
    let data = new SetFloatResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [newData]
    data.newData = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'quad_controller/SetFloatResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '93025a54ac8a01801326d7285fa0b92c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success
    float32 newData
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetFloatResponse(null);
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
      resolved.newData = 0.0
    }

    return resolved;
    }
};

module.exports = {
  Request: SetFloatRequest,
  Response: SetFloatResponse,
  md5sum() { return '33440febf8aeff64c8c846264db06c0f'; },
  datatype() { return 'quad_controller/SetFloat'; }
};
