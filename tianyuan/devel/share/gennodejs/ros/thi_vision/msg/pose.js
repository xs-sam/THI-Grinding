// Auto-generated. Do not edit!

// (in-package thi_vision.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let position = require('./position.js');
let orientation = require('./orientation.js');

//-----------------------------------------------------------

class pose {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.point = null;
      this.angle = null;
    }
    else {
      if (initObj.hasOwnProperty('point')) {
        this.point = initObj.point
      }
      else {
        this.point = new position();
      }
      if (initObj.hasOwnProperty('angle')) {
        this.angle = initObj.angle
      }
      else {
        this.angle = new orientation();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type pose
    // Serialize message field [point]
    bufferOffset = position.serialize(obj.point, buffer, bufferOffset);
    // Serialize message field [angle]
    bufferOffset = orientation.serialize(obj.angle, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type pose
    let len;
    let data = new pose(null);
    // Deserialize message field [point]
    data.point = position.deserialize(buffer, bufferOffset);
    // Deserialize message field [angle]
    data.angle = orientation.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'thi_vision/pose';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a5c3a4fd40f2693c968403414ea57b4f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new pose(null);
    if (msg.point !== undefined) {
      resolved.point = position.Resolve(msg.point)
    }
    else {
      resolved.point = new position()
    }

    if (msg.angle !== undefined) {
      resolved.angle = orientation.Resolve(msg.angle)
    }
    else {
      resolved.angle = new orientation()
    }

    return resolved;
    }
};

module.exports = pose;
