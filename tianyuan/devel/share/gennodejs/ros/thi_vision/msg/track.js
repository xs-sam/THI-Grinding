// Auto-generated. Do not edit!

// (in-package thi_vision.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let pose = require('./pose.js');

//-----------------------------------------------------------

class track {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.track = null;
    }
    else {
      if (initObj.hasOwnProperty('track')) {
        this.track = initObj.track
      }
      else {
        this.track = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type track
    // Serialize message field [track]
    // Serialize the length for message field [track]
    bufferOffset = _serializer.uint32(obj.track.length, buffer, bufferOffset);
    obj.track.forEach((val) => {
      bufferOffset = pose.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type track
    let len;
    let data = new track(null);
    // Deserialize message field [track]
    // Deserialize array length for message field [track]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.track = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.track[i] = pose.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 24 * object.track.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'thi_vision/track';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7c10583178550886efa4f5156f56c1e6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new track(null);
    if (msg.track !== undefined) {
      resolved.track = new Array(msg.track.length);
      for (let i = 0; i < resolved.track.length; ++i) {
        resolved.track[i] = pose.Resolve(msg.track[i]);
      }
    }
    else {
      resolved.track = []
    }

    return resolved;
    }
};

module.exports = track;
