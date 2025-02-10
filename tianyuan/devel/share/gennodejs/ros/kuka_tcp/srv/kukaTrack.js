// Auto-generated. Do not edit!

// (in-package kuka_tcp.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let kukaPoint = require('../msg/kukaPoint.js');

//-----------------------------------------------------------


//-----------------------------------------------------------

class kukaTrackRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.track = null;
      this.speed = null;
      this.mod = null;
    }
    else {
      if (initObj.hasOwnProperty('track')) {
        this.track = initObj.track
      }
      else {
        this.track = [];
      }
      if (initObj.hasOwnProperty('speed')) {
        this.speed = initObj.speed
      }
      else {
        this.speed = 0;
      }
      if (initObj.hasOwnProperty('mod')) {
        this.mod = initObj.mod
      }
      else {
        this.mod = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type kukaTrackRequest
    // Serialize message field [track]
    // Serialize the length for message field [track]
    bufferOffset = _serializer.uint32(obj.track.length, buffer, bufferOffset);
    obj.track.forEach((val) => {
      bufferOffset = kukaPoint.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [speed]
    bufferOffset = _serializer.int32(obj.speed, buffer, bufferOffset);
    // Serialize message field [mod]
    bufferOffset = _serializer.int32(obj.mod, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type kukaTrackRequest
    let len;
    let data = new kukaTrackRequest(null);
    // Deserialize message field [track]
    // Deserialize array length for message field [track]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.track = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.track[i] = kukaPoint.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [speed]
    data.speed = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [mod]
    data.mod = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 24 * object.track.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kuka_tcp/kukaTrackRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f55ac92146a46389f08a770a8fc980c0';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    kukaPoint[] track
    int32 speed # mm/s
    int32 mod # movel 1  movej 2
    
    ================================================================================
    MSG: kuka_tcp/kukaPoint
    # 机器人点位信息
    float32 x
    float32 y
    float32 z
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
    const resolved = new kukaTrackRequest(null);
    if (msg.track !== undefined) {
      resolved.track = new Array(msg.track.length);
      for (let i = 0; i < resolved.track.length; ++i) {
        resolved.track[i] = kukaPoint.Resolve(msg.track[i]);
      }
    }
    else {
      resolved.track = []
    }

    if (msg.speed !== undefined) {
      resolved.speed = msg.speed;
    }
    else {
      resolved.speed = 0
    }

    if (msg.mod !== undefined) {
      resolved.mod = msg.mod;
    }
    else {
      resolved.mod = 0
    }

    return resolved;
    }
};

class kukaTrackResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.ok = null;
    }
    else {
      if (initObj.hasOwnProperty('ok')) {
        this.ok = initObj.ok
      }
      else {
        this.ok = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type kukaTrackResponse
    // Serialize message field [ok]
    bufferOffset = _serializer.bool(obj.ok, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type kukaTrackResponse
    let len;
    let data = new kukaTrackResponse(null);
    // Deserialize message field [ok]
    data.ok = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kuka_tcp/kukaTrackResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6f6da3883749771fac40d6deb24a8c02';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool ok
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new kukaTrackResponse(null);
    if (msg.ok !== undefined) {
      resolved.ok = msg.ok;
    }
    else {
      resolved.ok = false
    }

    return resolved;
    }
};

module.exports = {
  Request: kukaTrackRequest,
  Response: kukaTrackResponse,
  md5sum() { return '542cc9503392943f62224d9b971f53d6'; },
  datatype() { return 'kuka_tcp/kukaTrack'; }
};
