// Auto-generated. Do not edit!

// (in-package thi_vision.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let orientation = require('../msg/orientation.js');
let pcArea = require('../msg/pcArea.js');

//-----------------------------------------------------------

let track = require('../msg/track.js');

//-----------------------------------------------------------

class visionTracksRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.pointCloudFile = null;
      this.fileType = null;
      this.workpiece = null;
      this.press = null;
      this.width = null;
      this.grooveWidth = null;
      this.avoidLeft = null;
      this.avoidRight = null;
      this.length = null;
      this.angleLeft = null;
      this.angleRight = null;
      this.listPcAreas = null;
    }
    else {
      if (initObj.hasOwnProperty('pointCloudFile')) {
        this.pointCloudFile = initObj.pointCloudFile
      }
      else {
        this.pointCloudFile = '';
      }
      if (initObj.hasOwnProperty('fileType')) {
        this.fileType = initObj.fileType
      }
      else {
        this.fileType = '';
      }
      if (initObj.hasOwnProperty('workpiece')) {
        this.workpiece = initObj.workpiece
      }
      else {
        this.workpiece = 0;
      }
      if (initObj.hasOwnProperty('press')) {
        this.press = initObj.press
      }
      else {
        this.press = 0.0;
      }
      if (initObj.hasOwnProperty('width')) {
        this.width = initObj.width
      }
      else {
        this.width = 0.0;
      }
      if (initObj.hasOwnProperty('grooveWidth')) {
        this.grooveWidth = initObj.grooveWidth
      }
      else {
        this.grooveWidth = 0.0;
      }
      if (initObj.hasOwnProperty('avoidLeft')) {
        this.avoidLeft = initObj.avoidLeft
      }
      else {
        this.avoidLeft = 0.0;
      }
      if (initObj.hasOwnProperty('avoidRight')) {
        this.avoidRight = initObj.avoidRight
      }
      else {
        this.avoidRight = 0.0;
      }
      if (initObj.hasOwnProperty('length')) {
        this.length = initObj.length
      }
      else {
        this.length = 0.0;
      }
      if (initObj.hasOwnProperty('angleLeft')) {
        this.angleLeft = initObj.angleLeft
      }
      else {
        this.angleLeft = new orientation();
      }
      if (initObj.hasOwnProperty('angleRight')) {
        this.angleRight = initObj.angleRight
      }
      else {
        this.angleRight = new orientation();
      }
      if (initObj.hasOwnProperty('listPcAreas')) {
        this.listPcAreas = initObj.listPcAreas
      }
      else {
        this.listPcAreas = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type visionTracksRequest
    // Serialize message field [pointCloudFile]
    bufferOffset = _serializer.string(obj.pointCloudFile, buffer, bufferOffset);
    // Serialize message field [fileType]
    bufferOffset = _serializer.string(obj.fileType, buffer, bufferOffset);
    // Serialize message field [workpiece]
    bufferOffset = _serializer.int32(obj.workpiece, buffer, bufferOffset);
    // Serialize message field [press]
    bufferOffset = _serializer.float32(obj.press, buffer, bufferOffset);
    // Serialize message field [width]
    bufferOffset = _serializer.float32(obj.width, buffer, bufferOffset);
    // Serialize message field [grooveWidth]
    bufferOffset = _serializer.float32(obj.grooveWidth, buffer, bufferOffset);
    // Serialize message field [avoidLeft]
    bufferOffset = _serializer.float32(obj.avoidLeft, buffer, bufferOffset);
    // Serialize message field [avoidRight]
    bufferOffset = _serializer.float32(obj.avoidRight, buffer, bufferOffset);
    // Serialize message field [length]
    bufferOffset = _serializer.float32(obj.length, buffer, bufferOffset);
    // Serialize message field [angleLeft]
    bufferOffset = orientation.serialize(obj.angleLeft, buffer, bufferOffset);
    // Serialize message field [angleRight]
    bufferOffset = orientation.serialize(obj.angleRight, buffer, bufferOffset);
    // Serialize message field [listPcAreas]
    // Serialize the length for message field [listPcAreas]
    bufferOffset = _serializer.uint32(obj.listPcAreas.length, buffer, bufferOffset);
    obj.listPcAreas.forEach((val) => {
      bufferOffset = pcArea.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type visionTracksRequest
    let len;
    let data = new visionTracksRequest(null);
    // Deserialize message field [pointCloudFile]
    data.pointCloudFile = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [fileType]
    data.fileType = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [workpiece]
    data.workpiece = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [press]
    data.press = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [width]
    data.width = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [grooveWidth]
    data.grooveWidth = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [avoidLeft]
    data.avoidLeft = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [avoidRight]
    data.avoidRight = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [length]
    data.length = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [angleLeft]
    data.angleLeft = orientation.deserialize(buffer, bufferOffset);
    // Deserialize message field [angleRight]
    data.angleRight = orientation.deserialize(buffer, bufferOffset);
    // Deserialize message field [listPcAreas]
    // Deserialize array length for message field [listPcAreas]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.listPcAreas = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.listPcAreas[i] = pcArea.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.pointCloudFile);
    length += _getByteLength(object.fileType);
    object.listPcAreas.forEach((val) => {
      length += pcArea.getMessageSize(val);
    });
    return length + 64;
  }

  static datatype() {
    // Returns string type for a service object
    return 'thi_vision/visionTracksRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7310b90945a93b256f7864c27e961494';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string pointCloudFile
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new visionTracksRequest(null);
    if (msg.pointCloudFile !== undefined) {
      resolved.pointCloudFile = msg.pointCloudFile;
    }
    else {
      resolved.pointCloudFile = ''
    }

    if (msg.fileType !== undefined) {
      resolved.fileType = msg.fileType;
    }
    else {
      resolved.fileType = ''
    }

    if (msg.workpiece !== undefined) {
      resolved.workpiece = msg.workpiece;
    }
    else {
      resolved.workpiece = 0
    }

    if (msg.press !== undefined) {
      resolved.press = msg.press;
    }
    else {
      resolved.press = 0.0
    }

    if (msg.width !== undefined) {
      resolved.width = msg.width;
    }
    else {
      resolved.width = 0.0
    }

    if (msg.grooveWidth !== undefined) {
      resolved.grooveWidth = msg.grooveWidth;
    }
    else {
      resolved.grooveWidth = 0.0
    }

    if (msg.avoidLeft !== undefined) {
      resolved.avoidLeft = msg.avoidLeft;
    }
    else {
      resolved.avoidLeft = 0.0
    }

    if (msg.avoidRight !== undefined) {
      resolved.avoidRight = msg.avoidRight;
    }
    else {
      resolved.avoidRight = 0.0
    }

    if (msg.length !== undefined) {
      resolved.length = msg.length;
    }
    else {
      resolved.length = 0.0
    }

    if (msg.angleLeft !== undefined) {
      resolved.angleLeft = orientation.Resolve(msg.angleLeft)
    }
    else {
      resolved.angleLeft = new orientation()
    }

    if (msg.angleRight !== undefined) {
      resolved.angleRight = orientation.Resolve(msg.angleRight)
    }
    else {
      resolved.angleRight = new orientation()
    }

    if (msg.listPcAreas !== undefined) {
      resolved.listPcAreas = new Array(msg.listPcAreas.length);
      for (let i = 0; i < resolved.listPcAreas.length; ++i) {
        resolved.listPcAreas[i] = pcArea.Resolve(msg.listPcAreas[i]);
      }
    }
    else {
      resolved.listPcAreas = []
    }

    return resolved;
    }
};

class visionTracksResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.tracks = null;
      this.ok = null;
    }
    else {
      if (initObj.hasOwnProperty('tracks')) {
        this.tracks = initObj.tracks
      }
      else {
        this.tracks = [];
      }
      if (initObj.hasOwnProperty('ok')) {
        this.ok = initObj.ok
      }
      else {
        this.ok = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type visionTracksResponse
    // Serialize message field [tracks]
    // Serialize the length for message field [tracks]
    bufferOffset = _serializer.uint32(obj.tracks.length, buffer, bufferOffset);
    obj.tracks.forEach((val) => {
      bufferOffset = track.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [ok]
    bufferOffset = _serializer.bool(obj.ok, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type visionTracksResponse
    let len;
    let data = new visionTracksResponse(null);
    // Deserialize message field [tracks]
    // Deserialize array length for message field [tracks]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.tracks = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.tracks[i] = track.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [ok]
    data.ok = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.tracks.forEach((val) => {
      length += track.getMessageSize(val);
    });
    return length + 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'thi_vision/visionTracksResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a33069b7a2e523cfca40ce4212d31d16';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    track[] tracks
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new visionTracksResponse(null);
    if (msg.tracks !== undefined) {
      resolved.tracks = new Array(msg.tracks.length);
      for (let i = 0; i < resolved.tracks.length; ++i) {
        resolved.tracks[i] = track.Resolve(msg.tracks[i]);
      }
    }
    else {
      resolved.tracks = []
    }

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
  Request: visionTracksRequest,
  Response: visionTracksResponse,
  md5sum() { return '54640a39f52ffd596022a5e1d0dc6ffe'; },
  datatype() { return 'thi_vision/visionTracks'; }
};
