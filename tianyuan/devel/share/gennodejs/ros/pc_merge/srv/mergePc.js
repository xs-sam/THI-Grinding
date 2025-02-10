// Auto-generated. Do not edit!

// (in-package pc_merge.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class mergePcRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.workpiece = null;
      this.pcnum = null;
      this.saveName = null;
    }
    else {
      if (initObj.hasOwnProperty('workpiece')) {
        this.workpiece = initObj.workpiece
      }
      else {
        this.workpiece = 0;
      }
      if (initObj.hasOwnProperty('pcnum')) {
        this.pcnum = initObj.pcnum
      }
      else {
        this.pcnum = 0;
      }
      if (initObj.hasOwnProperty('saveName')) {
        this.saveName = initObj.saveName
      }
      else {
        this.saveName = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type mergePcRequest
    // Serialize message field [workpiece]
    bufferOffset = _serializer.int32(obj.workpiece, buffer, bufferOffset);
    // Serialize message field [pcnum]
    bufferOffset = _serializer.int32(obj.pcnum, buffer, bufferOffset);
    // Serialize message field [saveName]
    bufferOffset = _serializer.string(obj.saveName, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type mergePcRequest
    let len;
    let data = new mergePcRequest(null);
    // Deserialize message field [workpiece]
    data.workpiece = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [pcnum]
    data.pcnum = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [saveName]
    data.saveName = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.saveName);
    return length + 12;
  }

  static datatype() {
    // Returns string type for a service object
    return 'pc_merge/mergePcRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd8c18a316570ef41dee51e3304654d20';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 workpiece # 0是上半件 1是下右半件 2是下左半件
    int32 pcnum # 需要合成的点云数量 如果为-1 则采用参数文件中的量
    string saveName # 保存的文件名 为空则用参数文件中的值
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new mergePcRequest(null);
    if (msg.workpiece !== undefined) {
      resolved.workpiece = msg.workpiece;
    }
    else {
      resolved.workpiece = 0
    }

    if (msg.pcnum !== undefined) {
      resolved.pcnum = msg.pcnum;
    }
    else {
      resolved.pcnum = 0
    }

    if (msg.saveName !== undefined) {
      resolved.saveName = msg.saveName;
    }
    else {
      resolved.saveName = ''
    }

    return resolved;
    }
};

class mergePcResponse {
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
    // Serializes a message object of type mergePcResponse
    // Serialize message field [ok]
    bufferOffset = _serializer.bool(obj.ok, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type mergePcResponse
    let len;
    let data = new mergePcResponse(null);
    // Deserialize message field [ok]
    data.ok = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'pc_merge/mergePcResponse';
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
    const resolved = new mergePcResponse(null);
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
  Request: mergePcRequest,
  Response: mergePcResponse,
  md5sum() { return 'de99575f7cba9c0362917ec861ff5f7b'; },
  datatype() { return 'pc_merge/mergePc'; }
};
