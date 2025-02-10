// Auto-generated. Do not edit!

// (in-package node_manager.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class nodeInfoRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.nodeType = null;
      this.packName = null;
      this.nodeName = null;
      this.nodeState = null;
    }
    else {
      if (initObj.hasOwnProperty('nodeType')) {
        this.nodeType = initObj.nodeType
      }
      else {
        this.nodeType = '';
      }
      if (initObj.hasOwnProperty('packName')) {
        this.packName = initObj.packName
      }
      else {
        this.packName = '';
      }
      if (initObj.hasOwnProperty('nodeName')) {
        this.nodeName = initObj.nodeName
      }
      else {
        this.nodeName = '';
      }
      if (initObj.hasOwnProperty('nodeState')) {
        this.nodeState = initObj.nodeState
      }
      else {
        this.nodeState = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type nodeInfoRequest
    // Serialize message field [nodeType]
    bufferOffset = _serializer.string(obj.nodeType, buffer, bufferOffset);
    // Serialize message field [packName]
    bufferOffset = _serializer.string(obj.packName, buffer, bufferOffset);
    // Serialize message field [nodeName]
    bufferOffset = _serializer.string(obj.nodeName, buffer, bufferOffset);
    // Serialize message field [nodeState]
    bufferOffset = _serializer.string(obj.nodeState, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type nodeInfoRequest
    let len;
    let data = new nodeInfoRequest(null);
    // Deserialize message field [nodeType]
    data.nodeType = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [packName]
    data.packName = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [nodeName]
    data.nodeName = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [nodeState]
    data.nodeState = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.nodeType);
    length += _getByteLength(object.packName);
    length += _getByteLength(object.nodeName);
    length += _getByteLength(object.nodeState);
    return length + 16;
  }

  static datatype() {
    // Returns string type for a service object
    return 'node_manager/nodeInfoRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ebc8bfed655bb2297e6ad26fdcc4dbe6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string nodeType # launch / node
    string packName # 包名
    string nodeName # .launch名字 / 节点名字
    string nodeState # start / stop
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new nodeInfoRequest(null);
    if (msg.nodeType !== undefined) {
      resolved.nodeType = msg.nodeType;
    }
    else {
      resolved.nodeType = ''
    }

    if (msg.packName !== undefined) {
      resolved.packName = msg.packName;
    }
    else {
      resolved.packName = ''
    }

    if (msg.nodeName !== undefined) {
      resolved.nodeName = msg.nodeName;
    }
    else {
      resolved.nodeName = ''
    }

    if (msg.nodeState !== undefined) {
      resolved.nodeState = msg.nodeState;
    }
    else {
      resolved.nodeState = ''
    }

    return resolved;
    }
};

class nodeInfoResponse {
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
    // Serializes a message object of type nodeInfoResponse
    // Serialize message field [ok]
    bufferOffset = _serializer.bool(obj.ok, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type nodeInfoResponse
    let len;
    let data = new nodeInfoResponse(null);
    // Deserialize message field [ok]
    data.ok = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'node_manager/nodeInfoResponse';
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
    const resolved = new nodeInfoResponse(null);
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
  Request: nodeInfoRequest,
  Response: nodeInfoResponse,
  md5sum() { return '7752f5a8c2441ad5def7b480e5f4374d'; },
  datatype() { return 'node_manager/nodeInfo'; }
};
