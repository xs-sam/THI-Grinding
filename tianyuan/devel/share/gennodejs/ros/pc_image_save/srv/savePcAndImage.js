// Auto-generated. Do not edit!

// (in-package pc_image_save.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class savePcAndImageRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.fileBaseName = null;
      this.PointCloudFileType = null;
    }
    else {
      if (initObj.hasOwnProperty('fileBaseName')) {
        this.fileBaseName = initObj.fileBaseName
      }
      else {
        this.fileBaseName = '';
      }
      if (initObj.hasOwnProperty('PointCloudFileType')) {
        this.PointCloudFileType = initObj.PointCloudFileType
      }
      else {
        this.PointCloudFileType = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type savePcAndImageRequest
    // Serialize message field [fileBaseName]
    bufferOffset = _serializer.string(obj.fileBaseName, buffer, bufferOffset);
    // Serialize message field [PointCloudFileType]
    bufferOffset = _serializer.string(obj.PointCloudFileType, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type savePcAndImageRequest
    let len;
    let data = new savePcAndImageRequest(null);
    // Deserialize message field [fileBaseName]
    data.fileBaseName = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [PointCloudFileType]
    data.PointCloudFileType = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.fileBaseName);
    length += _getByteLength(object.PointCloudFileType);
    return length + 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'pc_image_save/savePcAndImageRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '30e1c552ee03f9b74de36d8d8acae442';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string fileBaseName
    string PointCloudFileType
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new savePcAndImageRequest(null);
    if (msg.fileBaseName !== undefined) {
      resolved.fileBaseName = msg.fileBaseName;
    }
    else {
      resolved.fileBaseName = ''
    }

    if (msg.PointCloudFileType !== undefined) {
      resolved.PointCloudFileType = msg.PointCloudFileType;
    }
    else {
      resolved.PointCloudFileType = ''
    }

    return resolved;
    }
};

class savePcAndImageResponse {
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
    // Serializes a message object of type savePcAndImageResponse
    // Serialize message field [ok]
    bufferOffset = _serializer.bool(obj.ok, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type savePcAndImageResponse
    let len;
    let data = new savePcAndImageResponse(null);
    // Deserialize message field [ok]
    data.ok = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'pc_image_save/savePcAndImageResponse';
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
    const resolved = new savePcAndImageResponse(null);
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
  Request: savePcAndImageRequest,
  Response: savePcAndImageResponse,
  md5sum() { return 'dd3b3b413ecc248d44e5417f1e4d808b'; },
  datatype() { return 'pc_image_save/savePcAndImage'; }
};
