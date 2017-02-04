// Auto-generated. Do not edit!

// (in-package rosserial_msgs.srv)


"use strict";

let _serializer = require('../base_serialize.js');
let _deserializer = require('../base_deserialize.js');
let _finder = require('../find.js');

//-----------------------------------------------------------


//-----------------------------------------------------------

class RequestParamRequest {
  constructor() {
    this.name = '';
  }

  static serialize(obj, bufferInfo) {
    // Serializes a message object of type RequestParamRequest
    // Serialize message field [name]
    bufferInfo = _serializer.string(obj.name, bufferInfo);
    return bufferInfo;
  }

  static deserialize(buffer) {
    //deserializes a message object of type RequestParamRequest
    let tmp;
    let len;
    let data = new RequestParamRequest();
    // Deserialize message field [name]
    tmp = _deserializer.string(buffer);
    data.name = tmp.data;
    buffer = tmp.buffer;
    return {
      data: data,
      buffer: buffer
    }
  }

  static datatype() {
    // Returns string type for a service object
    return 'rosserial_msgs/RequestParamRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c1f3d28f1b044c871e6eff2e9fc3c667';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string name
    
    
    `;
  }

};

class RequestParamResponse {
  constructor() {
    this.ints = [];
    this.floats = [];
    this.strings = [];
  }

  static serialize(obj, bufferInfo) {
    // Serializes a message object of type RequestParamResponse
    // Serialize the length for message field [ints]
    bufferInfo = _serializer.uint32(obj.ints.length, bufferInfo);
    // Serialize message field [ints]
    obj.ints.forEach((val) => {
      bufferInfo = _serializer.int32(val, bufferInfo);
    });
    // Serialize the length for message field [floats]
    bufferInfo = _serializer.uint32(obj.floats.length, bufferInfo);
    // Serialize message field [floats]
    obj.floats.forEach((val) => {
      bufferInfo = _serializer.float32(val, bufferInfo);
    });
    // Serialize the length for message field [strings]
    bufferInfo = _serializer.uint32(obj.strings.length, bufferInfo);
    // Serialize message field [strings]
    obj.strings.forEach((val) => {
      bufferInfo = _serializer.string(val, bufferInfo);
    });
    return bufferInfo;
  }

  static deserialize(buffer) {
    //deserializes a message object of type RequestParamResponse
    let tmp;
    let len;
    let data = new RequestParamResponse();
    // Deserialize array length for message field [ints]
    tmp = _deserializer.uint32(buffer);
    len = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [ints]
    data.ints = new Array(len);
    for (let i = 0; i < len; ++i) {
      tmp = _deserializer.int32(buffer);
      data.ints[i] = tmp.data;
      buffer = tmp.buffer;
    }
    // Deserialize array length for message field [floats]
    tmp = _deserializer.uint32(buffer);
    len = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [floats]
    data.floats = new Array(len);
    for (let i = 0; i < len; ++i) {
      tmp = _deserializer.float32(buffer);
      data.floats[i] = tmp.data;
      buffer = tmp.buffer;
    }
    // Deserialize array length for message field [strings]
    tmp = _deserializer.uint32(buffer);
    len = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [strings]
    data.strings = new Array(len);
    for (let i = 0; i < len; ++i) {
      tmp = _deserializer.string(buffer);
      data.strings[i] = tmp.data;
      buffer = tmp.buffer;
    }
    return {
      data: data,
      buffer: buffer
    }
  }

  static datatype() {
    // Returns string type for a service object
    return 'rosserial_msgs/RequestParamResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9f0e98bda65981986ddf53afa7a40e49';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    int32[]   ints
    float32[] floats
    string[]  strings
    
    
    `;
  }

};

module.exports = {
  Request: RequestParamRequest,
  Response: RequestParamResponse
};
