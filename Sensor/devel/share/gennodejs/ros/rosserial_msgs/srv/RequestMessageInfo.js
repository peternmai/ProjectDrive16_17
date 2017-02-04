// Auto-generated. Do not edit!

// (in-package rosserial_msgs.srv)


"use strict";

let _serializer = require('../base_serialize.js');
let _deserializer = require('../base_deserialize.js');
let _finder = require('../find.js');

//-----------------------------------------------------------


//-----------------------------------------------------------

class RequestMessageInfoRequest {
  constructor() {
    this.type = '';
  }

  static serialize(obj, bufferInfo) {
    // Serializes a message object of type RequestMessageInfoRequest
    // Serialize message field [type]
    bufferInfo = _serializer.string(obj.type, bufferInfo);
    return bufferInfo;
  }

  static deserialize(buffer) {
    //deserializes a message object of type RequestMessageInfoRequest
    let tmp;
    let len;
    let data = new RequestMessageInfoRequest();
    // Deserialize message field [type]
    tmp = _deserializer.string(buffer);
    data.type = tmp.data;
    buffer = tmp.buffer;
    return {
      data: data,
      buffer: buffer
    }
  }

  static datatype() {
    // Returns string type for a service object
    return 'rosserial_msgs/RequestMessageInfoRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'dc67331de85cf97091b7d45e5c64ab75';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    string type
    
    `;
  }

};

class RequestMessageInfoResponse {
  constructor() {
    this.md5 = '';
    this.definition = '';
  }

  static serialize(obj, bufferInfo) {
    // Serializes a message object of type RequestMessageInfoResponse
    // Serialize message field [md5]
    bufferInfo = _serializer.string(obj.md5, bufferInfo);
    // Serialize message field [definition]
    bufferInfo = _serializer.string(obj.definition, bufferInfo);
    return bufferInfo;
  }

  static deserialize(buffer) {
    //deserializes a message object of type RequestMessageInfoResponse
    let tmp;
    let len;
    let data = new RequestMessageInfoResponse();
    // Deserialize message field [md5]
    tmp = _deserializer.string(buffer);
    data.md5 = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [definition]
    tmp = _deserializer.string(buffer);
    data.definition = tmp.data;
    buffer = tmp.buffer;
    return {
      data: data,
      buffer: buffer
    }
  }

  static datatype() {
    // Returns string type for a service object
    return 'rosserial_msgs/RequestMessageInfoResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'fe452186a069bed40f09b8628fe5eac8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    
    string md5
    string definition
    
    
    `;
  }

};

module.exports = {
  Request: RequestMessageInfoRequest,
  Response: RequestMessageInfoResponse
};
