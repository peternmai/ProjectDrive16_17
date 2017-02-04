// Auto-generated. Do not edit!

// (in-package rosserial_msgs.srv)


"use strict";

let _serializer = require('../base_serialize.js');
let _deserializer = require('../base_deserialize.js');
let _finder = require('../find.js');

//-----------------------------------------------------------


//-----------------------------------------------------------

class RequestServiceInfoRequest {
  constructor() {
    this.service = '';
  }

  static serialize(obj, bufferInfo) {
    // Serializes a message object of type RequestServiceInfoRequest
    // Serialize message field [service]
    bufferInfo = _serializer.string(obj.service, bufferInfo);
    return bufferInfo;
  }

  static deserialize(buffer) {
    //deserializes a message object of type RequestServiceInfoRequest
    let tmp;
    let len;
    let data = new RequestServiceInfoRequest();
    // Deserialize message field [service]
    tmp = _deserializer.string(buffer);
    data.service = tmp.data;
    buffer = tmp.buffer;
    return {
      data: data,
      buffer: buffer
    }
  }

  static datatype() {
    // Returns string type for a service object
    return 'rosserial_msgs/RequestServiceInfoRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1cbcfa13b08f6d36710b9af8741e6112';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    string service
    
    `;
  }

};

class RequestServiceInfoResponse {
  constructor() {
    this.service_md5 = '';
    this.request_md5 = '';
    this.response_md5 = '';
  }

  static serialize(obj, bufferInfo) {
    // Serializes a message object of type RequestServiceInfoResponse
    // Serialize message field [service_md5]
    bufferInfo = _serializer.string(obj.service_md5, bufferInfo);
    // Serialize message field [request_md5]
    bufferInfo = _serializer.string(obj.request_md5, bufferInfo);
    // Serialize message field [response_md5]
    bufferInfo = _serializer.string(obj.response_md5, bufferInfo);
    return bufferInfo;
  }

  static deserialize(buffer) {
    //deserializes a message object of type RequestServiceInfoResponse
    let tmp;
    let len;
    let data = new RequestServiceInfoResponse();
    // Deserialize message field [service_md5]
    tmp = _deserializer.string(buffer);
    data.service_md5 = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [request_md5]
    tmp = _deserializer.string(buffer);
    data.request_md5 = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [response_md5]
    tmp = _deserializer.string(buffer);
    data.response_md5 = tmp.data;
    buffer = tmp.buffer;
    return {
      data: data,
      buffer: buffer
    }
  }

  static datatype() {
    // Returns string type for a service object
    return 'rosserial_msgs/RequestServiceInfoResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c3d6dd25b909596479fbbc6559fa6874';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string service_md5
    string request_md5
    string response_md5
    
    
    `;
  }

};

module.exports = {
  Request: RequestServiceInfoRequest,
  Response: RequestServiceInfoResponse
};
