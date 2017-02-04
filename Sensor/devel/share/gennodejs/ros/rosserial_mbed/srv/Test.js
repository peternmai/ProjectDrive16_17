// Auto-generated. Do not edit!

// (in-package rosserial_mbed.srv)


"use strict";

let _serializer = require('../base_serialize.js');
let _deserializer = require('../base_deserialize.js');
let _finder = require('../find.js');

//-----------------------------------------------------------


//-----------------------------------------------------------

class TestRequest {
  constructor() {
    this.input = '';
  }

  static serialize(obj, bufferInfo) {
    // Serializes a message object of type TestRequest
    // Serialize message field [input]
    bufferInfo = _serializer.string(obj.input, bufferInfo);
    return bufferInfo;
  }

  static deserialize(buffer) {
    //deserializes a message object of type TestRequest
    let tmp;
    let len;
    let data = new TestRequest();
    // Deserialize message field [input]
    tmp = _deserializer.string(buffer);
    data.input = tmp.data;
    buffer = tmp.buffer;
    return {
      data: data,
      buffer: buffer
    }
  }

  static datatype() {
    // Returns string type for a service object
    return 'rosserial_mbed/TestRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '39e92f1778057359c64c7b8a7d7b19de';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string input
    
    `;
  }

};

class TestResponse {
  constructor() {
    this.output = '';
  }

  static serialize(obj, bufferInfo) {
    // Serializes a message object of type TestResponse
    // Serialize message field [output]
    bufferInfo = _serializer.string(obj.output, bufferInfo);
    return bufferInfo;
  }

  static deserialize(buffer) {
    //deserializes a message object of type TestResponse
    let tmp;
    let len;
    let data = new TestResponse();
    // Deserialize message field [output]
    tmp = _deserializer.string(buffer);
    data.output = tmp.data;
    buffer = tmp.buffer;
    return {
      data: data,
      buffer: buffer
    }
  }

  static datatype() {
    // Returns string type for a service object
    return 'rosserial_mbed/TestResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0825d95fdfa2c8f4bbb4e9c74bccd3fd';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string output
    
    
    `;
  }

};

module.exports = {
  Request: TestRequest,
  Response: TestResponse
};
