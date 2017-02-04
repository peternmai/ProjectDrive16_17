// Auto-generated. Do not edit!

// (in-package rosserial_arduino.msg)


"use strict";

let _serializer = require('../base_serialize.js');
let _deserializer = require('../base_deserialize.js');
let _finder = require('../find.js');

//-----------------------------------------------------------

class Adc {
  constructor() {
    this.adc0 = 0;
    this.adc1 = 0;
    this.adc2 = 0;
    this.adc3 = 0;
    this.adc4 = 0;
    this.adc5 = 0;
  }

  static serialize(obj, bufferInfo) {
    // Serializes a message object of type Adc
    // Serialize message field [adc0]
    bufferInfo = _serializer.uint16(obj.adc0, bufferInfo);
    // Serialize message field [adc1]
    bufferInfo = _serializer.uint16(obj.adc1, bufferInfo);
    // Serialize message field [adc2]
    bufferInfo = _serializer.uint16(obj.adc2, bufferInfo);
    // Serialize message field [adc3]
    bufferInfo = _serializer.uint16(obj.adc3, bufferInfo);
    // Serialize message field [adc4]
    bufferInfo = _serializer.uint16(obj.adc4, bufferInfo);
    // Serialize message field [adc5]
    bufferInfo = _serializer.uint16(obj.adc5, bufferInfo);
    return bufferInfo;
  }

  static deserialize(buffer) {
    //deserializes a message object of type Adc
    let tmp;
    let len;
    let data = new Adc();
    // Deserialize message field [adc0]
    tmp = _deserializer.uint16(buffer);
    data.adc0 = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [adc1]
    tmp = _deserializer.uint16(buffer);
    data.adc1 = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [adc2]
    tmp = _deserializer.uint16(buffer);
    data.adc2 = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [adc3]
    tmp = _deserializer.uint16(buffer);
    data.adc3 = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [adc4]
    tmp = _deserializer.uint16(buffer);
    data.adc4 = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [adc5]
    tmp = _deserializer.uint16(buffer);
    data.adc5 = tmp.data;
    buffer = tmp.buffer;
    return {
      data: data,
      buffer: buffer
    }
  }

  static datatype() {
    // Returns string type for a message object
    return 'rosserial_arduino/Adc';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6d7853a614e2e821319068311f2af25b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint16 adc0
    uint16 adc1
    uint16 adc2
    uint16 adc3
    uint16 adc4
    uint16 adc5
    
    `;
  }

};

module.exports = Adc;
