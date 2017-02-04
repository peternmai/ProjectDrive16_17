// Auto-generated. Do not edit!

// (in-package rosserial_msgs.msg)


"use strict";

let _serializer = require('../base_serialize.js');
let _deserializer = require('../base_deserialize.js');
let _finder = require('../find.js');

//-----------------------------------------------------------

class Log {
  constructor() {
    this.level = 0;
    this.msg = '';
  }

  static serialize(obj, bufferInfo) {
    // Serializes a message object of type Log
    // Serialize message field [level]
    bufferInfo = _serializer.uint8(obj.level, bufferInfo);
    // Serialize message field [msg]
    bufferInfo = _serializer.string(obj.msg, bufferInfo);
    return bufferInfo;
  }

  static deserialize(buffer) {
    //deserializes a message object of type Log
    let tmp;
    let len;
    let data = new Log();
    // Deserialize message field [level]
    tmp = _deserializer.uint8(buffer);
    data.level = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [msg]
    tmp = _deserializer.string(buffer);
    data.msg = tmp.data;
    buffer = tmp.buffer;
    return {
      data: data,
      buffer: buffer
    }
  }

  static datatype() {
    // Returns string type for a message object
    return 'rosserial_msgs/Log';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '11abd731c25933261cd6183bd12d6295';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    #ROS Logging Levels
    uint8 ROSDEBUG=0
    uint8 INFO=1
    uint8 WARN=2
    uint8 ERROR=3
    uint8 FATAL=4
    
    uint8 level
    string msg
    
    `;
  }

};

// Constants for message
Log.Constants = {
  ROSDEBUG: 0,
  INFO: 1,
  WARN: 2,
  ERROR: 3,
  FATAL: 4,
}

module.exports = Log;
