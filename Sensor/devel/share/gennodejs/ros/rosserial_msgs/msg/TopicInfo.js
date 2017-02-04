// Auto-generated. Do not edit!

// (in-package rosserial_msgs.msg)


"use strict";

let _serializer = require('../base_serialize.js');
let _deserializer = require('../base_deserialize.js');
let _finder = require('../find.js');

//-----------------------------------------------------------

class TopicInfo {
  constructor() {
    this.topic_id = 0;
    this.topic_name = '';
    this.message_type = '';
    this.md5sum = '';
    this.buffer_size = 0;
  }

  static serialize(obj, bufferInfo) {
    // Serializes a message object of type TopicInfo
    // Serialize message field [topic_id]
    bufferInfo = _serializer.uint16(obj.topic_id, bufferInfo);
    // Serialize message field [topic_name]
    bufferInfo = _serializer.string(obj.topic_name, bufferInfo);
    // Serialize message field [message_type]
    bufferInfo = _serializer.string(obj.message_type, bufferInfo);
    // Serialize message field [md5sum]
    bufferInfo = _serializer.string(obj.md5sum, bufferInfo);
    // Serialize message field [buffer_size]
    bufferInfo = _serializer.int32(obj.buffer_size, bufferInfo);
    return bufferInfo;
  }

  static deserialize(buffer) {
    //deserializes a message object of type TopicInfo
    let tmp;
    let len;
    let data = new TopicInfo();
    // Deserialize message field [topic_id]
    tmp = _deserializer.uint16(buffer);
    data.topic_id = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [topic_name]
    tmp = _deserializer.string(buffer);
    data.topic_name = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [message_type]
    tmp = _deserializer.string(buffer);
    data.message_type = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [md5sum]
    tmp = _deserializer.string(buffer);
    data.md5sum = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [buffer_size]
    tmp = _deserializer.int32(buffer);
    data.buffer_size = tmp.data;
    buffer = tmp.buffer;
    return {
      data: data,
      buffer: buffer
    }
  }

  static datatype() {
    // Returns string type for a message object
    return 'rosserial_msgs/TopicInfo';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0ad51f88fc44892f8c10684077646005';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # special topic_ids
    uint16 ID_PUBLISHER=0
    uint16 ID_SUBSCRIBER=1
    uint16 ID_SERVICE_SERVER=2
    uint16 ID_SERVICE_CLIENT=4
    uint16 ID_PARAMETER_REQUEST=6
    uint16 ID_LOG=7
    uint16 ID_TIME=10
    uint16 ID_TX_STOP=11
    
    # The endpoint ID for this topic
    uint16 topic_id
    
    string topic_name
    string message_type
    
    # MD5 checksum for this message type
    string md5sum
    
    # size of the buffer message must fit in
    int32 buffer_size
    
    `;
  }

};

// Constants for message
TopicInfo.Constants = {
  ID_PUBLISHER: 0,
  ID_SUBSCRIBER: 1,
  ID_SERVICE_SERVER: 2,
  ID_SERVICE_CLIENT: 4,
  ID_PARAMETER_REQUEST: 6,
  ID_LOG: 7,
  ID_TIME: 10,
  ID_TX_STOP: 11,
}

module.exports = TopicInfo;
