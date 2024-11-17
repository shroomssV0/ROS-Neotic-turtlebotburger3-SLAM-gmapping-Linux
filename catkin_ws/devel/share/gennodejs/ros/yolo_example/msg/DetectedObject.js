// Auto-generated. Do not edit!

// (in-package yolo_example.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let BoundingBox = require('./BoundingBox.js');

//-----------------------------------------------------------

class DetectedObject {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.label = null;
      this.confidence = null;
      this.box = null;
    }
    else {
      if (initObj.hasOwnProperty('label')) {
        this.label = initObj.label
      }
      else {
        this.label = '';
      }
      if (initObj.hasOwnProperty('confidence')) {
        this.confidence = initObj.confidence
      }
      else {
        this.confidence = 0.0;
      }
      if (initObj.hasOwnProperty('box')) {
        this.box = initObj.box
      }
      else {
        this.box = new BoundingBox();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DetectedObject
    // Serialize message field [label]
    bufferOffset = _serializer.string(obj.label, buffer, bufferOffset);
    // Serialize message field [confidence]
    bufferOffset = _serializer.float64(obj.confidence, buffer, bufferOffset);
    // Serialize message field [box]
    bufferOffset = BoundingBox.serialize(obj.box, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DetectedObject
    let len;
    let data = new DetectedObject(null);
    // Deserialize message field [label]
    data.label = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [confidence]
    data.confidence = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [box]
    data.box = BoundingBox.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.label);
    return length + 44;
  }

  static datatype() {
    // Returns string type for a message object
    return 'yolo_example/DetectedObject';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'edf2e0356664286b429d6f21ae05e759';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string label
    float64 confidence
    BoundingBox box
    ================================================================================
    MSG: yolo_example/BoundingBox
    float64 x1
    float64 y1
    float64 x2
    float64 y2
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DetectedObject(null);
    if (msg.label !== undefined) {
      resolved.label = msg.label;
    }
    else {
      resolved.label = ''
    }

    if (msg.confidence !== undefined) {
      resolved.confidence = msg.confidence;
    }
    else {
      resolved.confidence = 0.0
    }

    if (msg.box !== undefined) {
      resolved.box = BoundingBox.Resolve(msg.box)
    }
    else {
      resolved.box = new BoundingBox()
    }

    return resolved;
    }
};

module.exports = DetectedObject;
