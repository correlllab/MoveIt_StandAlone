
"use strict";

let Analog = require('./Analog.js');
let Digital = require('./Digital.js');
let IOStates = require('./IOStates.js');
let MasterboardDataMsg = require('./MasterboardDataMsg.js');
let RobotModeDataMsg = require('./RobotModeDataMsg.js');
let RobotStateRTMsg = require('./RobotStateRTMsg.js');
let ToolDataMsg = require('./ToolDataMsg.js');

module.exports = {
  Analog: Analog,
  Digital: Digital,
  IOStates: IOStates,
  MasterboardDataMsg: MasterboardDataMsg,
  RobotModeDataMsg: RobotModeDataMsg,
  RobotStateRTMsg: RobotStateRTMsg,
  ToolDataMsg: ToolDataMsg,
};
