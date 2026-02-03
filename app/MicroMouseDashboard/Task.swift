enum Task: UInt8, CaseIterable, Identifiable {
  case none = 0
  
  // 1-10: Maze tasks
  
  case mazeSearch = 1
  case mazeSlowSolve = 2
  case mazeFastSolve = 3

  // 11-20: Test drive tasks.
  
  case testDriveStraight = 11
  case testDriveLeftTurn = 12
  case testDriveRightTurn = 13
  case testDriveTurn180 = 14
  case testGyro = 15
  case testDriveStraightVisionAlign = 16
  
  // 21-30: Manual control tasks.
  
  case manualChassisSpeeds = 21

  // 100+: Other

  case armed = 100
  case armedTriggering
  case armedTriggered

  case visionCalibrate
  
  case unknown = 255

  var id: Self { self }
}

let TaskNames: [Task: String] = [
  .none: "None",
  
  .mazeSearch: "Maze Search",
  .mazeSlowSolve: "Maze Slow Solve",
  .mazeFastSolve: "Maze Fast Solve",

  .testDriveStraight: "TEST - Drive Straight",
  .testDriveLeftTurn: "TEST - Left Turn",
  .testDriveRightTurn: "TEST - Right Turn",
  .testDriveTurn180: "TEST - Turn 180˚",
  .testGyro: "TEST - Gyro",
  .testDriveStraightVisionAlign: "TEST - Drive Straight Vision Align",
  
  .manualChassisSpeeds: "Manual Control - Chassis Speeds",

  .armed: "Armed",
  .armedTriggering: "Armed Triggering",
  .armedTriggered: "Armed Triggered",

  .visionCalibrate: "Vision Calibrating",
  
  .unknown: "Unknown",
]

let TaskDescriptions: [Task: String] = [
  .none: "None",
  
  .mazeSearch: "Search to the center of the maze, then back to the start",
  .mazeSlowSolve: "Solve the maze using the same control method as Search mode",
  .mazeFastSolve: "Solve the maze as fast as possible while using previous search data",

  .testDriveStraight: "Drive straight for 2 cell lengths",
  .testDriveLeftTurn: "Make a left turn",
  .testDriveRightTurn: "Make a right turn",
  .testDriveTurn180: "Turn 180 degrees in place",
  .testGyro: "Maintain a rotational velocity of 0 deg/s",
  .testDriveStraightVisionAlign: "Drive straight while staying centered between maze walls",
]
