enum Task: UInt8, CaseIterable, Identifiable {
  case none = 0
  
  // 1-10: Maze tasks
  
  case mazeSearch = 1
  case mazeSlowSolve = 2
  case mazeFastSolve = 3

  // 11-40: Test drive tasks.
  
  case testDriveStraightFromBackWallToSenseSpot = 11
  case testDriveStraightOneCell = 12
  case testDriveTurnRightFromSenseSpotToSenseSpot = 13
  case testDriveTurnLeftFromSenseSpotToSenseSpot = 14
  case testDriveTurnRightInPlace = 15
  case testDriveTurnLeftInPlace = 16
  case testDriveTurn180InPlace = 17
  case testGyro = 18
  case testDriveStraightFourCellsFromBackWallWithVisionAlign = 19
  
  // 41-50: Manual control tasks.
  
  case manualChassisSpeeds = 41

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

  .testDriveStraightFromBackWallToSenseSpot: "TEST - Drive Straight From Back Wall To Sense Spot",
  .testDriveStraightOneCell: "TEST - Drive Straight One Cell",
  .testDriveTurnRightFromSenseSpotToSenseSpot: "TEST - Drive Turn Right From Sense Spot To Sense Spot",
  .testDriveTurnLeftFromSenseSpotToSenseSpot: "TEST - Drive Turn Left From Sense Spot To Sense Spot",
  .testDriveTurnRightInPlace: "TEST - Drive Turn Right In Place",
  .testDriveTurnLeftInPlace: "TEST - Drive Turn Left In Place",
  .testDriveTurn180InPlace: "TEST - Drive Turn 180 In Place",
  .testGyro: "TEST - Gyro",
  .testDriveStraightFourCellsFromBackWallWithVisionAlign: "TEST - Drive Straight Four Cells From Back Wall With Vision Align",
  
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

  .testDriveStraightFromBackWallToSenseSpot: "Drive straight from back wall to sense spot",
  .testDriveStraightOneCell: "Drive straight for one cell",
  .testDriveTurnRightFromSenseSpotToSenseSpot: "Make a right turn from sense spot to sense spot",
  .testDriveTurnLeftFromSenseSpotToSenseSpot: "Make a left turn from sense spot to sense spot",
  .testDriveTurnRightInPlace: "Turn right in place",
  .testDriveTurnLeftInPlace: "Turn left in place",
  .testDriveTurn180InPlace: "Turn 180 degrees in place",
  .testGyro: "Maintain a rotational velocity of 0 deg/s",
  .testDriveStraightFourCellsFromBackWallWithVisionAlign: "Drive straight four cells from back wall with vision align",
]
