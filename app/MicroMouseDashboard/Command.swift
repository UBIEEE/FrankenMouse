enum Command: UInt8, CaseIterable, Identifiable {
  case resendAllFeedback = 0
  case resetMaze = 1
  case calibrateVision = 2
  case resetVisionCalibration = 3
  
  var id: Self { self }
}
