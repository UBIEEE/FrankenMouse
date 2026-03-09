import Foundation

enum FeedbackTopicWrite: UInt8, CaseIterable, Identifiable {
  case mainTask
  case mainCommand
  case mainSong
  
  case drivePID
  case driveChassisSpeeds

  var id: Self { self }
}

enum FeedbackTopicReceive: UInt8, CaseIterable, Identifiable {
  case mainTask
  case mainError
  case mainSong
  case mainStatus
  case mainBatteryVoltage

  case visionRawReadings
  case visionDistances

  case driveMotorData
  case driveIMUData
  case drivePID
  case driveChassisSpeeds

  case mazeCell
  case mazeCoordinates

  var id: Self { self }
}
