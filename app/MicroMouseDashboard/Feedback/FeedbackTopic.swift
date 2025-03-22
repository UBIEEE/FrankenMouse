import Foundation

enum FeedbackTopicWrite: UInt8, CaseIterable, Identifiable {
  case mainTask
  case mainAppReady

  case drivePIDData
  case visionCalibrate

  case mazeReset

  case musicPlaySong

  var id: Self { self }
}

enum FeedbackTopicReceive: UInt8, CaseIterable, Identifiable {
  case mainTask
  case mainError

  case driveMotorData
  case driveIMUData
  case drivePIDData

  case visionRawData
  case visionNormData
  case visionCalibrate

  case mazeCell
  case mousePosition

  case musicIsPlaying

  var id: Self { self }
}
