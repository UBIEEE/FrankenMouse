import Foundation

protocol AppFeedbackBase: NSObject {
  func publishMainTask(_ task: UInt8, _ startingPosition: UInt8)
  func publishAppReady()
  func publishDrivePID(_ values: [Float32])
  func publishVisionCalibrate()
  func publishVisionCalibrateReset()
  func publishMazeReset()
  func publishMusicSong(_ song: UInt8)
}
