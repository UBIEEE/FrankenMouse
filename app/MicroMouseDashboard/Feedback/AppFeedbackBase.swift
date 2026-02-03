import Foundation

protocol AppFeedbackBase: NSObject {
  func publishMainTask(_ task: Task, _ startingPosition: StartingPosition)
  func publishMainCommand(_ command: Command)
  func publishMainSong(_ song: Song)
  func publishDrivePID(_ values: [Float32])
  func publishDriveChassisSpeeds(_ linear: Float32, _ angular: Float32)
}
