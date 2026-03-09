import SpriteKit
import SwiftUI

let MAZE_WIDTH_CM = 16 * 18
let MAZE_WIDTH_MM = MAZE_WIDTH_CM * 10

struct DrivePage: View {
  @EnvironmentObject var feedback: AppFeedback

  var body: some View {
    NavigationStack {
      List {
        Section("Left Encoder") {
          VStack(alignment: .leading) {
            Text("\(feedback.driveService.motorLeftPosition)")
            Text("Distance (mm)")
              .font(.subheadline)
              .foregroundColor(.secondary)
          }
          VStack(alignment: .leading) {
            Text("\(feedback.driveService.motorLeftVelocity)")
            Text("Velocity (mm/s)")
              .font(.subheadline)
              .foregroundColor(.secondary)
          }
          VStack(alignment: .leading) {
            Text("\(feedback.driveService.motorLeftTicks)")
            Text("Ticks")
              .font(.subheadline)
              .foregroundColor(.secondary)
          }
        }
        Section("Right Encoder") {
          VStack(alignment: .leading) {
            Text("\(feedback.driveService.motorRightPosition)")
            Text("Distance (mm)")
              .font(.subheadline)
              .foregroundColor(.secondary)
          }
          VStack(alignment: .leading) {
            Text("\(feedback.driveService.motorRightVelocity)")
            Text("Velocity (mm/s)")
              .font(.subheadline)
              .foregroundColor(.secondary)
          }
          VStack(alignment: .leading) {
            Text("\(feedback.driveService.motorRightTicks)")
            Text("Ticks")
              .font(.subheadline)
              .foregroundColor(.secondary)
          }
        }
      }
      .navigationTitle("Drive")
    }
  }
}

#Preview {
  DrivePage()
    .environmentObject(AppFeedback())
}
