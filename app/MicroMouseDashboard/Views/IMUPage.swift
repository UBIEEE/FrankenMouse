import SwiftUI

struct IMUPage: View {
  @EnvironmentObject var feedback: AppFeedback

  var body: some View {
    NavigationStack {
      List {
        let imuData = feedback.driveService.imuData
        Section("Angular Velocity (Deg/s)") {
          Text("X\t").foregroundColor(.secondary) + Text(String(format: "%.2f", imuData[0]))

          Text("Y\t").foregroundColor(.secondary) + Text(String(format: "%.2f", imuData[1]))

          Text("Z\t").foregroundColor(.secondary) + Text(String(format: "%.2f", imuData[2]))
        }
        Section("Linear Acceleration (Gravities)") {
          Text("X\t").foregroundColor(.secondary) + Text(String(format: "%.2f", imuData[3]))

          Text("Y\t").foregroundColor(.secondary) + Text(String(format: "%.2f", imuData[4]))

          Text("Z\t").foregroundColor(.secondary) + Text(String(format: "%.2f", imuData[5]))
        }
      }
      .navigationTitle("IMU")
    }
  }
}

#Preview {
  IMUPage()
    .environmentObject(AppFeedback())
}
