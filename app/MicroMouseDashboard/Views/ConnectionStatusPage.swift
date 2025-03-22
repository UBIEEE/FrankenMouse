import SwiftUI

struct ConnectionStatusPage: View {
  @EnvironmentObject var feedback: AppFeedback

  var body: some View {
    VStack {
      Text("Connection Status")
        .font(.title)

      VStack(alignment: .leading, spacing: 5) {
        Text("\(Utilities.boolToEmoji(feedback.connectionState.deviceFound)) Device found")
        Text("\(Utilities.boolToEmoji(feedback.connectionState.deviceConnected)) Device connected")
        Text(
          "\(Utilities.boolToEmoji(feedback.connectionState.mainServiceReady)) Main Service discovered"
        )
        Text(
          "\(Utilities.boolToEmoji(feedback.connectionState.musicServiceReady)) Music Service discovered"
        )
        Text(
          "\(Utilities.boolToEmoji(feedback.connectionState.visionServiceReady)) Vision Service discovered"
        )
        Text(
          "\(Utilities.boolToEmoji(feedback.connectionState.driveServiceReady)) Drive Service discovered"
        )
      }
      .padding()
    }
    .padding()
  }
}

#Preview {
  ConnectionStatusPage()
    .environmentObject(AppFeedback())
}
