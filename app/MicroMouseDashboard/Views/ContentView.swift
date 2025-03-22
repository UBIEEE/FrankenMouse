import SwiftUI

struct ContentView: View {
  @StateObject var feedback = AppFeedback()

  var body: some View {
    VStack {
      // Bluetooth is disabled in Settings.
      if !feedback.isEnabled {
        FeedbackDisabledPage()
      }
      // MicroMouse isn't found/connected/ready yet.
      else if !feedback.connectionState.isReady {
        ConnectionStatusPage()
          .environmentObject(feedback)
      }
      // App is ready to go!
      else {
        NavigationView()
          .environmentObject(feedback)
      }
    }
    .onDisappear {
      feedback.destroy()
    }
  }
}

#Preview {
  ContentView()
}
