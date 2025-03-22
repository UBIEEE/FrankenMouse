import SwiftUI

struct FeedbackDisabledPage: View {
  var body: some View {
    VStack(spacing: 5) {
      Text("🤯")
      Text("Failed to initialize Feedback!")
        .foregroundStyle(.red)
      Text("Maybe try enabling Bluetooth?")
    }
    .padding()
  }
}

#Preview {
  FeedbackDisabledPage()
}
