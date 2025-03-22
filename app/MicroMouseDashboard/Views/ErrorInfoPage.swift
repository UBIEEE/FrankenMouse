import SwiftUI

struct ErrorInfoPage: View {
  @EnvironmentObject var feedback: AppFeedback

  var body: some View {
    NavigationStack {
      List {
        Section("RSSI") {
          Text("\(feedback.rssi)")
            .foregroundStyle(
              feedback.rssi >= -70
                ? .green : feedback.rssi >= -85 ? .yellow : feedback.rssi >= -100 ? .orange : .red
            )
          Button("Read") {
            feedback.readRSSI()
          }
        }
        Section("Errors") {
          ForEach(feedback.mainService.errorCodes, id: \.self) { errorCode in
            Text("\(errorCode)")
          }
        }
      }
      .navigationTitle("Error Info")
    }
    .onAppear(
      perform: ({
        feedback.readRSSI()
      }))
  }
}

#Preview {
  ErrorInfoPage()
    .environmentObject(AppFeedback())
}
