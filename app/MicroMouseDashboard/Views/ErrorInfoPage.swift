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
          ForEach(Array(feedback.mainService.errorCodes.keys), id: \.self) { timestamp in
            let error = feedback.mainService.errorCodes[timestamp]!
            Text("\(timestamp): \(error.errorCategory)-\(error.errorCode)")
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
