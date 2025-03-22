import SwiftUI

struct VisionPage: View {
  @EnvironmentObject var feedback: AppFeedback

  var body: some View {
    NavigationStack {
      List {
        Section("Distances") {
          VStack(alignment: .leading) {
            Text("\(feedback.visionService.farLeftDistance)")
            Text("Front Wall (mm)\t\t(Left Edge Sensor)")
              .font(.subheadline)
              .foregroundColor(.secondary)
          }
          VStack(alignment: .leading) {
            Text("\(feedback.visionService.midLeftDistance)")
            Text("Left Wall (mm)\t\t(Left Middle Sensor)")
              .font(.subheadline)
              .foregroundColor(.secondary)
          }
          VStack(alignment: .leading) {
            Text("\(feedback.visionService.midRightDistance)")
            Text("Right Wall (mm)\t\t(Right Middle Sensor)")
              .font(.subheadline)
              .foregroundColor(.secondary)
          }
          VStack(alignment: .leading) {
            Text("\(feedback.visionService.farRightDistance)")
            Text("Front Wall (mm)\t\t(Right Edge Sensor)")
              .font(.subheadline)
              .foregroundColor(.secondary)
          }
        }

        Section("Raw Sensor Readings (% of Max Intensity)") {
          VStack(alignment: .leading) {
            Text("\(feedback.visionService.rawFarLeftReading)")
            Text("Left Edge")
              .font(.subheadline)
              .foregroundColor(.secondary)
          }
          VStack(alignment: .leading) {
            Text("\(feedback.visionService.rawMidLeftReading)")
            Text("Left Middle")
              .font(.subheadline)
              .foregroundColor(.secondary)
          }
          VStack(alignment: .leading) {
            Text("\(feedback.visionService.rawMidRightReading)")
            Text("Right Middle")
              .font(.subheadline)
              .foregroundColor(.secondary)
          }
          VStack(alignment: .leading) {
            Text("\(feedback.visionService.rawFarRightReading)")
            Text("Right Edge")
              .font(.subheadline)
              .foregroundColor(.secondary)
          }
        }
        Section(
          header: Text("Calibration"),
          footer: Text(
            "Take measurements when no walls are visible to determine light bleed under/through cover"
          )
        ) {
          VStack(alignment: .leading) {
            Text("\(feedback.visionService.isCalibrated ? "Calibrated" : "Not Calibrated")")
            Text("Is Calibrated?")
              .font(.subheadline)
              .foregroundColor(.secondary)
          }
          Button("Reset") {
            feedback.publishVisionCalibrateReset()
          }
          Button("Calibrate") {
            feedback.publishVisionCalibrate()
          }
        }

      }
      .navigationTitle("Vision")
    }
  }
}

#Preview {
  VisionPage()
    .environmentObject(AppFeedback())
}
