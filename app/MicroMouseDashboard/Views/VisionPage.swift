import SwiftUI

struct VisionPage: View {
  @EnvironmentObject var btManager: BluetoothManager
  
  var body: some View {
    NavigationStack {
      List {
        Section("Distances") {
          VStack(alignment: .leading) {
            Text("\(btManager.visionService.farLeftReading)")
            Text("Front Wall (mm)\t\t(Left Edge Sensor)")
              .font(.subheadline)
              .foregroundColor(.secondary)
          }
          VStack(alignment: .leading) {
            Text("\(btManager.visionService.midLeftReading)")
            Text("Left Wall (mm)\t\t(Left Middle Sensor)")
              .font(.subheadline)
              .foregroundColor(.secondary)
          }
          VStack(alignment: .leading) {
            Text("\(btManager.visionService.midRightReading)")
            Text("Right Wall (mm)\t\t(Right Middle Sensor)")
              .font(.subheadline)
              .foregroundColor(.secondary)
          }
          VStack(alignment: .leading) {
            Text("\(btManager.visionService.farRightReading)")
            Text("Front Wall (mm)\t\t(Right Edge Sensor)")
              .font(.subheadline)
              .foregroundColor(.secondary)
          }
        }
        
        Section("Raw Sensor Readings (% of Max Intensity)") {
          VStack(alignment: .leading) {
            Text("\(btManager.visionService.rawFarLeftReading)")
            Text("Left Edge")
              .font(.subheadline)
              .foregroundColor(.secondary)
          }
          VStack(alignment: .leading) {
            Text("\(btManager.visionService.rawMidLeftReading)")
            Text("Left Middle")
              .font(.subheadline)
              .foregroundColor(.secondary)
          }
          VStack(alignment: .leading) {
            Text("\(btManager.visionService.rawMidRightReading)")
            Text("Right Middle")
              .font(.subheadline)
              .foregroundColor(.secondary)
          }
          VStack(alignment: .leading) {
            Text("\(btManager.visionService.rawFarRightReading)")
            Text("Right Edge")
              .font(.subheadline)
              .foregroundColor(.secondary)
          }
        }
        Section(header: Text("Calibration"),
                footer: Text("Take measurements when no walls are visible to determine light bleed under/through cover")) {
          VStack(alignment: .leading) {
            Text("\(btManager.visionService.isCalibrated ? "Calibrated" : "Not Calibrated")")
            Text("Is Calibrated?")
              .font(.subheadline)
              .foregroundColor(.secondary)
          }
          Button("Reset") {
            let calibrateChar = btManager.connectionState.visionService.calibrateChar!
            let calibrateData = Data([0])
            btManager.writeValueToChar(calibrateChar, calibrateData)
          }
          Button("Calibrate") {
            let calibrateChar = btManager.connectionState.visionService.calibrateChar!
            let calibrateData = Data([1])
            btManager.writeValueToChar(calibrateChar, calibrateData)
          }
        }
        
      }
      .navigationTitle("Vision")
    }
  }
}

#Preview {
  VisionPage()
    .environmentObject(BluetoothManager())
}
