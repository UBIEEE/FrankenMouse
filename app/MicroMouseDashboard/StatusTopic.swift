enum StatusTopic: UInt8, CaseIterable, Identifiable {
  case isVisionCalibrated = 0
  case powerSource = 1
  case batteryStatus = 2

  var id: Self { self }
}
  
