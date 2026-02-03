enum Song: UInt8, CaseIterable, Identifiable {
  case none = 0
  case homeDepot = 1
  case nokiaRingtone = 2
  
  case unknown = 255

  var id: Self { self }
}
