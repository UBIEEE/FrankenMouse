enum StartingPosition: UInt8, CaseIterable, Identifiable {
  case westOfGoal = 0
  case eastOfGoal = 1

  var id: Self { self }
}

