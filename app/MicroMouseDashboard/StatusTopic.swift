enum StatusTopic: UInt8, CaseIterable, Identifiable {
  case none = 0
  case mazeWallGone

  var id: Self { self }
}
  
let StatusTopicDescriptions: [StatusTopic: String] = [
  .none: "Invalid",
  .mazeWallGone: "Maze wall gone",
]
