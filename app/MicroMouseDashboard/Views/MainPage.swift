import SwiftUI

struct MainPage: View {
  @EnvironmentObject var btManager: BluetoothManager
  
  enum Task: UInt8, CaseIterable, Identifiable {
    case mazeSearch         = 1
    case mazeSlowSolve      = 2
    case mazeFastSolve      = 3
    case testDriveStraight  = 4
    case testDriveLeftTurn  = 5
    case testDriveRightTurn = 6
    case testGyro           = 7
    case testVisionSide     = 8
    
    case armed
    case armedTriggering
    case armedTriggered
    case visionCalibrate
    
    var id: Self { self }
  }
  
  private let taskNames: [Task: String] = [
    .mazeSearch:         "Maze Search",
    .mazeSlowSolve:      "Maze Slow Solve",
    .mazeFastSolve:      "Maze Fast Solve",
    .testDriveStraight:  "TEST - Drive Straight",
    .testDriveLeftTurn:  "TEST - Left Turn",
    .testDriveRightTurn: "TEST - Right Turn",
    .testGyro:           "TEST - Gyro",
    .testVisionSide:     "TEST - Vision",
    .armed:              "Armed",
    .armedTriggering:    "Armed Triggering",
    .armedTriggered:     "Armed Triggered",
    .visionCalibrate:    "Vision Calibrating"
  ]
  private let taskDescriptions: [Task: String] = [
    .mazeSearch: "Search to the center of the maze, then back to the start",
    .mazeSlowSolve: "Solve the maze using the same control method as Search mode",
    .mazeFastSolve: "Solve the maze as fast as possible while using previous search data",
    .testDriveStraight: "Drive straight for 2 cell lengths at 500mm/s",
    .testDriveLeftTurn:  "Make a left turn",
    .testDriveRightTurn: "Make a right turn",
    .testGyro: "Maintain a rotational velocity of 0 deg/s",
    .testVisionSide: "Maintain an equal distance between side walls while stationary",
  ]
  
  @State private var selectedTask = Task.mazeSearch
  
  enum StartingPosition: UInt8, CaseIterable, Identifiable {
    case westOfGoal = 0
    case eastOfGoal = 1
    
    var id: Self { self }
  }
  
  private let startingPositionNames: [StartingPosition: String] = [
    .westOfGoal: "West of Goal",
    .eastOfGoal: "East of Goal",
  ]
  
  @State private var startingPosition = StartingPosition.westOfGoal

  var body: some View {
    NavigationStack {
      List {
        Section("Current task") {
          let currentTask = btManager.mainService.currentTask
          if (currentTask == 0) {
            Text("None")
          }
          else {
            Text(taskNames[Task(rawValue: currentTask)!]!)
          }
        }
        
        Section("Error message") {
          Text("None")
        }
        
        Section(header: Text("User task Selection"), footer: Text(taskDescriptions[selectedTask]!)) {
          
          if (selectedTask == .mazeSearch) {
            Picker("Starting Position", selection: $startingPosition) {
              ForEach(StartingPosition.allCases, id: \.self) { startingPosition in
                Text(startingPositionNames[startingPosition]!)
              }
            }
          }
          
          Picker("Task", selection: $selectedTask) {
            ForEach(Task.allCases, id: \.self) { task in
              if (task.rawValue < Task.armed.rawValue) {
                Text(taskNames[task]!)
              }
            }
          }
        }
        Section {
          Button("Run Task") {
            setTask(selectedTask.rawValue)
          }
        }
        Section {
          Button("Stop") {
            setTask(0)
          }
        }
      }
      .navigationTitle("MicroMouse")
    }
  }
  
  func setTask(_ task: UInt8) {
    let taskChar = btManager.connectionState.mainService.taskChar!
    let taskData = Data([task, startingPosition.rawValue])
    btManager.writeValueToChar(taskChar, taskData)
  }
}

#Preview {
  MainPage()
    .environmentObject(BluetoothManager())
}
