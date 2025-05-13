import SwiftUI

struct MainPage: View {
  @EnvironmentObject var feedback: AppFeedback

  enum Task: UInt8, CaseIterable, Identifiable {
    // 1-10: Maze tasks
    case mazeSearch = 1
    case mazeSlowSolve = 2
    case mazeFastSolve = 3

    // 11-20: Test drive tasks.
    case testDriveStraight = 11
    case testDriveLeftTurn = 12
    case testDriveRightTurn = 13
    case testDriveTurn180 = 14
    case testGyro = 15
    case testDriveStraightVisionAlign = 16
    
    // 21-30: Manual control tasks.
    case manualChassisSpeeds = 21

    // 100+: Other

    case armed = 100
    case armedTriggering
    case armedTriggered

    case visionCalibrate

    var id: Self { self }
  }
  
  // Tasks before the cutoff are selectable by the user.
  let userSelectionCutoff = Task.manualChassisSpeeds

  private let taskNames: [Task: String] = [
    .mazeSearch: "Maze Search",
    .mazeSlowSolve: "Maze Slow Solve",
    .mazeFastSolve: "Maze Fast Solve",

    .testDriveStraight: "TEST - Drive Straight",
    .testDriveLeftTurn: "TEST - Left Turn",
    .testDriveRightTurn: "TEST - Right Turn",
    .testDriveTurn180: "TEST - Turn 180˚",
    .testGyro: "TEST - Gyro",
    .testDriveStraightVisionAlign: "TEST - Drive Straight Vision Align",
    
    .manualChassisSpeeds: "Manual Control - Chassis Speeds",

    .armed: "Armed",
    .armedTriggering: "Armed Triggering",
    .armedTriggered: "Armed Triggered",

    .visionCalibrate: "Vision Calibrating",
  ]
  private let taskDescriptions: [Task: String] = [
    .mazeSearch: "Search to the center of the maze, then back to the start",
    .mazeSlowSolve: "Solve the maze using the same control method as Search mode",
    .mazeFastSolve: "Solve the maze as fast as possible while using previous search data",

    .testDriveStraight: "Drive straight for 2 cell lengths",
    .testDriveLeftTurn: "Make a left turn",
    .testDriveRightTurn: "Make a right turn",
    .testDriveTurn180: "Turn 180 degrees in place",
    .testGyro: "Maintain a rotational velocity of 0 deg/s",
    .testDriveStraightVisionAlign: "Drive straight while staying centered between maze walls",
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
          let currentTask = feedback.mainService.currentTask
          if currentTask == 0 {
            Text("None")
          } else {
            Text(taskNames[Task(rawValue: currentTask)!]!)
          }
        }

        Section("Error message") {
          Text("None")
        }

        Section(header: Text("User task Selection"), footer: Text(taskDescriptions[selectedTask]!))
        {

          if selectedTask == .mazeSearch {
            Picker("Starting Position", selection: $startingPosition) {
              ForEach(StartingPosition.allCases, id: \.self) { startingPosition in
                Text(startingPositionNames[startingPosition]!)
              }
            }
          }

          Picker("Task", selection: $selectedTask) {
            ForEach(Task.allCases, id: \.self) { task in
              if task.rawValue < userSelectionCutoff.rawValue {
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
    feedback.publishMainTask(task, startingPosition.rawValue)
  }
}

#Preview {
  MainPage()
    .environmentObject(AppFeedback())
}
