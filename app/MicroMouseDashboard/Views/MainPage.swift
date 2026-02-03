import SwiftUI

struct MainPage: View {
  @EnvironmentObject var feedback: AppFeedback
  
  // Tasks before the cutoff are selectable by the user.
  let userSelectionCutoff = Task.manualChassisSpeeds

  @State private var selectedTask = Task.mazeSearch

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
          if currentTask == .none {
            Text("None")
          } else {
            Text(TaskNames[currentTask] ?? "UNKNOWN")
          }
        }

        Section("Error message") {
          Text("None")
        }

        Section(header: Text("User task Selection"), footer: Text(TaskDescriptions[selectedTask] ?? ""))
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
              if task != .none && task.rawValue < userSelectionCutoff.rawValue {
                Text(TaskNames[task]!)
              }
            }
          }
        }
        Section {
          Button("Run Task") {
            setTask(selectedTask)
          }
        }
        Section {
          Button("Stop") {
            setTask(.none)
          }
        }
      }
      .navigationTitle("MicroMouse")
    }
  }

  func setTask(_ task: Task) {
    feedback.publishMainTask(task, startingPosition)
  }
}

#Preview {
  MainPage()
    .environmentObject(AppFeedback())
}
