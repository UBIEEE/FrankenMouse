import SwiftUI

struct MainPage: View {
  @EnvironmentObject var feedback: AppFeedback

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
          Picker("Task", selection: $selectedTask) {
            ForEach(Task.allCases, id: \.self) { task in
              if task != .none && TaskDescriptions[task] != nil { // Tasks with no description will not be options
                Text(TaskNames[task]!)
              }
            }
          }
#if !os(macOS)
          .pickerStyle(.wheel)
#endif
        }
        
        if selectedTask == .mazeSearch {
          Picker("Starting Position", selection: $startingPosition) {
            ForEach(StartingPosition.allCases, id: \.self) { startingPosition in
              Text(startingPositionNames[startingPosition]!)
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
