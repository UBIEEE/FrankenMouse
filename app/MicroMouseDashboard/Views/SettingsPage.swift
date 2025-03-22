import SwiftUI

struct SettingsPage: View {
  @EnvironmentObject var feedback: AppFeedback

  var body: some View {
    NavigationStack {
      List {
        Section("Drive") {
          NavigationLink("PID Constants") {
            Settings_DrivePIDConstantsPage()
          }
        }
        Section(
          header: Text("Extra stuff"),
          footer: Text(
            "Ask the MicroMouse to re-send all values. This shouldn't ever be necessary, but it's here just in case."
          )
        ) {
          Button("Update All Values") {
            feedback.publishAppReady()
          }
        }
      }
      .navigationTitle("Settings")
    }
  }
}

#Preview {
  SettingsPage()
    .environmentObject(AppFeedback())
}

struct Settings_DrivePIDConstantsPage: View {
  @EnvironmentObject var feedback: AppFeedback

  @State private var pidConstantsText = [String](repeating: "", count: 6)
  @State private var valueEditingIndex = -1

  @FocusState private var focusedField: Bool

  private func sendValues() {
    guard Utilities.isPreviewRunning() == false else { return }

    feedback.publishDrivePID(feedback.driveService.pidConstants)
  }

  private func menuItems(index: Int) -> some View {
    Group {
      #if !os(macOS)
        Button("Edit", systemImage: "pencil") {
          valueEditingIndex = index
        }
      #endif
      Button("Send", systemImage: "arrow.up") {
        sendValues()
      }
    }
  }

  private func stepperValueText(index: Int) -> some View {
    return Text("\(feedback.driveService.pidConstants[index])")
  }

  private func stepperValueTextEdit(index: Int) -> some View {
    func updateTextFromValue() {
      pidConstantsText[index] = String(feedback.driveService.pidConstants[index])
    }

    func updateValueFromText() {
      if let newValue = Float32(pidConstantsText[index]) {
        feedback.driveService.pidConstants[index] = newValue
      }
    }

    return TextField("Value", text: $pidConstantsText[index])
      .onSubmit {
        updateValueFromText()
        valueEditingIndex = -1
        sendValues()
      }
      .onAppear {
        updateTextFromValue()
        #if !os(macOS)
          focusedField = true
        #endif
      }
      #if os(macOS)
        .onChange(of: feedback.driveService.pidConstants) { _, _ in
          updateTextFromValue()
        }
      #else
        .focused($focusedField)
        .keyboardType(.decimalPad)
        .toolbar {
          ToolbarItemGroup(placement: .keyboard) {
            Spacer()
            Button("Done") {
              updateValueFromText()
              valueEditingIndex = -1
              sendValues()
            }
          }
        }
      #endif
  }

  private func stepper(_ title: String, index: Int) -> some View {
    return Stepper(
      value: $feedback.driveService.pidConstants[index], in: 0...Float32.greatestFiniteMagnitude,
      step: 0.001,
      label: {

        #if !os(macOS)
          if valueEditingIndex == index {
            stepperValueTextEdit(index: index)
          } else {
            stepperValueText(index: index)
          }
        #else
          stepperValueTextEdit(index: index)
        #endif
        Text(title)

      },
      onEditingChanged: { notDone in
        if !notDone {
          sendValues()
        }
      }
    )
    .contextMenu {
      menuItems(index: index)
    }
  }

  var body: some View {
    NavigationStack {
      List {
        Section("Translational") {
          stepper("Proportional", index: 0)
          stepper("Integral", index: 1)
          stepper("Derivative", index: 2)
        }
        Section("Rotational") {
          stepper("Proportional", index: 3)
          stepper("Integral", index: 4)
          stepper("Derivative", index: 5)
        }
      }
      .navigationTitle("Drive PID Constants")
    }
  }

}
#Preview {
  Settings_DrivePIDConstantsPage()
    .environmentObject(AppFeedback())
}
