import SwiftUI

struct MazePage: View {
  @EnvironmentObject var feedback: AppFeedback

  var body: some View {
    NavigationStack {
      List {
      }
      .navigationTitle("Maze")
    }
  }
}

#Preview {
  MazePage()
    .environmentObject(AppFeedback())
}
