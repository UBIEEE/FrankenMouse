import SwiftUI

struct MusicPage: View {
  @EnvironmentObject var feedback: AppFeedback

  @State private var selectedSong: Song = .homeDepot

  var body: some View {
    NavigationStack {
      List {
        Section("Status") {
          // Show whether the MicroMouse is currently playing anything!
          Text("\(feedback.mainService.song != .none ? "Playing" : "Not Playing")")
        }
        Section("User Controls") {
          // Select the song to play.
          Picker("Song", selection: $selectedSong) {
            Text("Home Depot").tag(Song.homeDepot)
            Text("Nokia Ringtone").tag(Song.nokiaRingtone)
          }
          Button("\(feedback.mainService.song != .none ? "Restart" : "Play")") {
            // Tell the MicroMouse to play the selected song.
            feedback.publishMainSong(selectedSong)
          }
          Button("Stop") {
            // Tell the MicroMouse to be quiet.
            feedback.publishMainSong(.none)
          }
          // Disable stop button when nothing is playing.
          .disabled(feedback.mainService.song == .none)
        }
      }
      .navigationTitle("Music")
    }
  }
}

#Preview {
  MusicPage()
    .environmentObject(AppFeedback())
}
