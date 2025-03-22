import SwiftUI

struct MusicPage: View {
  @EnvironmentObject var feedback: AppFeedback

  enum Song: UInt8, CaseIterable, Identifiable {
    case homeDepot = 4
    case nokiaRingtone = 5

    var id: Self { self }
  }

  @State private var selectedSong: Song = .homeDepot

  var body: some View {
    NavigationStack {
      List {
        Section("Status") {
          // Show whether the MicroMouse is currently playing anything!
          Text("\(feedback.musicService.isPlaying ? "Playing" : "Not Playing")")
        }
        Section("User Controls") {
          // Select the song to play.
          Picker("Song", selection: $selectedSong) {
            Text("Home Depot").tag(Song.homeDepot)
            Text("Nokia Ringtone").tag(Song.nokiaRingtone)
          }
          Button("\(feedback.musicService.isPlaying ? "Restart" : "Play")") {
            // Tell the MicroMouse to play the selected song.
            feedback.publishMusicSong(selectedSong.rawValue)
          }
          Button("Stop") {
            // Tell the MicroMouse to be quiet.
            feedback.publishMusicSong(0)
          }
          // Disable stop button when nothing is playing.
          .disabled(!feedback.musicService.isPlaying)
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
