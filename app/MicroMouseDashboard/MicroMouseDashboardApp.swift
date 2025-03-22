import SwiftUI

@main
struct MicroMouseDashboardApp: App {
  #if os(macOS)
    @NSApplicationDelegateAdaptor private var appDelegate: AppDelegate
  #endif

  var body: some Scene {
    #if os(macOS)
      // Create single window for macOS app.
      Window("MicroMouse Dashboard", id: "main") {
        ContentView()
      }
    #else
      // Normal window group for iOS app.
      WindowGroup {
        ContentView()
      }
    #endif
  }
}
