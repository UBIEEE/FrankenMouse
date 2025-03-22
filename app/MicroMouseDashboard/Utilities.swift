import SwiftUI

class Utilities {
  static func boolToEmoji(_ isTrue: Bool) -> String {
    return isTrue ? "✅" : "❌"
  }

  static func isPreviewRunning() -> Bool {
    return ProcessInfo.processInfo.environment["XCODE_RUNNING_FOR_PREVIEWS"] == "1"
  }
}
