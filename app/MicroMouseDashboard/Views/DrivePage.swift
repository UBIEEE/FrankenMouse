import SpriteKit
import SwiftUI

let MAZE_WIDTH_CM = 16 * 18
let MAZE_WIDTH_MM = MAZE_WIDTH_CM * 10

struct DrivePage: View {
  @EnvironmentObject var feedback: AppFeedback

  var body: some View {
    var odometryScene: OdometryScene {
      let scene = OdometryScene()
      scene.size = CGSize(width: MAZE_WIDTH_CM, height: MAZE_WIDTH_CM)
      scene.scaleMode = .fill
      #if !os(macOS)
        scene.backgroundColor = .secondarySystemGroupedBackground
      #endif
      scene.addMouse()
      return scene
    }

    NavigationStack {
      List {
        Section("Left Encoder") {
          VStack(alignment: .leading) {
            Text("\(feedback.driveService.motorLeftPosition)")
            Text("Distance (mm)")
              .font(.subheadline)
              .foregroundColor(.secondary)
          }
          VStack(alignment: .leading) {
            Text("\(feedback.driveService.motorLeftVelocity)")
            Text("Velocity (mm/s)")
              .font(.subheadline)
              .foregroundColor(.secondary)
          }
        }
        Section("Right Encoder") {
          VStack(alignment: .leading) {
            Text("\(feedback.driveService.motorRightPosition)")
            Text("Distance (mm)")
              .font(.subheadline)
              .foregroundColor(.secondary)
          }
          VStack(alignment: .leading) {
            Text("\(feedback.driveService.motorRightVelocity)")
            Text("Velocity (mm/s)")
              .font(.subheadline)
              .foregroundColor(.secondary)
          }
        }
        let d = feedback.driveService
        Section("Position (mm)") {
          Text("X\t").foregroundColor(.secondary) + Text(String(format: "%.2f", d.xPos))

          Text("Y\t").foregroundColor(.secondary) + Text(String(format: "%.2f", d.yPos))
        }
        Section("Rotation (deg)") {
          Text("θ\t").foregroundColor(.secondary) + Text(String(format: "%.2f", d.thetaRad))
        }
        Section("Odometry") {
          SpriteView(scene: odometryScene)
            .aspectRatio(1, contentMode: .fit)
            .frame(maxWidth: 350)
            .onChange(of: d.driveData) {
              odometryScene.updatePos(xPos: d.xPos, yPos: d.yPos, thetaRad: d.thetaRad)
            }

        }
      }
      .navigationTitle("Drive")
    }
  }
}

class OdometryScene: SKScene {
  var mouseNode = SKSpriteNode()

  func addMouse() {
    let size = CGSize(width: 7, height: 10)
    #if os(macOS)
      mouseNode = SKSpriteNode(color: .systemBlue, size: size)
    #else
      let image = UIImage(systemName: "arrowshape.up.fill")
      let texture = SKTexture(image: image!)
      mouseNode = SKSpriteNode(texture: texture, size: size)
    #endif

    addChild(mouseNode)

    updatePos(
      xPos: Float32(MAZE_WIDTH_MM) / 2.0,
      yPos: Float32(MAZE_WIDTH_MM) / 2.0,
      thetaRad: 0)
  }

  func updatePos(xPos: Float32, yPos: Float32, thetaRad: Float32) {
    let x = CGFloat(xPos / 10.0)
    let y = CGFloat(Float32(MAZE_WIDTH_CM) - yPos / 10.0)

    mouseNode.position = CGPoint(x: x, y: y)
    mouseNode.zRotation = CGFloat(thetaRad)
  }
}

#Preview {
  DrivePage()
    .environmentObject(AppFeedback())
}
