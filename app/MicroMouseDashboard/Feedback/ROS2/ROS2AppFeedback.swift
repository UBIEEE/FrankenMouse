import Foundation

class ROS2AppFeedback: NSObject, AppFeedbackBase, ObservableObject {
  @Published var isEnabled: Bool = false

  struct ConnectionState {
    var deviceFound = false
    var deviceConnected = false

    var musicServiceFound = true
    var musicServiceReady = true

    var visionServiceFound = true
    var visionServiceReady = true

    var mainServiceFound = true
    var mainServiceReady = true

    var driveServiceFound = true
    var driveServiceReady = true

    var isReady = true
  }

  @Published var connectionState = ConnectionState()

  @Published var rssi = 0

  struct MusicService {
    var isPlaying = false
  }

  @Published var musicService = MusicService()

  struct VisionService {
    var rawSensorData = [Float32](repeating: 0, count: 4)
    var sensorDistances = [Float32](repeating: 0, count: 4)
    var isCalibrated = false

    var rawFarRightReading: Float32 {
      return rawSensorData[0]
    }

    var rawMidRightReading: Float32 {
      return rawSensorData[1]
    }

    var rawMidLeftReading: Float32 {
      return rawSensorData[2]
    }

    var rawFarLeftReading: Float32 {
      return rawSensorData[3]
    }

    var farRightDistance: Float32 {
      return sensorDistances[0]
    }

    var midRightDistance: Float32 {
      return sensorDistances[1]
    }

    var midLeftDistance: Float32 {
      return sensorDistances[2]
    }

    var farLeftDistance: Float32 {
      return sensorDistances[3]
    }
  }

  @Published var visionService = VisionService()

  struct MainService {
    var currentTask: UInt8 = 0
    var startingPosition: UInt8 = 0
    var errorCodes: [UInt8] = []
  }

  @Published var mainService = MainService()

  struct DriveService {
    var driveData = [Float32](repeating: 0, count: 4 + 3)
    var imuData = [Float32](repeating: 0, count: 6)
    var pidConstants = [Float32](repeating: 0, count: 6)

    var motorLeftPosition: Float32 {
      return driveData[0]
    }

    var motorLeftVelocity: Float32 {
      return driveData[1]
    }

    var motorRightPosition: Float32 {
      return driveData[2]
    }

    var motorRightVelocity: Float32 {
      return driveData[3]
    }

    var xPos: Float32 {
      return driveData[4]
    }

    var yPos: Float32 {
      return driveData[5]
    }

    var thetaRad: Float32 {
      return driveData[6]
    }
  }

  @Published var driveService = DriveService()

  override init() {
    super.init()

    isEnabled = ros2Init(
      self,
      { _self, task in
        _self!.mainService.currentTask = task
      },
      { _self, error in
        _self!.mainService.errorCodes.append(error)
      },
      { _self, _data in
        let data = _data!.compactMap { $0 as? Float32 }
        _self!.driveService.driveData = data
      },
      { _self, _data in
        let data = _data!.compactMap { $0 as? Float32 }
        _self!.driveService.imuData = data
      },
      { _self, _data in
        let data = _data!.compactMap { $0 as? Float32 }
        _self!.driveService.pidConstants = data
      },
      { _self, _data in
        let data = _data!.compactMap { $0 as? Float32 }
        _self!.visionService.rawSensorData = data
      },
      { _self, _data in
        let data = _data!.compactMap { $0 as? Float32 }
        _self!.visionService.sensorDistances = data
      },
      { _self, isCalibrated in
        _self!.visionService.isCalibrated = isCalibrated.boolValue
      },
      { _self, isPlaying in
        _self!.musicService.isPlaying = isPlaying.boolValue
      }
    )
  }

  // Swift is stupid and doesn't call deinit, so I call this manually instead.
  func destroy() {
    ros2Destroy()
  }

  private var updateTimer = Timer.publish(every: 0.015, on: .main, in: .common)
    .autoconnect()
    .sink { _ in
      ros2Process()
    }

  func updateValueFor(topic: FeedbackTopicReceive, data: Data) {
    print("Updating topic \(topic)")
  }

  private func resetState() {
    connectionState = ConnectionState()
  }

  func readRSSI() {}

  func publishMainTask(_ task: UInt8, _ startingPosition: UInt8) {
    ros2PublishMainTask(task, startingPosition)
  }

  func publishAppReady() {
    ros2PublishAppReady()
  }

  func publishDrivePID(_ values: [Float32]) {
    ros2PublishDrivePID(values)
  }

  func publishVisionCalibrate() {
    ros2PublishVisionCalibrate()
  }

  func publishVisionCalibrateReset() {
    ros2PublishVisionCalibrateReset()
  }

  func publishMazeReset() {
    ros2PublishMazeReset()
  }

  func publishMusicSong(_ song: UInt8) {
    ros2PublishMusicSong(song)
  }
}

class AppFeedback: ROS2AppFeedback {}
