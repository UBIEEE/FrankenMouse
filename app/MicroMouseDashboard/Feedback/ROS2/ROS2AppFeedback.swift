import Foundation

class ROS2AppFeedback: NSObject, AppFeedbackBase, ObservableObject {
  @Published var isEnabled: Bool = false

  struct ConnectionState {
    var deviceFound = false
    var deviceConnected = false

    var mainServiceFound = true
    var mainServiceReady = true

    var visionServiceFound = true
    var visionServiceReady = true

    var driveServiceFound = true
    var driveServiceReady = true
    
    var mazeServiceFound = true
    var mazeServiceReady = true

    var isReady = true
  }

  @Published var connectionState = ConnectionState()

  @Published var rssi = 0

  struct MainService {
    var currentTask: Task = .none
    var startingPosition: StartingPosition = .westOfGoal
    
    struct Error {
      var timestamp: UInt32 = 0
      var errorCategory: UInt8 = 0
      var errorCode: UInt8 = 0
    }
    
    var errorCodes: [UInt32: Error] = [:]
    
    var song: Song = .none
    
    var statusTopics: [StatusTopic: UInt8] = [:]
  }

  @Published var mainService = MainService()

  struct VisionService {
    var rawSensorData = [Float32](repeating: 0, count: 4)
    var sensorDistances = [Float32](repeating: 0, count: 4)

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

  struct DriveService {
    var driveData = [Float32](repeating: 0, count: 4 + 3)
    var imuData = [Float32](repeating: 0, count: 6)
    var pid = [Float32](repeating: 0, count: 6)
    var linearVelocity: Float32 = 0
    var angularVelocity: Float32 = 0

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
  
  struct MazeService {
    struct Cell {
      var north : Bool = false
      var east : Bool = false
      var south : Bool = false
      var west : Bool = false
      var visited : Bool = false
    }
    
    var grid: [[Cell]] = Array(repeating: Array(repeating: .init(), count: 16), count: 16)
    
    var mousePosition: (x: Int, y: Int) = (0, 0)
  }
  
  @Published var mazeService = MazeService()

  override init() {
    super.init()

    isEnabled = ros2Init(
      self,
      { _self, task, startingPosition in
        _self!.mainService.currentTask = Task(rawValue: task) ?? .unknown
        _self!.mainService.startingPosition = StartingPosition(rawValue: startingPosition) ?? .westOfGoal;
      },
      { _self, timestamp, errorCategory, errorCode in
        let error = MainService.Error(timestamp: timestamp, errorCategory: errorCategory, errorCode: errorCode)
        _self!.mainService.errorCodes[timestamp] = error;
      },
      { _self, song in
        _self!.mainService.song = Song(rawValue: song) ?? .unknown
      },
      { _self, _statusTopic, value in
        let statusTopic = StatusTopic(rawValue: _statusTopic)!
        _self!.mainService.statusTopics[statusTopic] = value;
      },
      { _self, _data in
        let data = _data!.compactMap { $0 as? Float32 }
        _self!.visionService.rawSensorData = data
      },
      { _self, _data in
        let data = _data!.compactMap { $0 as? Float32 }
        _self!.visionService.sensorDistances = data
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
        _self!.driveService.pid = data
      },
      { _self, linear, angular in
        _self!.driveService.linearVelocity = linear;
        _self!.driveService.angularVelocity = angular;
      },
      { _self, x, y, north, east, south, west, visited in
        let cell = MazeService.Cell(north: north.boolValue, east: east.boolValue, south: south.boolValue, west: west.boolValue, visited: visited.boolValue)
        _self!.mazeService.grid[Int(x)][Int(y)] = cell
      },
      { _self, x, y in
        _self!.mazeService.mousePosition = (Int(x), Int(y))
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

  func publishMainTask(_ task: Task, _ startingPosition: StartingPosition) {
    ros2PublishMainTask(task.rawValue, startingPosition.rawValue)
  }

  func publishMainCommand(_ command: Command) {
    ros2PublishMainCommand(command.rawValue)
  }
  
  func publishMainSong(_ song: Song) {
    ros2PublishMainSong(song.rawValue)
  }

  func publishDrivePID(_ values: [Float32]) {
    ros2PublishDrivePID(values)
  }

  func publishDriveChassisSpeeds(_ linear: Float32, _ angular: Float32) {
    ros2PublishDriveChassisSpeeds(linear, angular)
  }
}

class AppFeedback: ROS2AppFeedback {}
