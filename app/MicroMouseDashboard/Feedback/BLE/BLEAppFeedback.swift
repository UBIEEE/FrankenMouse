import CoreBluetooth

class BLEAppFeedback: NSObject, AppFeedbackBase, ObservableObject,
  CBCentralManagerDelegate, CBPeripheralDelegate
{

  @Published var isEnabled: Bool = false

  private var centralManager: CBCentralManager!
  private var microMouse: CBPeripheral?

  struct ConnectionState {
    var deviceFound = false
    var deviceConnected = false

    var foundWriteChars: [FeedbackTopicWrite: CBCharacteristic] = [:]
    var foundReceiveChars: [FeedbackTopicReceive: CBCharacteristic] = [:]

    var mainServiceFound = false
    var mainServiceReady: Bool {
      return mainServiceFound
        && [.mainTask, .mainCommand, .mainSong].allSatisfy { foundWriteChars.keys.contains($0) }
        && [.mainTask, .mainError, .mainSong, .mainStatus].allSatisfy { foundReceiveChars.keys.contains($0) }
    }

    var visionServiceFound = false
    var visionServiceReady: Bool {
      return visionServiceFound
        && [.visionRawReadings, .visionDistances].allSatisfy { foundReceiveChars.keys.contains($0) }
    }

    var driveServiceFound = false
    var driveServiceReady: Bool {
      return driveServiceFound
        && [.drivePID, .driveChassisSpeeds].allSatisfy { foundWriteChars.keys.contains($0) }
        && [.driveMotorData, .driveIMUData, .drivePID, .driveChassisSpeeds].allSatisfy { foundReceiveChars.keys.contains($0) }
    }
    
    var mazeServiceFound = false
    var mazeServiceReady: Bool {
      return mazeServiceFound
        && [.mazeCell, .mazeCoordinates].allSatisfy { foundReceiveChars.keys.contains($0) }
    }

    var isReady: Bool {
      return deviceFound && deviceConnected && mainServiceReady && visionServiceReady && driveServiceReady && mazeServiceReady
    }
  }

  @Published var connectionState = ConnectionState()

  @Published var rssi = 0

  //
  // Main service
  //

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

  //
  // Vision service
  //

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

  //
  // Drive service
  //

  struct DriveService {
    var driveData = [Float32](repeating: 0, count: 4 + 3)
    var imuData = [Float32](repeating: 0, count: 6)
    var pid = [Float32](repeating: -1, count: 6)
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
  
  //
  // Maze Service
  //
  
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

  //
  // Basic stuff
  //

  override init() {
    super.init()
    self.centralManager = CBCentralManager(delegate: self, queue: nil)
  }

  public func destroy() {
    centralManager.stopScan()
    if microMouse != nil {
      centralManager.cancelPeripheralConnection(microMouse!)
    }
  }

  private func resetState() {
    connectionState = ConnectionState()
  }

  //
  // CBCentralManagerDelegate stuff
  //

  func centralManagerDidUpdateState(_ central: CBCentralManager) {

    if central.state == .poweredOn {
      isEnabled = true
      centralManager.scanForPeripherals(withServices: nil)
    } else {
      isEnabled = false
    }
  }

  func centralManager(
    _ central: CBCentralManager, didDiscover peripheral: CBPeripheral,
    advertisementData: [String: Any], rssi RSSI: NSNumber
  ) {

    if peripheral.name == AppConstants.Bluetooth.MicroMouseName {
      print("MicroMouse Found!")
      microMouse = peripheral
      microMouse!.delegate = self
      centralManager.stopScan()
      centralManager.connect(peripheral)

      connectionState.deviceFound = true
    }
  }

  func centralManager(
    _ central: CBCentralManager,
    didConnect peripheral: CBPeripheral
  ) {

    print("MicroMouse Connected!")
    microMouse!.discoverServices(nil)

    connectionState.deviceConnected = true
  }

  func centralManager(
    _ central: CBCentralManager,
    didDisconnectPeripheral peripheral: CBPeripheral, error: Error?
  ) {

    print("MicroMouse Disconnected!")
    resetState()
    microMouse = nil
    centralManager.scanForPeripherals(withServices: nil)
  }

  //
  // CBPeripheralDelegate stuff
  //

  func peripheral(
    _ peripheral: CBPeripheral,
    didDiscoverServices error: Error?
  ) {

    if let services = peripheral.services {
      for service in services {
        peripheral.discoverCharacteristics(nil, for: service)

        switch service.uuid {
        case AppConstants.Bluetooth.MainService.ServiceUUID:
          connectionState.mainServiceFound = true
        case AppConstants.Bluetooth.VisionService.ServiceUUID:
          connectionState.visionServiceFound = true
        case AppConstants.Bluetooth.DriveService.ServiceUUID:
          connectionState.driveServiceFound = true
        case AppConstants.Bluetooth.MazeService.ServiceUUID:
          connectionState.mazeServiceFound = true
        default:
          print("Unknown Service Discovered: \(service.uuid.uuidString)")
        }
      }
    }

  }

  func peripheral(
    _ peripheral: CBPeripheral,
    didDiscoverCharacteristicsFor service: CBService, error: Error?
  ) {

    if let characteristics = service.characteristics {
      for ch in characteristics {

        func setNotify() {
          microMouse!.setNotifyValue(true, for: ch)
        }

        switch ch.uuid {
          // Main service
        case AppConstants.Bluetooth.MainService.TaskUUID:  // Write & Notify
          connectionState.foundWriteChars[.mainTask] = ch
          connectionState.foundReceiveChars[.mainTask] = ch
          setNotify()
        case AppConstants.Bluetooth.MainService.CommandUUID:  // Write
          connectionState.foundWriteChars[.mainCommand] = ch
        case AppConstants.Bluetooth.MainService.ErrorUUID:  // Notify
          connectionState.foundReceiveChars[.mainError] = ch
          setNotify()
        case AppConstants.Bluetooth.MainService.SongUUID: // Write & Notify
          connectionState.foundWriteChars[.mainSong] = ch
          connectionState.foundReceiveChars[.mainSong] = ch
          setNotify()
        case AppConstants.Bluetooth.MainService.StatusUUID:  // Notify
          connectionState.foundReceiveChars[.mainStatus] = ch
          setNotify()

          // Vision service
        case AppConstants.Bluetooth.VisionService.RawReadingsUUID:  // Notify
          connectionState.foundReceiveChars[.visionRawReadings] = ch
          setNotify()
        case AppConstants.Bluetooth.VisionService.DistancesUUID:  // Notify
          connectionState.foundReceiveChars[.visionDistances] = ch
          setNotify()
          
          // Drive service
        case AppConstants.Bluetooth.DriveService.MotorDataUUID:  // Notify
          connectionState.foundReceiveChars[.driveMotorData] = ch
          setNotify()
        case AppConstants.Bluetooth.DriveService.IMUDataUUID:  // Notify
          connectionState.foundReceiveChars[.driveIMUData] = ch
          setNotify()
        case AppConstants.Bluetooth.DriveService.PIDUUID:  // Write & Notify
          connectionState.foundWriteChars[.drivePID] = ch
          connectionState.foundReceiveChars[.drivePID] = ch
          setNotify()
        case AppConstants.Bluetooth.DriveService.ChassisSpeedsUUID: // Write & Notify
          connectionState.foundWriteChars[.driveChassisSpeeds] = ch
          connectionState.foundReceiveChars[.driveChassisSpeeds] = ch
          setNotify()
          
          // Maze service
        case AppConstants.Bluetooth.MazeService.CellUUID: // Notify
          connectionState.foundReceiveChars[.mazeCell] = ch
          setNotify()
        case AppConstants.Bluetooth.MazeService.MousePositionUUID: // Notify
          connectionState.foundReceiveChars[.mazeCoordinates] = ch
          setNotify()
          
        default:
          print("Unknown Characteristic Discovered: \(ch.uuid)")
        }
      }
    }

  }

  func peripheral(
    _ peripheral: CBPeripheral,
    didUpdateValueFor ch: CBCharacteristic, error: Error?
  ) {

    func getFloatValues(_ data: Data, numValues: Int) -> [Float32] {
      var values: [Float32] = []

      for i in 0..<numValues {
        let valueData = Data(data[i * 4..<(i + 1) * 4])
        let value = valueData.withUnsafeBytes { $0.load(as: Float32.self) }
        values.append(value)
      }

      return values
    }

    switch ch.uuid {
      // Main service
    case AppConstants.Bluetooth.MainService.TaskUUID:
        mainService.currentTask = Task(rawValue: ch.value![0]) ?? .unknown
        mainService.startingPosition = StartingPosition(rawValue: ch.value![1]) ?? .westOfGoal
    case AppConstants.Bluetooth.MainService.ErrorUUID:
        let timestamp: UInt32 = UInt32(ch.value![0]) << 24
                              | UInt32(ch.value![1]) << 16
                              | UInt32(ch.value![2]) << 8
                              | UInt32(ch.value![3])
        let errorCategory = ch.value![5]
        let errorCode = ch.value![4]
        mainService.errorCodes[timestamp] = MainService.Error(timestamp: timestamp,
                                                              errorCategory: errorCategory,
                                                              errorCode: errorCode)
    case AppConstants.Bluetooth.MainService.SongUUID:
      mainService.song = Song(rawValue: ch.value![0]) ?? .unknown
    case AppConstants.Bluetooth.MainService.StatusUUID:
      let topic = StatusTopic(rawValue: ch.value![0])!
      mainService.statusTopics[topic] = ch.value![1]

      // Vision service
    case AppConstants.Bluetooth.VisionService.RawReadingsUUID:
      visionService.rawSensorData = getFloatValues(ch.value!, numValues: 4)
    case AppConstants.Bluetooth.VisionService.DistancesUUID:
      visionService.sensorDistances = getFloatValues(ch.value!, numValues: 4)
      
      // Drive service
    case AppConstants.Bluetooth.DriveService.MotorDataUUID:
      driveService.driveData = getFloatValues(ch.value!, numValues: 4 + 3)
    case AppConstants.Bluetooth.DriveService.IMUDataUUID:
      driveService.imuData = getFloatValues(ch.value!, numValues: 6)
    case AppConstants.Bluetooth.DriveService.PIDUUID:
      driveService.pid = getFloatValues(ch.value!, numValues: 6)
    case AppConstants.Bluetooth.DriveService.ChassisSpeedsUUID:
      let speeds = getFloatValues(ch.value!, numValues: 2)
      driveService.linearVelocity = speeds[0]
      driveService.angularVelocity = speeds[1]
      
    default:
      print("Unknown Characteristic Update: \(ch.uuid)")
    }
  }

  func peripheral(_ peripheral: CBPeripheral, didReadRSSI RSSI: NSNumber, error: Error?) {
    rssi = RSSI.intValue
  }

  func readRSSI() {
    microMouse?.readRSSI()
  }

  private func publishTopic(_ topic: FeedbackTopicWrite, _ value: Data) {
    let char = connectionState.foundWriteChars[topic]!
    microMouse?.writeValue(value, for: char, type: .withResponse)
  }

  func publishMainTask(_ task: Task, _ startingPosition: StartingPosition) {
    let taskData = Data([task.rawValue, startingPosition.rawValue])
    publishTopic(.mainTask, taskData)
  }

  func publishMainCommand(_ command: Command) {
    let commandData = Data([command.rawValue])
    publishTopic(.mainCommand, commandData)
  }
  
  func publishMainSong(_ song: Song) {
    let songData = Data([song.rawValue])
    publishTopic(.mainSong, songData)
  }

  func publishDrivePID(_ values: [Float32]) {
    let valuesData = values.withUnsafeBufferPointer { buffer in
      Data(buffer: buffer)
    }

    publishTopic(.drivePID, valuesData)
  }
  
  func publishDriveChassisSpeeds(_ linear: Float32, _ angular: Float32) {
    let values = [linear, angular]
    let valuesData = values.withUnsafeBufferPointer { buffer in
      Data(buffer: buffer)
    }
    
    publishTopic(.driveChassisSpeeds, valuesData)
  }
}

class AppFeedback: BLEAppFeedback {}
