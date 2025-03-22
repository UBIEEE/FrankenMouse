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

    var musicServiceFound = false
    var musicServiceReady: Bool {
      return musicServiceFound && [.musicPlaySong].allSatisfy { foundWriteChars.keys.contains($0) }
        && [.musicIsPlaying].allSatisfy { foundReceiveChars.keys.contains($0) }
    }

    var visionServiceFound = false
    var visionServiceReady: Bool {
      return visionServiceFound
        && [.visionRawData, .visionNormData, .visionCalibrate].allSatisfy {
          foundReceiveChars.keys.contains($0)
        }
    }

    var mainServiceFound = false
    var mainServiceReady: Bool {
      return mainServiceFound
        && [.mainTask, .mainAppReady].allSatisfy { foundWriteChars.keys.contains($0) }
        && [.mainTask, .mainError].allSatisfy { foundReceiveChars.keys.contains($0) }
    }

    var driveServiceFound = false
    var driveServiceReady: Bool {
      return driveServiceFound && [.drivePIDData].allSatisfy { foundWriteChars.keys.contains($0) }
        && [.driveMotorData, .driveIMUData].allSatisfy { foundReceiveChars.keys.contains($0) }
    }

    var isReady: Bool {
      return deviceFound && deviceConnected && musicServiceReady && visionServiceReady
        && mainServiceReady && driveServiceReady
    }
  }

  @Published var connectionState = ConnectionState()

  @Published var rssi = 0

  //
  // Music serivce
  //

  struct MusicService {
    var isPlaying = false
  }

  @Published var musicService = MusicService()

  //
  // Vision service
  //

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

  //
  // Main service
  //

  struct MainService {
    var currentTask: UInt8 = 0
    var startingPosition: UInt8 = 0
    var errorCodes: [UInt8] = []
  }

  @Published var mainService = MainService()

  //
  // Drive service
  //

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
        case AppConstants.Bluetooth.MusicService.ServiceUUID:
          connectionState.musicServiceFound = true
        case AppConstants.Bluetooth.VisionService.ServiceUUID:
          connectionState.visionServiceFound = true
        case AppConstants.Bluetooth.MainService.ServiceUUID:
          connectionState.mainServiceFound = true
        case AppConstants.Bluetooth.DriveService.ServiceUUID:
          connectionState.driveServiceFound = true
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
        // Music service
        case AppConstants.Bluetooth.MusicService.PlaySongUUID:  // Write
          connectionState.foundWriteChars[.musicPlaySong] = ch
        case AppConstants.Bluetooth.MusicService.IsPlayingUUID:  // Notify
          connectionState.foundReceiveChars[.musicIsPlaying] = ch
          setNotify()

        // Vision service
        case AppConstants.Bluetooth.VisionService.RawReadingsUUID:  // Notify
          connectionState.foundReceiveChars[.visionRawData] = ch
          setNotify()
        case AppConstants.Bluetooth.VisionService.DistancesUUID:  // Notify
          connectionState.foundReceiveChars[.visionNormData] = ch
          setNotify()
        case AppConstants.Bluetooth.VisionService.CalibrateUUID:  // Write & Notify
          connectionState.foundWriteChars[.visionCalibrate] = ch
          connectionState.foundReceiveChars[.visionCalibrate] = ch
          setNotify()

        // Main service
        case AppConstants.Bluetooth.MainService.TaskUUID:  // Write
          connectionState.foundWriteChars[.mainTask] = ch
          setNotify()
        case AppConstants.Bluetooth.MainService.AppReadyUUID:  // Write
          connectionState.foundWriteChars[.mainAppReady] = ch
        case AppConstants.Bluetooth.MainService.ErrorCodeUUID:  // Notify
          connectionState.foundReceiveChars[.mainError] = ch
          setNotify()

        // Drive service
        case AppConstants.Bluetooth.DriveService.MotorDataUUID:  // Notify
          connectionState.foundReceiveChars[.driveMotorData] = ch
          setNotify()
        case AppConstants.Bluetooth.DriveService.IMUDataUUID:  // Notify
          connectionState.foundReceiveChars[.driveIMUData] = ch
          setNotify()
        case AppConstants.Bluetooth.DriveService.PIDConstantsUUID:  // Notify
          connectionState.foundReceiveChars[.drivePIDData] = ch
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
    // Music service
    case AppConstants.Bluetooth.MusicService.IsPlayingUUID:
      musicService.isPlaying = ch.value![0] == 1

    // Vision service
    case AppConstants.Bluetooth.VisionService.RawReadingsUUID:
      visionService.rawSensorData = getFloatValues(ch.value!, numValues: 4)
    case AppConstants.Bluetooth.VisionService.DistancesUUID:
      visionService.sensorDistances = getFloatValues(ch.value!, numValues: 4)
    case AppConstants.Bluetooth.VisionService.CalibrateUUID:
      visionService.isCalibrated = ch.value![0] == 1

    // Main service
    case AppConstants.Bluetooth.MainService.TaskUUID:
      mainService.currentTask = ch.value![0]
      mainService.startingPosition = ch.value![1]
    case AppConstants.Bluetooth.MainService.ErrorCodeUUID:
      mainService.errorCodes.append(ch.value![0])

    // Drive service
    case AppConstants.Bluetooth.DriveService.MotorDataUUID:
      driveService.driveData = getFloatValues(ch.value!, numValues: 4 + 3)
    case AppConstants.Bluetooth.DriveService.IMUDataUUID:
      driveService.imuData = getFloatValues(ch.value!, numValues: 6)
    case AppConstants.Bluetooth.DriveService.PIDConstantsUUID:
      driveService.pidConstants = getFloatValues(ch.value!, numValues: 6)

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

  func publishMainTask(_ task: UInt8, _ startingPosition: UInt8) {
    let taskData = Data([task, startingPosition])
    publishTopic(.mainTask, taskData)
  }

  func publishAppReady() {
    let appReadyData = Data([1])
    publishTopic(.mainAppReady, appReadyData)
  }

  func publishDrivePID(_ values: [Float32]) {
    let valuesData = values.withUnsafeBufferPointer { buffer in
      Data(buffer: buffer)
    }

    publishTopic(.drivePIDData, valuesData)
  }

  func publishVisionCalibrate() {
    let calibrateData = Data([1])
    publishTopic(.visionCalibrate, calibrateData)
  }

  func publishVisionCalibrateReset() {
    let calibrateData = Data([0])
    publishTopic(.visionCalibrate, calibrateData)
  }

  func publishMazeReset() {
    let resetData = Data([1])
    publishTopic(.mazeReset, resetData)
  }

  func publishMusicSong(_ song: UInt8) {
    // Tell the MicroMouse to play the selected song.
    let playData = Data([song])
    publishTopic(.musicPlaySong, playData)
  }
}

class AppFeedback: BLEAppFeedback {}
