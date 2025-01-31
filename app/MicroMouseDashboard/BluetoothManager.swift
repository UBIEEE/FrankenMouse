import CoreBluetooth

class BluetoothManager: NSObject, ObservableObject,
                        CBCentralManagerDelegate, CBPeripheralDelegate {
  
  @Published var isBluetoothEnabled: Bool = false
  
  private var centralManager: CBCentralManager!
  private var microMouse: CBPeripheral?
  
  //
  // Connection state
  //
  
  struct ConnectionState {
    var deviceFound = false
    var deviceConnected = false
    
    //
    // Music service
    //
    
    struct MusicService {
      var serviceFound = false
      var playSongChar: CBCharacteristic?
      var isPlayingChar: CBCharacteristic?
      
      var isReady: Bool {
        get {
          return serviceFound && playSongChar != nil && isPlayingChar != nil
        }
      }
    }
    var musicService = MusicService()
    
    //
    // Vision service
    //
    
    struct VisionService {
      var serviceFound = false
      var rawDataChar: CBCharacteristic?
      var normalizedDataChar: CBCharacteristic?
      var calibrateChar: CBCharacteristic?
      
      var isReady: Bool {
        get {
          return serviceFound && rawDataChar != nil &&
                 normalizedDataChar != nil && calibrateChar != nil
        }
      }
    }
    
    var visionService = VisionService()
    
    //
    // Main service
    //
    
    struct MainService {
      var serviceFound = false
      var taskChar: CBCharacteristic?
      var appReadyChar: CBCharacteristic?
      var errorCodeChar: CBCharacteristic?
      
      var isReady: Bool {
        get {
          return serviceFound && taskChar != nil && appReadyChar != nil &&
                 errorCodeChar != nil
        }
      }
    }
    
    var mainService = MainService()
    
    //
    // Drive service
    //
    
    struct DriveService {
      var serviceFound = false
      var motorDataChar: CBCharacteristic?
      var imuDataChar: CBCharacteristic?
      var pidConstantsChar: CBCharacteristic?
      
      var isReady: Bool {
        get {
          return serviceFound && motorDataChar != nil && imuDataChar != nil &&
                 pidConstantsChar != nil
        }
      }
    }
    
    var driveService = DriveService()
    
    var isReady : Bool {
      get {
        return deviceFound && deviceConnected && musicService.isReady &&
               visionService.isReady && mainService.isReady &&
               driveService.isReady
      }
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
    var rawSensorData        = [Float32](repeating: 0, count: 4)
    var normalizedSensorData = [Float32](repeating: 0, count: 4)
    var isCalibrated         = false

    var rawFarRightReading: Float32 {
      get { return rawSensorData[0] }
    }
    
    var rawMidRightReading: Float32 {
      get { return rawSensorData[1] }
    }
    
    var rawMidLeftReading: Float32 {
      get { return rawSensorData[2] }
    }
    
    var rawFarLeftReading: Float32 {
      get { return rawSensorData[3] }
    }
    
    var farRightReading: Float32 {
      get { return normalizedSensorData[0] }
    }
    
    var midRightReading: Float32 {
      get { return normalizedSensorData[1] }
    }
    
    var midLeftReading: Float32 {
      get { return normalizedSensorData[2] }
    }
    
    var farLeftReading: Float32 {
      get { return normalizedSensorData[3] }
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
    var driveData    = [Float32](repeating: 0, count: 4+3)
    var imuData      = [Float32](repeating: 0, count: 6)
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
  
  private func resetState() {
    connectionState = ConnectionState()
  }
  
  //
  // CBCentralManagerDelegate stuff
  //
  
  func centralManagerDidUpdateState(_ central: CBCentralManager) {
    
    if central.state == .poweredOn {
      isBluetoothEnabled = true
      centralManager.scanForPeripherals(withServices: nil)
    }
    else {
      isBluetoothEnabled = false
    }
  }
  
  func centralManager(_ central: CBCentralManager, didDiscover peripheral: CBPeripheral,
                      advertisementData: [String: Any], rssi RSSI: NSNumber) {
    
    if peripheral.name == AppConstants.Bluetooth.MicroMouseName {
      print("MicroMouse Found!");
      microMouse = peripheral
      microMouse!.delegate = self
      centralManager.stopScan()
      centralManager.connect(peripheral)
      
      connectionState.deviceFound = true
    }
  }
  
  func centralManager(_ central: CBCentralManager,
                      didConnect peripheral: CBPeripheral) {
    
    print("MicroMouse Connected!")
    microMouse!.discoverServices(nil)
    
    connectionState.deviceConnected = true
  }
  
  func centralManager(_ central: CBCentralManager,
                      didDisconnectPeripheral peripheral: CBPeripheral, error: Error?) {
    
    print("MicroMouse Disconnected!")
    resetState()
    microMouse = nil
    centralManager.scanForPeripherals(withServices: nil)
  }
  
  
  //
  // CBPeripheralDelegate stuff
  //
  
  func peripheral(_ peripheral: CBPeripheral,
                  didDiscoverServices error: Error?) {

    if let services = peripheral.services {
      for service in services {
        peripheral.discoverCharacteristics(nil, for: service)

        switch service.uuid {
        case AppConstants.Bluetooth.MusicService.ServiceUUID:
          connectionState.musicService.serviceFound = true
        case AppConstants.Bluetooth.VisionService.ServiceUUID:
          connectionState.visionService.serviceFound = true
        case AppConstants.Bluetooth.MainService.ServiceUUID:
          connectionState.mainService.serviceFound = true
        case AppConstants.Bluetooth.DriveService.ServiceUUID:
          connectionState.driveService.serviceFound = true
        default:
          print("Unknown Service Discovered: \(service.uuid.uuidString)")
        }
      }
    }
    
  }

  func peripheral(_ peripheral: CBPeripheral,
                  didDiscoverCharacteristicsFor service: CBService, error: Error?) {
    
    if let characteristics = service.characteristics {
      for ch in characteristics {
        
        func setNotify() {
          microMouse!.setNotifyValue(true, for: ch)
        }
        
        switch ch.uuid {
        // Music service
        case AppConstants.Bluetooth.MusicService.PlaySongUUID: // Write
          connectionState.musicService.playSongChar = ch
        case AppConstants.Bluetooth.MusicService.IsPlayingUUID: // Notify
          connectionState.musicService.isPlayingChar = ch
          setNotify()
          
        // Vision service
        case AppConstants.Bluetooth.VisionService.RawDataUUID: // Notify
          connectionState.visionService.rawDataChar = ch
          setNotify()
        case AppConstants.Bluetooth.VisionService.NormalizedDataUUID: // Notify
          connectionState.visionService.normalizedDataChar = ch
          setNotify()
        case AppConstants.Bluetooth.VisionService.CalibrateUUID: // Write & Notify
          connectionState.visionService.calibrateChar = ch
          setNotify()

        // Main service
        case AppConstants.Bluetooth.MainService.TaskUUID: // Write
          connectionState.mainService.taskChar = ch
          setNotify()
        case AppConstants.Bluetooth.MainService.AppReadyUUID: // Write
          connectionState.mainService.appReadyChar = ch
        case AppConstants.Bluetooth.MainService.ErrorCodeUUID:
          connectionState.mainService.errorCodeChar = ch
          setNotify()
          
        // Drive service
        case AppConstants.Bluetooth.DriveService.MotorDataUUID: // Notify
          connectionState.driveService.motorDataChar = ch
          setNotify()
        case AppConstants.Bluetooth.DriveService.IMUDataUUID: // Notify
          connectionState.driveService.imuDataChar = ch
          setNotify()
        case AppConstants.Bluetooth.DriveService.PIDConstantsUUID: // Notify
          connectionState.driveService.pidConstantsChar = ch
          setNotify()
          
        default:
          print("Unknown Characteristic Discovered: \(ch.uuid)")
        }
      }
    }
       
  }
  
  func peripheral(_ peripheral: CBPeripheral,
                  didUpdateValueFor ch: CBCharacteristic, error: Error?) {
    
    func getFloatValues(_ data: Data, numValues: Int) -> [Float32] {
      var values: [Float32] = []
      
      for i in 0..<numValues {
        let valueData = Data(data[i*4..<(i+1)*4])
        let value = valueData.withUnsafeBytes { $0.load(as: Float32.self )}
        values.append(value)
      }
      
      return values
    }

    switch ch.uuid {
    // Music service
    case AppConstants.Bluetooth.MusicService.IsPlayingUUID:
      musicService.isPlaying = ch.value![0] == 1
      
    // Vision service
    case AppConstants.Bluetooth.VisionService.RawDataUUID:
      visionService.rawSensorData = getFloatValues(ch.value!, numValues: 4)
    case AppConstants.Bluetooth.VisionService.NormalizedDataUUID:
      visionService.normalizedSensorData = getFloatValues(ch.value!, numValues: 4)
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
      driveService.driveData = getFloatValues(ch.value!, numValues: 4+3)
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
  
  func writeValueToChar(_ characteristic: CBCharacteristic, _ value: Data) {
    microMouse?.writeValue(value, for: characteristic, type: .withResponse)
  }
}
