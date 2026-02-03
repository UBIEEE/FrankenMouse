import CoreBluetooth

class AppConstants {

  class Bluetooth {
    static let MicroMouseName = "PetersMicroMouse"

    class MainService {
      static let ServiceUUID = CBUUID(string: "00000000-CC7A-482A-984A-7F2ED5B3E58F")

      // Characteristics (0000-0009)
      static let TaskUUID = CBUUID(string: "00000000-8E22-4541-9D4C-21EDAE82ED19")  // Write & Notify
      static let CommandUUID = CBUUID(string: "00000001-8E22-4541-9D4C-21EDAE82ED19")  // Write
      static let ErrorUUID = CBUUID(string: "00000002-8E22-4541-9D4C-21EDAE82ED19")  // Notify
      static let SongUUID = CBUUID(string: "00000003-8E22-4541-9D4C-21EDAE82ED19") // Write & Notify
      static let StatusUUID = CBUUID(string: "00000004-8E22-4541-9D4C-21EDAE82ED19") // Notify
    }

    class VisionService {
      static let ServiceUUID = CBUUID(string: "00000001-CC7A-482A-984A-7F2ED5B3E58F")

      // Characteristics (000A-000E)
      static let RawReadingsUUID = CBUUID(string: "0000000A-8E22-4541-9D4C-21EDAE82ED19")  // Notify
      static let DistancesUUID = CBUUID(string: "0000000B-8E22-4541-9D4C-21EDAE82ED19")  // Notify
    }

    class DriveService {
      static let ServiceUUID = CBUUID(string: "00000003-CC7A-482A-984A-7F2ED5B3E58F")

      // Characteristics (000F-0013)
      static let MotorDataUUID = CBUUID(string: "0000000F-8E22-4541-9D4C-21EDAE82ED19")  // Notify
      static let IMUDataUUID = CBUUID(string: "00000010-8E22-4541-9D4C-21EDAE82ED19")  // Notify
      static let PIDUUID = CBUUID(string: "00000011-8E22-4541-9D4C-21EDAE82ED19")  // Write & Notify
      static let ChassisSpeedsUUID = CBUUID(string: "00000012-8E22-4541-9D4C-21EDAE82ED19")  // Write & Notify
    }

    class MazeService {
      static let ServiceUUID = CBUUID(string: "00000004-CC7A-482A-984A-7F2ED5B3E58F")

      // Characteristics (0014-0018)
      static let CellUUID = CBUUID(string: "00000014-8E22-4541-9D4C-21EDAE82ED19")  // Write
      static let MousePositionUUID = CBUUID(string: "00000015-8E22-4541-9D4C-21EDAE82ED19")  // Notify

    }
  }
}
