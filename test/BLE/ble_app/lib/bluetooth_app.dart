import 'dart:convert';
import 'dart:async';
import 'package:flutter/material.dart';
import 'package:flutter_reactive_ble/flutter_reactive_ble.dart';

// Define UUIDs as constants - these should match the Arduino code
const String serviceUUID = "00000000-5EA4-4183-85CD-B1EB8D5CFBAD";
const String characteristicUUID = "00000001-5EA4-4183-85CD-B1EB8D5CFBAD";

class MyHomePage extends StatefulWidget {
  const MyHomePage({super.key, required this.title});

  final String title;

  @override
  State<MyHomePage> createState() => _MyHomePageState();
}

class _MyHomePageState extends State<MyHomePage> {
  final _ble = FlutterReactiveBle();

  StreamSubscription<DiscoveredDevice>? _scanSub;
  StreamSubscription<ConnectionStateUpdate>? _connectSub;
  StreamSubscription<List<int>>? _notifySub;

  List<DiscoveredDevice> _devices = [];
  String? _selectedDeviceId; // will hold the device ID selected for connection
  String?
      _selectedDeviceName; // will hold the device name selected for connection
  var _stateMessage = 'Scanning...'; // displays app status
  QualifiedCharacteristic? _writeCharacteristic;

  bool _isConnected = false; // flag to indicate connection

  // on initialization scan for devices
  Timer? _scanTimer;
  Timer? _commandTimer;

  // Dropdown variables
  String? _selectedOption;
  final List<String> _options = ['Option 1', 'Option 2', 'Option 3', 'Option 4', 'Option 5', 'Option 4', 'Option 4', 'Option 4', 'Option 4', 'Option 4'];

  @override
  void initState() {
    super.initState();
    _scanSub = _ble.scanForDevices(withServices: []).listen(_onScanUpdate);
    _scanTimer = Timer.periodic(Duration(seconds: 5), (timer) {
      _scanSub?.cancel();
      _scanSub = _ble.scanForDevices(withServices: []).listen(_onScanUpdate);
    });
  }

  // when terminating cancel all the subscriptions
  @override
  void dispose() {
    _notifySub?.cancel();
    _connectSub?.cancel();
    _scanSub?.cancel();
    _commandTimer?.cancel(); // Cancel the timer when disposed
    super.dispose();
  }

  // update devices that found with "BLE" in their name
  void _onScanUpdate(DiscoveredDevice d) {
    if (d.name.contains("A4") &&
        !_devices.any((device) => device.id == d.id)) {
      setState(() {
        _devices.add(d);
      });
    }
  }

  // Connect to the devices that was selected by user
  void _connectToDevice() {
    if (_selectedDeviceId != null) {
      setState(() {
        _stateMessage = 'Connecting to $_selectedDeviceName...';
      });

      _connectSub = _ble.connectToDevice(id: _selectedDeviceId!).listen(
        (update) {
          if (update.connectionState == DeviceConnectionState.connected) {
            setState(() {
              _stateMessage = 'Connected to $_selectedDeviceName!';
              _isConnected = true;
            });
            _onConnected(_selectedDeviceId!);
          }
        },
        onError: (error) {
          setState(() {
            _stateMessage = 'Connection error: $error';
          });
        },
      );
    }
  }

  // Handle disconnection
  void _disconnectFromDevice() {
    try {
      if (_notifySub != null) {
        _notifySub?.cancel();
        _notifySub = null;
      }

      if (_connectSub != null) {
        _connectSub?.cancel();
        _connectSub = null;
      }

      setState(() {
        _isConnected = false;
        _stateMessage = 'Disconnected from $_selectedDeviceName.';
        _writeCharacteristic = null;
      });
    } catch (e) {
      setState(() {
        _stateMessage = 'Error during disconnection: $e';
      });
    }
  }

  void _onConnected(String deviceId) {
    final characteristic = QualifiedCharacteristic(
      deviceId: deviceId,
      serviceId: Uuid.parse(serviceUUID), // Use the constant here
      characteristicId: Uuid.parse(characteristicUUID), // Use the constant here
    );

    _writeCharacteristic = characteristic;

    _notifySub = _ble.subscribeToCharacteristic(characteristic).listen((bytes) {
      setState(() {
        _stateMessage = 'Data received: ${Utf8Decoder().convert(bytes)}';
      });
    });
  }

  Future<void> _sendCommand(String command) async {
    if (_writeCharacteristic != null) {
      try {
        await _ble.writeCharacteristicWithResponse(
          _writeCharacteristic!,
          value: utf8.encode(command),
        );
        setState(() {
          _stateMessage = "Command '$command' sent!";
        });
      } catch (e) {
        setState(() {
          _stateMessage = "Error sending command: $e";
        });
      }
    }
  }

  // Start sending command when button is pressed
  void _startSendingCommand(String command) {
    _commandTimer?.cancel(); // Cancel any existing timer

    // Send command every 0.5 seconds while button is held down
    _commandTimer = Timer.periodic(Duration(milliseconds: 150), (timer) {
      _sendCommand(command);
    });
  }

  // Stop sending command when button is released
  void _stopSendingCommand() {
    _commandTimer?.cancel();
  }

  String _mapOptionToCommand(String option) {
    switch (option) {
      case 'Option 1':
        return '5';
      case 'Option 2':
        return '6';
      case 'Option 3':
        return '7';
      case 'Option 4':
        return '8';
      case 'Option 5':
        return '9';
      default:
        return 'X';
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        backgroundColor: Theme.of(context).colorScheme.inversePrimary,
        title: Text(widget.title),
      ),
      body: Column(
        children: [
          Container(
            padding: const EdgeInsets.all(16.0),
            color: Colors.grey[300],
            width: double.infinity,
            child: Text(
              _stateMessage,
              style: Theme.of(context).textTheme.titleMedium,
              textAlign: TextAlign.center,
            ),
          ),
          if (_devices.isNotEmpty)
            Padding(
              padding: const EdgeInsets.all(16.0),
              child: DropdownButton<String>(
                isExpanded: true,
                hint: const Text("Select a BLE Device"),
                value: _selectedDeviceId,
                items: _devices.map((device) {
                  return DropdownMenuItem(
                    value: device.id,
                    child: Text(device.name),
                  );
                }).toList(),
                onChanged: (value) {
                  setState(() {
                    _selectedDeviceId = value;
                    _selectedDeviceName = _devices.firstWhere((device) => device.id == value).name;
                  });
                },
              ),
            ),
          if (!_isConnected)
            ElevatedButton(
              onPressed: _selectedDeviceId != null ? _connectToDevice : null,
              child: const Text('Connect'),
            ),
          if (_isConnected)
            ElevatedButton(
              onPressed: _disconnectFromDevice,
              child: const Text('Disconnect'),
            ),

          // **************** Joystick Controls ****************
          Expanded(
            child: Column(
              mainAxisAlignment: MainAxisAlignment.start, // Moves buttons higher
              children: [
                const SizedBox(height: 20), // Space at the top
                Row(
                  mainAxisAlignment: MainAxisAlignment.center,
                  children: [
                    GestureDetector( // Forward
                      onTap: () => _sendCommand('1'),
                      onTapDown: (_) => _startSendingCommand('1'),
                      onTapUp: (_) => _stopSendingCommand(),
                      onTapCancel: () => _stopSendingCommand(),
                      child: ElevatedButton(
                        onPressed: null,
                        style: ElevatedButton.styleFrom(
                          minimumSize: const Size(100, 60), // Wider rectangle
                          shape: RoundedRectangleBorder(
                            borderRadius: BorderRadius.circular(30), // Less rounded edges
                          ),
                        ),
                        child: const Icon(Icons.arrow_upward, size: 40),
                      ),
                    ),
                  ],
                ),
                const SizedBox(height: 10),
                Row(
                  mainAxisAlignment: MainAxisAlignment.center,
                  children: [
                    GestureDetector( // Left
                      onTap: () => _sendCommand('2'),
                      onTapDown: (_) => _startSendingCommand('2'),
                      onTapUp: (_) => _stopSendingCommand(),
                      onTapCancel: () => _stopSendingCommand(),
                      child: ElevatedButton(
                        onPressed: null,
                        style: ElevatedButton.styleFrom(
                          minimumSize: const Size(100, 60),
                          shape: RoundedRectangleBorder(
                            borderRadius: BorderRadius.circular(30),
                          ),
                        ),
                        child: const Icon(Icons.arrow_back, size: 40),
                      ),
                    ),
                    const SizedBox(width: 10),
                    GestureDetector( // Vertical
                      onTap: () => _sendCommand('0'),
                      onTapDown: (_) => _startSendingCommand('0'),
                      onTapUp: (_) => _stopSendingCommand(),
                      onTapCancel: () => _stopSendingCommand(),
                      child: ElevatedButton(
                        onPressed: null,
                        style: ElevatedButton.styleFrom(
                          minimumSize: const Size(100, 60),
                          backgroundColor: Colors.red,
                          shape: RoundedRectangleBorder(
                            borderRadius: BorderRadius.circular(30),
                          ),
                        ),
                        child: const Icon(Icons.stop, color: Colors.white, size: 40),
                      ),
                    ),
                    const SizedBox(width: 10),
                    GestureDetector( // Right
                      onTap: () => _sendCommand('3'),
                      onTapDown: (_) => _startSendingCommand('3'),
                      onTapUp: (_) => _stopSendingCommand(),
                      onTapCancel: () => _stopSendingCommand(),
                      child: ElevatedButton(
                        onPressed: null,
                        style: ElevatedButton.styleFrom(
                          minimumSize: const Size(100, 60),
                          shape: RoundedRectangleBorder(
                            borderRadius: BorderRadius.circular(30),
                          ),
                        ),
                        child: const Icon(Icons.arrow_forward, size: 40),
                      ),
                    ),
                  ],
                ),
                const SizedBox(height: 10),
                Row(
                  mainAxisAlignment: MainAxisAlignment.center,
                  children: [
                    GestureDetector( // Backward
                      onTap: () => _sendCommand('4'),
                      onTapDown: (_) => _startSendingCommand('4'),
                      onTapUp: (_) => _stopSendingCommand(),
                      onTapCancel: () => _stopSendingCommand(),
                      child: ElevatedButton(
                        onPressed: null,
                        style: ElevatedButton.styleFrom(
                          minimumSize: const Size(100, 60),
                          shape: RoundedRectangleBorder(
                            borderRadius: BorderRadius.circular(30),
                          ),
                        ),
                        child: const Icon(Icons.arrow_downward, size: 40),
                      ),
                    ),
                  ],
                ),
                const SizedBox(height: 30),
                const Text("Select an Option:"),
                Padding(
                  padding: const EdgeInsets.all(16.0),
                  child: Container(
                    width: double.infinity, // Ensures the dropdown takes full width
                    child: PopupMenuButton<String>(
                      onSelected: (String newValue) {
                        setState(() {
                          _selectedOption = newValue;
                          _sendCommand(_mapOptionToCommand(newValue)); // Send command
                        });
                      },
                      itemBuilder: (BuildContext context) => _options.map((String value) {
                        return PopupMenuItem<String>(
                          value: value,
                          child: Container(
                            width: double.infinity, // Forces full width for each option
                            padding: EdgeInsets.symmetric(vertical: 12),
                            child: Text(value, textAlign: TextAlign.center),
                          ),
                        );
                      }).toList(),
                      child: Container(
                        padding: EdgeInsets.symmetric(horizontal: 12, vertical: 8),
                        decoration: BoxDecoration(
                          border: Border.all(color: Colors.grey),
                          borderRadius: BorderRadius.circular(5),
                        ),
                        child: Row(
                          mainAxisAlignment: MainAxisAlignment.spaceBetween,
                          children: [
                            Text(_selectedOption ?? "Choose an option"),
                            Icon(Icons.arrow_drop_down),
                          ],
                        ),
                      ),
                    ),
                  ),
                ),
              ],
            ),
          ),
        ],
      ),
    );
  }
}
