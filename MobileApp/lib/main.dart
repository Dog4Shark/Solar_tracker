import 'dart:async';
import 'dart:convert';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_localizations/flutter_localizations.dart';
import 'package:intl/intl.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';
import 'package:time_machine/time_machine.dart';
import 'package:http/http.dart' as http;
import 'package:shared_preferences/shared_preferences.dart';

void main() async {
  WidgetsFlutterBinding.ensureInitialized();
  await TimeMachine.initialize({'rootBundle': rootBundle, 'asset': 'assets/time_machine/data/tzdb/tzdb.bin'});
  SharedPreferences prefs = await SharedPreferences.getInstance();
  String? localeCode = prefs.getString('locale');
  String? serverAddress = prefs.getString('server_address') ?? 'localhost';
  int? installationNumber = prefs.getInt('installation_number') ?? 1;
  runApp(MyApp(localeCode: localeCode, serverAddress: serverAddress, installationNumber: installationNumber));
}

class MyApp extends StatefulWidget {
  final String? localeCode;
  final String serverAddress;
  final int installationNumber;

  MyApp({this.localeCode, required this.serverAddress, required this.installationNumber});

  static void setLocale(BuildContext context, Locale newLocale) async {
    _MyAppState? state = context.findAncestorStateOfType<_MyAppState>();
    state?.setLocale(newLocale);
    SharedPreferences prefs = await SharedPreferences.getInstance();
    prefs.setString('locale', newLocale.languageCode);
  }

  static void setServerAddress(BuildContext context, String newAddress) async {
    _MyAppState? state = context.findAncestorStateOfType<_MyAppState>();
    state?.setServerAddress(newAddress);
    SharedPreferences prefs = await SharedPreferences.getInstance();
    prefs.setString('server_address', newAddress);
  }

  static void setInstallationNumber(BuildContext context, int newNumber) async {
    _MyAppState? state = context.findAncestorStateOfType<_MyAppState>();
    state?.setInstallationNumber(newNumber);
    SharedPreferences prefs = await SharedPreferences.getInstance();
    prefs.setInt('installation_number', newNumber);
  }

  @override
  _MyAppState createState() => _MyAppState();
}

class _MyAppState extends State<MyApp> {
  Locale _locale = Locale('en', 'US');
  String _serverAddress = 'localhost';
  int _installationNumber = 1;

  @override
  void initState() {
    super.initState();
    if (widget.localeCode != null) {
      setLocale(Locale(widget.localeCode!));
    }
    setServerAddress(widget.serverAddress);
    setInstallationNumber(widget.installationNumber);
  }

  void setLocale(Locale locale) {
    setState(() {
      _locale = locale;
    });
  }

  void setServerAddress(String address) {
    setState(() {
      _serverAddress = address;
    });
  }

  void setInstallationNumber(int number) {
    setState(() {
      _installationNumber = number;
    });
  }

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'PV Solar Tracker',
      theme: ThemeData(
        primarySwatch: Colors.blueGrey,
        brightness: Brightness.dark,
      ),
      locale: _locale,
      localizationsDelegates: [
        AppLocalizations.delegate,
        GlobalMaterialLocalizations.delegate,
        GlobalWidgetsLocalizations.delegate,
      ],
      supportedLocales: [
        const Locale('en', 'US'),
        const Locale('pl', 'PL'),
      ],
      home: MyHomePage(serverAddress: _serverAddress, installationNumber: _installationNumber),
      debugShowCheckedModeBanner: false,
    );
  }
}

class MyHomePage extends StatefulWidget {
  final String serverAddress;
  final int installationNumber;
  MyHomePage({required this.serverAddress, required this.installationNumber});

  @override
  _MyHomePageState createState() => _MyHomePageState();
}

class _MyHomePageState extends State<MyHomePage> {
  DateTime selectedDate = DateTime.now();
  int timezoneOffset = 0;
  double longitude = 0;
  double latitude = 0;
  double azimuth = 0;
  double elevation = 0;
  double targetAzimuth = 0;
  double targetElevation = 0;
  double autoAzimuth = 0;
  double autoElevation = 0;
  String language = 'en';
  bool isManualMode = true;
  double batteryCapacity = 0.0;
  double batteryVoltage = 0.0;
  double batteryChargeCurrent = 0.0;
  double pvVoltage = 0.0;
  double pvPower = 0.0;
  DateTime sunrise = DateTime.now();
  DateTime sunset = DateTime.now();
  bool fetchDataSuccess = true;
  bool sendDataSuccess = true;
  double minAzimuth = 0.0;
  double maxAzimuth = 90.0;
  double minElevation = 0.0;
  double maxElevation = 180.0;
  int actuatorFrequency = 0;

  bool showDateTimeSection = true;
  bool showCoordinatesSection = true;
  bool showAzimuthElevationSection = true;
  bool showBatterySection = true;
  bool showActuatorSection = true;

  TextEditingController longitudeController = TextEditingController();
  TextEditingController latitudeController = TextEditingController();
  TextEditingController azimuthController = TextEditingController();
  TextEditingController elevationController = TextEditingController();
  TextEditingController targetAzimuthController = TextEditingController();
  TextEditingController targetElevationController = TextEditingController();

  TextEditingController serverAddressController = TextEditingController();
  TextEditingController installationNumberController = TextEditingController();

  Timer? periodicTimer;

  @override
  void initState() {
    super.initState();
    serverAddressController.text = widget.serverAddress;
    installationNumberController.text = widget.installationNumber.toString();
    fetchData();
    periodicTimer = Timer.periodic(Duration(seconds: 10), (timer) {
      fetchData();
    });
  }

  @override
  void dispose() {
    periodicTimer?.cancel();
    super.dispose();
  }

  Future<void> fetchData() async {
    try {
      final response = await http.get(Uri.parse('http://${widget.serverAddress}/get_data.php?id=${widget.installationNumber}'));

      if (response.statusCode == 200) {
        final data = json.decode(response.body);
        setState(() {
          selectedDate = DateTime.parse(data['data_time']);
          timezoneOffset = int.parse(data['timezone_offset']);
          longitude = double.parse(data['longitude']);
          latitude = double.parse(data['latitude']);
          azimuth = double.parse(data['azimuth']);
          elevation = double.parse(data['elevation']);
          batteryCapacity = double.parse(data['battery_capacity']);
          batteryVoltage = double.parse(data['battery_voltage']);
          batteryChargeCurrent = double.parse(data['battery_charge_current']);
          pvVoltage = double.parse(data['pv_voltage']);
          pvPower = double.parse(data['pv_power']);
          sunrise = DateTime.parse(data['sunrise']);
          sunset = DateTime.parse(data['sunset']);
          isManualMode = data['is_manual_mode'] == '1';
          targetAzimuth = double.parse(data['target_azimuth'] ?? data['azimuth']);
          targetElevation = double.parse(data['target_elevation'] ?? data['elevation']);
          autoAzimuth = double.parse(data['auto_azimuth'] ?? data['azimuth']);
          autoElevation = double.parse(data['auto_elevation'] ?? data['elevation']);
          minAzimuth = double.parse(data['min_azimuth'] ?? '0.0');
          maxAzimuth = double.parse(data['max_azimuth'] ?? '90.0');
          minElevation = double.parse(data['min_elevation'] ?? '0.0');
          maxElevation = double.parse(data['max_elevation'] ?? '180.0');
          actuatorFrequency = int.parse(data['actuator_frequency']);
          fetchDataSuccess = true;

          longitudeController.text = longitude.toStringAsFixed(6);
          latitudeController.text = latitude.toStringAsFixed(6);
          azimuthController.text = azimuth.toString();
          elevationController.text = elevation.toString();
          targetAzimuthController.text = targetAzimuth.toString();
          targetElevationController.text = targetElevation.toString();

        });
      } else {
        setState(() {
          fetchDataSuccess = false;
        });
      }
    } catch (e) {
      setState(() {
        fetchDataSuccess = false;
      });
      print('Error: $e');
    }
  }

  Future<void> updateData(String key, dynamic value) async {
    try {
      final response = await http.post(
        Uri.parse('http://${widget.serverAddress}/update_data.php'),
        body: {
          'key': key,
          'value': value.toString(),
          'updated_by_app': '1',
          'id': widget.installationNumber.toString(),
        },
      );
      print(response.body);
      if (response.statusCode == 200) {
        setState(() {
          sendDataSuccess = true;
        });
      } else {
        setState(() {
          sendDataSuccess = false;
        });
      }
    } catch (e) {
      setState(() {
        sendDataSuccess = false;
      });
      print('Error: $e');
    }
  }

  String formatDateTime(DateTime dateTime) {
    return DateFormat('yyyy-MM-dd HH:mm:ss').format(dateTime);
  }

  void _selectDate() async {
    periodicTimer?.cancel();
    final DateTime? picked = await showDatePicker(
      context: context,
      initialDate: selectedDate,
      firstDate: DateTime(2000),
      lastDate: DateTime(2101),
    );
    if (picked != null) {
      final newDateTime = DateTime(
        picked.year,
        picked.month,
        picked.day,
        selectedDate.hour,
        selectedDate.minute,
      );
      setState(() {
        selectedDate = newDateTime;
      });
      await updateData('data_time', formatDateTime(selectedDate));
      await updateData('date_updated', 1);
    }
    periodicTimer = Timer.periodic(Duration(seconds: 10), (timer) {
      fetchData();
    });
  }

  void _selectTime() async {
    periodicTimer?.cancel();
    final TimeOfDay? picked = await showTimePicker(
      context: context,
      initialTime: TimeOfDay.fromDateTime(selectedDate),
    );
    if (picked != null) {
      final newDateTime = DateTime(
        selectedDate.year,
        selectedDate.month,
        selectedDate.day,
        picked.hour,
        picked.minute,
      );
      setState(() {
        selectedDate = newDateTime;
      });
      await updateData('data_time', formatDateTime(selectedDate));
      await updateData('date_updated', 1);
    }
    periodicTimer = Timer.periodic(Duration(seconds: 10), (timer) {
      fetchData();
    });
  }

  void _changeLanguage(String? lang) {
    if (lang != null) {
      setState(() {
        language = lang;
        Locale newLocale;
        if (lang == 'pl') {
          newLocale = Locale('pl', 'PL');
        } else {
          newLocale = Locale('en', 'US');
        }
        MyApp.setLocale(context, newLocale);
      });
    }
  }
  void _changeInstallationNumber(int? number) {
    if (number != null) {
      setState(() {
        MyApp.setInstallationNumber(context, number);
      });
      fetchData();
    }
  }

  void _toggleSectionVisibility(String section) {
    setState(() {
      switch (section) {
        case 'dateTime':
          showDateTimeSection = !showDateTimeSection;
          break;
        case 'coordinates':
          showCoordinatesSection = !showCoordinatesSection;
          break;
        case 'azimuthElevation':
          showAzimuthElevationSection = !showAzimuthElevationSection;
          break;
        case 'battery':
          showBatterySection = !showBatterySection;
          break;
        case 'actuators':
          showActuatorSection = !showActuatorSection;
          break;
      }
    });
  }

  void _showActuatorFrequencyDialog(BuildContext context) {
    var localizations = AppLocalizations.of(context)!;
    periodicTimer?.cancel();
    showDialog(
      context: context,
      builder: (BuildContext context) {
        double currentValue = actuatorFrequency.toDouble();
        return AlertDialog(
          title: Text(localizations.changeActuatorFrequency),
          content: Column(
            mainAxisSize: MainAxisSize.min,
            children: [
              StatefulBuilder(
                builder: (BuildContext context, StateSetter setState) {
                  return Slider(
                    value: currentValue,
                    min: 4,
                    max: 28,
                    divisions: 6,
                    label: '${currentValue.round()} min',
                    onChanged: (double value) {
                      setState(() {
                        currentValue = value;
                      });
                    },
                  );
                },
              ),
              Text('${currentValue.round()} min'),
            ],
          ),
          actions: <Widget>[
            ElevatedButton(
              onPressed: () {
                setState(() {
                  actuatorFrequency = currentValue.round();
                });
                updateData('actuator_frequency', actuatorFrequency);
                Navigator.of(context).pop();
                periodicTimer = Timer.periodic(Duration(seconds: 10), (timer) {
                  fetchData();
                });
              },
              child: Text(localizations.save),
            ),
          ],
        );
      },
    );
  }

  @override
  Widget build(BuildContext context) {
    var localizations = AppLocalizations.of(context)!;

    return Scaffold(
      appBar: AppBar(
        title: Text('PV Solar Tracker'),
        actions: [
          IconButton(
            icon: Icon(Icons.settings),
            onPressed: () {
              Navigator.push(
                context,
                MaterialPageRoute(
                    builder: (context) => SettingsPage(
                      serverAddress: widget.serverAddress,
                      installationNumber: widget.installationNumber,
                    )),
              );
            },
          ),
        ],
      ),
      body: Padding(
        padding: const EdgeInsets.all(8.0),
        child: SingleChildScrollView(
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              // Server status indicator
              Row(
                mainAxisAlignment: MainAxisAlignment.start,
                children: [
                  Text('Server: '),
                  Icon(
                    Icons.circle,
                    color: fetchDataSuccess ? Colors.green : Colors.red,
                    size: 12,
                  ),
                ],
              ),
              Divider(),
              // Date and Time
              _buildSectionHeader(localizations.dateAndTime, 'dateTime'),
              if (showDateTimeSection) ...[
                Row(
                  children: [
                    Expanded(child: Text(localizations.selectedDate(DateFormat.yMMMd().format(selectedDate)))),
                    ElevatedButton(
                      onPressed: _selectDate,
                      child: Text(localizations.selectDate),
                    ),
                  ],
                ),
                Row(
                  children: [
                    Expanded(child: Text(localizations.selectedTime(DateFormat.Hm().format(selectedDate)))),
                    ElevatedButton(
                      onPressed: _selectTime,
                      child: Text(localizations.selectTime),
                    ),
                  ],
                ),
                Row(
                  children: [
                    Text('${localizations.timezoneOffset}: '),
                    DropdownButton<int>(
                      value: timezoneOffset,
                      onChanged: (int? newValue) {
                        if (newValue != null) {
                          setState(() {
                            timezoneOffset = newValue;
                          });
                          updateData('timezone_offset', newValue);
                        }
                      },
                      items: List.generate(25, (index) => index - 12)
                          .map<DropdownMenuItem<int>>((int value) {
                        return DropdownMenuItem<int>(
                          value: value,
                          child: Text(
                            (value >= 0 ? "+$value" : "$value"),
                            style: TextStyle(fontSize: 16),
                          ),
                        );
                      }).toList(),
                    ),
                  ],
                ),
                Row(
                  mainAxisAlignment: MainAxisAlignment.spaceBetween,
                  children: [
                    Row(
                      children: [
                        Icon(Icons.wb_sunny),
                        SizedBox(width: 5),
                        Text('${localizations.sunrise}: ${DateFormat.Hm().format(sunrise)}'),
                      ],
                    ),
                    Row(
                      children: [
                        Icon(Icons.nights_stay),
                        SizedBox(width: 5),
                        Text('${localizations.sunset}: ${DateFormat.Hm().format(sunset)}'),
                      ],
                    ),
                  ],
                ),
              ],
              Divider(),
              // Longitude and Latitude
              _buildSectionHeader(localizations.coordinates, 'coordinates'),
              if (showCoordinatesSection) ...[
                Row(
                  mainAxisAlignment: MainAxisAlignment.spaceBetween,
                  children: [
                    Column(
                      crossAxisAlignment: CrossAxisAlignment.start,
                      children: [
                        Text('${localizations.longitude}: ${longitude.toStringAsFixed(6)}°'),
                        Text('${localizations.latitude}: ${latitude.toStringAsFixed(6)}°'),
                      ],
                    ),
                    ElevatedButton(
                      onPressed: () => _showCoordinateDialog(context),
                      child: Text(localizations.change),
                    ),
                  ],
                ),
              ],
              Divider(),
              // Azimuth and Elevation Text Fields
              _buildSectionHeader(localizations.azimuthAndElevation, 'azimuthElevation'),
              if (showAzimuthElevationSection) ...[
                Row(
                  mainAxisAlignment: MainAxisAlignment.spaceBetween,
                  children: [
                    Text(localizations.manualMode),
                    Switch(
                      value: isManualMode,
                      onChanged: (value) {
                        setState(() {
                          isManualMode = value;
                        });
                        updateData('is_manual_mode', value ? 1 : 0);
                      },
                    ),
                  ],
                ),
                Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Row(
                      mainAxisAlignment: MainAxisAlignment.spaceBetween,
                      children: [
                        Expanded(child: Text('${localizations.azimuth}: $azimuth')),
                        Expanded(child: Text(isManualMode
                            ? '${localizations.targetAzimuth}: ${targetAzimuth.toStringAsFixed(2)}'
                            : '${localizations.targetAzimuth}: ${autoAzimuth.toStringAsFixed(2)}')),
                      ],
                    ),
                    Row(
                      mainAxisAlignment: MainAxisAlignment.spaceBetween,
                      children: [
                        Expanded(child: Text('${localizations.elevation}: $elevation')),
                        Expanded(child: Text(isManualMode
                            ? '${localizations.targetElevation}: ${targetElevation.toStringAsFixed(2)}'
                            : '${localizations.targetElevation}: ${autoElevation.toStringAsFixed(2)}')),
                      ],
                    ),
                    Row(
                      mainAxisAlignment: MainAxisAlignment.end,
                      children: [
                        ElevatedButton(
                          onPressed: isManualMode ? () => _showAngleDialog(context) : null,
                          child: Text(localizations.change),
                        ),
                      ],
                    ),
                  ],
                ),
              ],
              Divider(),
              // Actuator Frequency
              _buildSectionHeader(localizations.actuators, 'actuators'),
              if (showActuatorSection) ...[
                Row(
                  mainAxisAlignment: MainAxisAlignment.spaceBetween,
                  children: [
                    Text('${localizations.actuatorFrequency}: $actuatorFrequency min'),
                    ElevatedButton(
                      onPressed: () => _showActuatorFrequencyDialog(context),
                      child: Text(localizations.change),
                    ),
                  ],
                ),
              ],
              Divider(),
              // Battery Information
              _buildSectionHeader(localizations.batteryInformation, 'battery'),
              if (showBatterySection) ...[
                Text(localizations.batteryCapacity(batteryCapacity.toStringAsFixed(2))),
                Text(localizations.batteryVoltage(batteryVoltage.toStringAsFixed(2))),
                Text(localizations.batteryChargeCurrent(batteryChargeCurrent.toStringAsFixed(2))),
                Text(localizations.pvVoltage(pvVoltage.toStringAsFixed(2))),
                Text(localizations.pvPower(pvPower.toStringAsFixed(2))),
              ],
            ],
          ),
        ),
      ),
    );
  }

  Widget _buildSectionHeader(String title, String section) {
    return GestureDetector(
      onTap: () => _toggleSectionVisibility(section),
      child: Row(
        mainAxisAlignment: MainAxisAlignment.spaceBetween,
        children: [
          Text(
            title,
            style: TextStyle(fontSize: 20, fontWeight: FontWeight.bold),
          ),
          Icon(
            showSectionIcon(section),
            size: 30,
          ),
        ],
      ),
    );
  }

  IconData showSectionIcon(String section) {
    switch (section) {
      case 'dateTime':
        return showDateTimeSection ? Icons.keyboard_arrow_up : Icons.keyboard_arrow_down;
      case 'coordinates':
        return showCoordinatesSection ? Icons.keyboard_arrow_up : Icons.keyboard_arrow_down;
      case 'azimuthElevation':
        return showAzimuthElevationSection ? Icons.keyboard_arrow_up : Icons.keyboard_arrow_down;
      case 'battery':
        return showBatterySection ? Icons.keyboard_arrow_up : Icons.keyboard_arrow_down;
      case 'actuators':
        return showActuatorSection ? Icons.keyboard_arrow_up : Icons.keyboard_arrow_down;
      default:
        return Icons.keyboard_arrow_down;
    }
  }

  void _showCoordinateDialog(BuildContext context) {
    var localizations = AppLocalizations.of(context)!;
    longitudeController.text = longitude.toStringAsFixed(6);
    latitudeController.text = latitude.toStringAsFixed(6);
    periodicTimer?.cancel();
    showDialog(
      context: context,
      builder: (BuildContext context) {
        return AlertDialog(
          title: Text(localizations.changeCoordinates),
          content: Column(
            mainAxisSize: MainAxisSize.min,
            children: [
              TextField(
                controller: longitudeController,
                keyboardType: TextInputType.numberWithOptions(signed: true, decimal: true),
                decoration: InputDecoration(labelText: localizations.longitude),
                onEditingComplete: () {
                  double newValue = double.tryParse(longitudeController.text) ?? longitude;
                  newValue = newValue.clamp(-180.0, 180.0);
                  setState(() {
                    longitude = newValue;
                    longitudeController.text = newValue.toStringAsFixed(6);
                  });
                  updateData('longitude', newValue);
                },
              ),
              TextField(
                controller: latitudeController,
                keyboardType: TextInputType.numberWithOptions(signed: true, decimal: true),
                decoration: InputDecoration(labelText: localizations.latitude),
                onEditingComplete: () {
                  double newValue = double.tryParse(latitudeController.text) ?? latitude;
                  newValue = newValue.clamp(-90.0, 90.0);
                  setState(() {
                    latitude = newValue;
                    latitudeController.text = newValue.toStringAsFixed(6);
                  });
                  updateData('latitude', newValue);
                },
              ),
            ],
          ),
          actions: <Widget>[
            ElevatedButton(
              onPressed: () {
                double newLongitude = double.parse(longitudeController.text).clamp(-180, 180);
                double newLatitude = double.parse(latitudeController.text).clamp(-90, 90);
                setState(() {
                  longitude = newLongitude;
                  latitude = newLatitude;
                });
                updateData('longitude', newLongitude);
                updateData('latitude', newLatitude);
                Navigator.of(context).pop();
                periodicTimer = Timer.periodic(Duration(seconds: 10), (timer) {
                  fetchData();
                });
              },
              child: Text(localizations.save),
            ),
          ],
        );
      },
    );
  }

  void _showAngleDialog(BuildContext context) {
    var localizations = AppLocalizations.of(context)!;
    targetAzimuthController.text = targetAzimuth.toString();
    targetElevationController.text = targetElevation.toString();
    periodicTimer?.cancel();
    showDialog(
      context: context,
      builder: (BuildContext context) {
        return AlertDialog(
          title: Text(localizations.changeAngles),
          content: Column(
            mainAxisSize: MainAxisSize.min,
            children: [
              TextField(
                controller: targetAzimuthController,
                keyboardType: TextInputType.numberWithOptions(signed: true, decimal: true),
                decoration: InputDecoration(labelText: localizations.azimuthDegrees),
                onEditingComplete: () {
                  double newValue = double.tryParse(targetAzimuthController.text) ?? targetAzimuth;
                  newValue = newValue.clamp(minAzimuth, maxAzimuth);
                  setState(() {
                    targetAzimuth = newValue;
                    targetAzimuthController.text = newValue.toString();
                  });
                  updateData('target_azimuth', newValue);
                },
              ),
              TextField(
                controller: targetElevationController,
                keyboardType: TextInputType.numberWithOptions(signed: true, decimal: true),
                decoration: InputDecoration(labelText: localizations.elevationDegrees),
                onEditingComplete: () {
                  double newValue = double.tryParse(targetElevationController.text) ?? targetElevation;
                  newValue = newValue.clamp(minElevation, maxElevation);
                  setState(() {
                    targetElevation = newValue;
                    targetElevationController.text = newValue.toString();
                  });
                  updateData('target_elevation', newValue);
                },
              ),
            ],
          ),
          actions: <Widget>[
            ElevatedButton(
              onPressed: () {
                double newAzimuth = double.parse(targetAzimuthController.text).clamp(minAzimuth, maxAzimuth);
                double newElevation = double.parse(targetElevationController.text).clamp(minElevation, maxElevation);
                setState(() {
                  targetAzimuth = newAzimuth;
                  targetElevation = newElevation;
                });
                updateData('target_azimuth', newAzimuth);
                updateData('target_elevation', newElevation);
                Navigator.of(context).pop();
                periodicTimer = Timer.periodic(Duration(seconds: 10), (timer) {
                  fetchData();
                });
              },
              child: Text(localizations.save),
            ),
          ],
        );
      },
    );
  }
}

class SettingsPage extends StatefulWidget {
  final String serverAddress;
  final int installationNumber;
  SettingsPage({required this.serverAddress, required this.installationNumber});

  @override
  _SettingsPageState createState() => _SettingsPageState();
}

class _SettingsPageState extends State<SettingsPage> {
  TextEditingController serverAddressController = TextEditingController();
  int installationNumber = 1;
  @override
  void initState() {
    super.initState();
    serverAddressController.text = widget.serverAddress;
    installationNumber = widget.installationNumber;
  }

  @override
  Widget build(BuildContext context) {
    var localizations = AppLocalizations.of(context)!;

    return Scaffold(
      appBar: AppBar(
        title: Text(localizations.settings),
      ),
      body: Padding(
        padding: const EdgeInsets.all(16.0),
        child: ListView(
          children: [
            Text(localizations.generalSettings, style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold)),
            Divider(),
            ListTile(
              title: Text(localizations.changeLanguage),
              trailing: DropdownButton<String>(
                value: Localizations.localeOf(context).languageCode,
                onChanged: (String? lang) {
                  if (lang != null) {
                    MyApp.setLocale(context, Locale(lang));
                  }
                },
                items: [
                  DropdownMenuItem(
                    value: 'en',
                    child: Text('English'),
                  ),
                  DropdownMenuItem(
                    value: 'pl',
                    child: Text('Polski'),
                  ),
                ],
              ),
            ),
            SizedBox(height: 20),
            Text(localizations.networkSettings, style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold)),
            Divider(),
            ListTile(
              title: Text(localizations.serverAddress),
              trailing: Container(
                width: 150,
                child: TextField(
                  controller: serverAddressController,
                  decoration: InputDecoration(
                    border: OutlineInputBorder(),
                  ),
                  keyboardType: TextInputType.url,
                  onChanged: (value) {
                    MyApp.setServerAddress(context, value);
                  },
                ),
              ),
            ),
            SizedBox(height: 20),
            ListTile(
              title: Text(localizations.installationNumber),
              trailing: DropdownButton<int>(
                value: installationNumber,
                onChanged: (int? newNumber) {
                  if (newNumber != null) {
                    setState(() {
                      installationNumber = newNumber;
                      MyApp.setInstallationNumber(context, newNumber);
                    });
                  }
                },
                items: [1, 2].map<DropdownMenuItem<int>>((int value) {
                  return DropdownMenuItem<int>(
                    value: value,
                    child: Text(value.toString()),
                  );
                }).toList(),
              ),
            ),
          ],
        ),
      ),
    );
  }
}
