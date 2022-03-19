#include "main.h"

#include "MotionSensors.h"

bool sendTelemetry(int totalSeen, int totalFpSeen, int totalFpQueried, int totalFpReported, int count)
{
    if (!online)
    {
        if (sendOnline())
        {
            online = true;
            reconnectTries = 0;
        }
        else
        {
            log_e("Error sending status=online");
        }
    }

    if (discovery && !sentDiscovery)
    {
        if (sendDiscoveryConnectivity() && sendTeleSensorDiscovery("Uptime", EC_DIAGNOSTIC, "{{ value_json.uptime }}", "s") && sendTeleSensorDiscovery("Free Mem", EC_DIAGNOSTIC, "{{ value_json.freeHeap }}", "bytes") && (BleFingerprintCollection::countIds.isEmpty() ? sendDeleteDiscovery("sensor", "Count") : sendTeleSensorDiscovery("Count", "", "{{ value_json.count }}", "")) && sendButtonDiscovery("Restart", EC_DIAGNOSTIC) && sendSwitchDiscovery("Status LED", EC_CONFIG) && sendNumberDiscovery("Max Distance", EC_CONFIG) && sendNumberDiscovery("Absorption", EC_CONFIG) && sendSwitchDiscovery("Active Scan", EC_CONFIG) && sendSwitchDiscovery("Auto Update", EC_CONFIG) && sendSwitchDiscovery("Arduino OTA", EC_CONFIG) && sendSwitchDiscovery("Prerelease", EC_CONFIG) && sendDeleteDiscovery("switch", "OTA Update") && Motion::SendDiscovery(doc)
#ifdef MACCHINA_A0
            && sendTeleSensorDiscovery("Battery", "", "{{ value_json.batt }}", "%")
            && sendTeleBinarySensorDiscovery("Running", "", "{{ value_json.run }}", "running")
#endif
            && sendDiscoveryBME280Temperature() && sendDiscoveryBME280Humidity() && sendDiscoveryBME280Pressure()
        )
        {
            sentDiscovery = true;
        }
        else
        {
            log_e("Error sending discovery");
        }
    }

    auto now = millis();

    if (now - lastTeleMillis < 15000)
        return false;

    lastTeleMillis = now;

    doc.clear();
    doc["ip"] = localIp;
    doc["uptime"] = getUptimeSeconds();
#ifdef FIRMWARE
    doc["firm"] = String(FIRMWARE);
#endif
    doc["rssi"] = WiFi.RSSI();
#ifdef MACCHINA_A0
    auto mv = a0_read_batt_mv();
    doc["mV"] = mv;
    bool run = (mv > 13200);
    unsigned int soc = round(-13275.04 + 2.049731 * mv - (0.00007847975 * mv) * mv);
    doc["batt"] = run ? (unsigned int)100 : max((unsigned int)0, min((unsigned int)100, soc));
    doc["run"] = run ? "ON" : "OFF";
#endif
#ifdef VERSION
    doc["ver"] = String(VERSION);
#endif

    if (!BleFingerprintCollection::countIds.isEmpty())
        doc["count"] = count;
    if (totalSeen > 0)
        doc["adverts"] = totalSeen;
    if (totalFpSeen > 0)
        doc["seen"] = totalFpSeen;
    if (totalFpQueried > 0)
        doc["queried"] = totalFpQueried;
    if (totalFpReported > 0)
        doc["reported"] = totalFpReported;

    if (teleFails > 0)
        doc["teleFails"] = teleFails;
    if (reconnectTries > 0)
        doc["reconnectTries"] = reconnectTries;

    doc["freeHeap"] = ESP.getFreeHeap();
    doc["maxAllocHeap"] = ESP.getMaxAllocHeap();
    doc["memFrag"] = 100 - (ESP.getMaxAllocHeap() * 100.0 / ESP.getFreeHeap());
    doc["resetReason"] = resetReason(rtc_get_reset_reason(0));

    char teleMessageBuffer[512];
    serializeJson(doc, teleMessageBuffer);

    for (int i = 0; i < 10; i++)
    {
        if (!publishTele || mqttClient.publish(teleTopic.c_str(), 0, false, teleMessageBuffer))
            return true;
        delay(50);
    }

    teleFails++;
    log_e("Error after 10 tries sending telemetry (%d times since boot)", teleFails);
    return false;
}

void connectToWifi()
{
    Serial.printf("Connecting to WiFi (%s)...\n", WiFi.macAddress().c_str());
    GUI::connected(false, false);

    WiFiSettings.onFailure = []()
    {
        GUI::status("WiFi Portal...");
    };
    WiFiSettings.onWaitLoop = []()
    {
        GUI::connecting();
        return 150;
    };
    WiFiSettings.onPortalWaitLoop = []()
    {
        if (getUptimeSeconds() > 600)
            ESP.restart();
    };

    GUI::connected(true, false);
#ifdef VERSION
    WiFiSettings.info("ESPresense Version: " + String(VERSION));
#endif
    room = WiFiSettings.string("room", ESPMAC, "Room");

    WiFiSettings.heading("MQTT <a href='https://espresense.com/configuration/settings#mqtt' target='_blank'>ℹ️</a>", false);
    mqttHost = WiFiSettings.string("mqtt_host", DEFAULT_MQTT_HOST, "Server");
    mqttPort = WiFiSettings.integer("mqtt_port", DEFAULT_MQTT_PORT, "Port");
    mqttUser = WiFiSettings.string("mqtt_user", DEFAULT_MQTT_USER, "Username");
    mqttPass = WiFiSettings.string("mqtt_pass", DEFAULT_MQTT_PASSWORD, "Password");
    discovery = WiFiSettings.checkbox("discovery", true, "Send to discovery topic");
    publishTele = WiFiSettings.checkbox("pub_tele", true, "Send to telemetry topic");
    publishRooms = WiFiSettings.checkbox("pub_rooms", true, "Send to rooms topic");
    publishDevices = WiFiSettings.checkbox("pub_devices", true, "Send to devices topic");

    WiFiSettings.heading("Updating <a href='https://espresense.com/configuration/settings#updating' target='_blank'>ℹ️</a>", false);
    autoUpdate = WiFiSettings.checkbox("auto_update", DEFAULT_AUTO_UPDATE, "Automatically update");
    prerelease = WiFiSettings.checkbox("prerelease", false, "Include pre-released versions in auto-update");
    arduinoOta = WiFiSettings.checkbox("arduino_ota", DEFAULT_ARDUINO_OTA, "Arduino OTA Update");

    WiFiSettings.heading("Scanning <a href='https://espresense.com/configuration/settings#scanning' target='_blank'>ℹ️</a>", false);
    activeScan = WiFiSettings.checkbox("active_scan", false, "Request scan results (usually not needed)");
    BleFingerprintCollection::knownMacs = WiFiSettings.string("known_macs", "", "Known BLE mac addresses (no colons, space seperated)");
    BleFingerprintCollection::query = WiFiSettings.string("query", DEFAULT_QUERY, "Query device ids for characteristics (eg. apple:1005:9-26)");

    WiFiSettings.heading("Counting <a href='https://espresense.com/configuration/settings#counting' target='_blank'>ℹ️</a>", false);
    BleFingerprintCollection::countIds = WiFiSettings.string("count_ids", "", "Include device ids (space seperated ids)");
    BleFingerprintCollection::countEnter = WiFiSettings.floating("count_enter", 0, 100, 2, "Start counting devices less than distance (in meters)");
    BleFingerprintCollection::countExit = WiFiSettings.floating("count_exit", 0, 100, 4, "Stop counting devices greater than distance (in meters)");
    BleFingerprintCollection::countMs = WiFiSettings.integer("count_ms", 0, 3000000, 30000, "Include devices with age less than (in ms)");

    WiFiSettings.heading("Filtering <a href='https://espresense.com/configuration/settings#filtering' target='_blank'>ℹ️</a>", false);
    if (BleFingerprintCollection::query == "1") BleFingerprintCollection::query = "apple:10"; // This is to keep query=true doing the same thing as older firmwares
    BleFingerprintCollection::include = WiFiSettings.string("include", DEFAULT_INCLUDE, "Include only sending these ids to mqtt (eg. apple:iphone10-6 apple:iphone13-2)");
    BleFingerprintCollection::exclude = WiFiSettings.string("exclude", DEFAULT_EXCLUDE, "Exclude sending these ids to mqtt (eg. exp:20 apple:iphone10-6)");
    BleFingerprintCollection::maxDistance = WiFiSettings.floating("max_dist", 0, 100, DEFAULT_MAX_DISTANCE, "Maximum distance to report (in meters)");
    BleFingerprintCollection::skipDistance = WiFiSettings.floating("skip_dist", 0, 10, DEFAULT_SKIP_DISTANCE, "Report early if beacon has moved more than this distance (in meters)");
    BleFingerprintCollection::skipMs = WiFiSettings.integer("skip_ms", 0, 3000000, DEFAULT_SKIP_MS, "Skip reporting if message age is less that this (in milliseconds)");

    WiFiSettings.heading("Calibration <a href='https://espresense.com/configuration/settings#calibration' target='_blank'>ℹ️</a>", false);
    BleFingerprintCollection::refRssi = WiFiSettings.integer("ref_rssi", -100, 100, DEFAULT_REF_RSSI, "Rssi expected from a 0dBm transmitter at 1 meter");
    BleFingerprintCollection::absorption = WiFiSettings.floating("absorption", -100, 100, DEFAULT_ABSORPTION, "Factor used to account for absorption, reflection, or diffraction");
    BleFingerprintCollection::forgetMs = WiFiSettings.integer("forget_ms", 0, 3000000, DEFAULT_FORGET_MS, "Forget beacon if not seen for (in milliseconds)");

    WiFiSettings.heading("Misc <a href='https://espresense.com/configuration/settings#misc' target='_blank'>ℹ️</a>", false);
    GUI::statusLed = WiFiSettings.checkbox("status_led", true, "Status LED");
    Motion::ConnectToWifi();

    WiFiSettings.heading("I2C Settings <a href='https://espresense.com/configuration/settings#i2c-settings' target='_blank'>ℹ️</a>", false);

    I2CDebug = WiFiSettings.checkbox("I2CDebug", false, "Debug I2C addreses. Look at the serial log to get the correct address");

    WiFiSettings.html("h4", "Bus 1:");
    I2C_Bus_1_SDA = WiFiSettings.integer("I2C_Bus_1_SDA", 0, 39, DEFAULT_I2C_BUS_1_SDA, "SDA pin (0 to disable)");
    I2C_Bus_1_SCL = WiFiSettings.integer("I2C_Bus_1_SCL", 0, 39, DEFAULT_I2C_BUS_1_SCL, "SCL pin (0 to disable)");

    WiFiSettings.html("h4", "Bus 2:");

    I2C_Bus_2_SDA = WiFiSettings.integer("I2C_Bus_2_SDA", 0, "SDA pin (0 to disable)");
    I2C_Bus_2_SCL = WiFiSettings.integer("I2C_Bus_2_SCL", 0, "SCL pin (0 to disable)");

    WiFiSettings.heading("I2C Sensors <a href='https://espresense.com/configuration/settings#i2c-sensors' target='_blank'>ℹ️</a>", false);

    WiFiSettings.html("h4", "BME280 - Weather Sensor:");
    BME280_I2c_Bus = WiFiSettings.integer("BME280_I2c_Bus", 1, 2, DEFAULT_I2C_BUS, "I2C Bus");
    BME280_I2c = WiFiSettings.string("BME280_I2c", "0x76", "I2C address (0x76 or 0x77)");

    WiFiSettings.hostname = "espresense-" + kebabify(room);

    if (!WiFiSettings.connect(true, 60))
        ESP.restart();
#ifdef FIRMWARE
    Serial.println("Firmware:     " + String(FIRMWARE));
#endif
#ifdef VERSION
    Serial.println("Version:      " + String(VERSION));
#endif
    Serial.print("IP address:   ");
    Serial.println(WiFi.localIP());
    Serial.print("DNS address:  ");
    Serial.println(WiFi.dnsIP());
    Serial.print("Hostname:     ");
    Serial.println(WiFi.getHostname());
    Serial.print("Room:         ");
    Serial.println(room);
    Serial.printf("MQTT server:  %s:%d\n", mqttHost.c_str(), mqttPort);
    Serial.printf("Max Distance: %.2f\n", BleFingerprintCollection::maxDistance);
    Motion::SerialReport();

    Serial.print("BME280_I2c Sensor: ");
    Serial.println(BME280_I2c + " on bus " + BME280_I2c_Bus);

    Serial.print("Query:        ");
    Serial.println(BleFingerprintCollection::query);
    Serial.print("Include:      ");
    Serial.println(BleFingerprintCollection::include);
    Serial.print("Exclude:      ");
    Serial.println(BleFingerprintCollection::exclude);
    Serial.print("Known Macs:   ");
    Serial.println(BleFingerprintCollection::knownMacs);
    Serial.print("Count Ids:    ");
    Serial.println(BleFingerprintCollection::countIds);

    localIp = WiFi.localIP().toString();
    id = slugify(room);
    roomsTopic = CHANNEL + "/rooms/" + id;
    statusTopic = roomsTopic + "/status";
    teleTopic = roomsTopic + "/telemetry";
    setTopic = roomsTopic + "/+/set";
}

void onMqttConnect(bool sessionPresent)
{
    xTimerStop(reconnectTimer, 0);
    mqttClient.subscribe("espresense/rooms/*/+/set", 1);
    mqttClient.subscribe(setTopic.c_str(), 1);
    GUI::connected(true, true);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason)
{
    GUI::connected(true, false);
    log_e("Disconnected from MQTT; reason %d\n", reason);
    xTimerStart(reconnectTimer, 0);
    online = false;
}

void onMqttMessage(char *topic, char *payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total)
{
    char new_payload[len + 1];
    new_payload[len] = '\0';
    strncpy(new_payload, payload, len);
    Serial.printf("%s: %s\n", topic, new_payload);

    String top = String(topic);
    String pay = String(new_payload);

    auto setPos = top.lastIndexOf("/set");
    if (setPos <= 1) return;
    auto commandPos = top.lastIndexOf("/", setPos - 1);
    if (commandPos < 0) return;
    auto command = top.substring(commandPos + 1, setPos);

    if (command == "max_distance")
    {
        BleFingerprintCollection::maxDistance = pay.toFloat();
        spurt("/max_dist", pay);
        online = false;
    }
    else if (command == "absorption")
    {
        BleFingerprintCollection::absorption = pay.toFloat();
        spurt("/absorption", pay);
        online = false;
    }
    else if (command == "active_scan")
    {
        activeScan = pay == "ON";
        spurt("/active_scan", String(activeScan));
        online = false;
    }
    else if (command == "query")
    {
        BleFingerprintCollection::query = pay;
        spurt("/query", pay);
        online = false;
    }
    else if (command == "include")
    {
        BleFingerprintCollection::include = pay;
        spurt("/include", pay);
        online = false;
    }
    else if (command == "exclude")
    {
        BleFingerprintCollection::exclude = pay;
        spurt("/exclude", pay);
        online = false;
    }
    else if (command == "known_macs")
    {
        BleFingerprintCollection::knownMacs = pay;
        spurt("/known_macs", pay);
        online = false;
    }
    else if (command == "count_ids")
    {
        BleFingerprintCollection::countIds = pay;
        spurt("/count_ids", pay);
        online = false;
    }
    else if (command == "status_led")
    {
        GUI::statusLed = pay == "ON";
        spurt("/status_led", String(GUI::statusLed));
        online = false;
    }
    else if (command == "arduino_ota")
    {
        arduinoOta = pay == "ON";
        spurt("/arduino_ota", String(arduinoOta));
        online = false;
    }
    else if (command == "auto_update")
    {
        autoUpdate = pay == "ON";
        spurt("/auto_update", String(autoUpdate));
        online = false;
    }
    else if (command == "prerelease")
    {
        prerelease = pay == "ON";
        spurt("/prerelease", String(prerelease));
        online = false;
    }
    else if (command == "restart")
    {
        ESP.restart();
    }
    else if (command == "dump_memory")
    {
        heap_caps_dump_all();
    }
}

void reconnect(TimerHandle_t xTimer)
{
    Serial.printf("%u Reconnect timer\n", xPortGetCoreID());
    if (updateInProgress) return;
    if (WiFi.isConnected() && mqttClient.connected()) return;

    if (reconnectTries++ > 50)
    {
        log_e("Too many reconnect attempts; Restarting");
        ESP.restart();
    }

    if (!WiFi.isConnected())
    {
        Serial.printf("%u Reconnecting to WiFi...\n", xPortGetCoreID());
        if (!WiFiSettings.connect(true, 60))
            ESP.restart();
    }

    Serial.printf("%u Reconnecting to MQTT...\n", xPortGetCoreID());
    mqttClient.connect();
}

void connectToMqtt()
{
    reconnectTimer = xTimerCreate("reconnectionTimer", pdMS_TO_TICKS(3000), pdTRUE, (void *)nullptr, reconnect);
    mqttClient.onConnect(onMqttConnect);
    mqttClient.onDisconnect(onMqttDisconnect);
    mqttClient.onMessage(onMqttMessage);
    mqttClient.setServer(mqttHost.c_str(), mqttPort);
    mqttClient.setWill(statusTopic.c_str(), 0, true, offline.c_str());
    mqttClient.setCredentials(mqttUser.c_str(), mqttPass.c_str());
    mqttClient.connect();
}

bool reportDevice(BleFingerprint *f)
{
    doc.clear();
    if (!f->report(&doc))
        return false;

    serializeJson(doc, buffer);
    String devicesTopic = CHANNEL + "/devices/" + f->getId() + "/" + id;

    bool p1 = false, p2 = false;
    for (int i = 0; i < 10; i++)
    {
        if (!mqttClient.connected())
            return false;

        if (!p1 && (!publishRooms || mqttClient.publish(roomsTopic.c_str(), 0, false, buffer)))
            p1 = true;

        if (!p2 && (!publishDevices || mqttClient.publish(devicesTopic.c_str(), 0, false, buffer)))
            p2 = true;

        if (p1 && p2)
            return true;
        delay(20);
    }
    teleFails++;
    return false;
}

int totalFpReported = 0;
int totalSeen = 0;
int totalFpSeen = 0;
int totalFpQueried = 0;

void reportTask(void *parameter)
{
    connectToMqtt();

    while (true)
    {
        while (updateInProgress || !mqttClient.connected())
            delay(1000);

        yield();
        auto copy = fingerprints.getCopy();

        int count = 0;
        for (auto i: copy)
            if (i->shouldCount())
                count++;

        yield();
        sendTelemetry(totalSeen, totalFpSeen, totalFpQueried, totalFpReported, count);
        yield();

        auto reported = 0;
        for (auto f : copy)
        {
            auto seen = f->getSeenCount();
            if (seen)
            {
                totalSeen += seen;
                totalFpSeen++;
            }
            if (reportDevice(f))
            {
                totalFpReported++;
                reported++;
            }
            yield();
        }
    }
}

void scanTask(void *parameter)
{
    NimBLEDevice::init("");
    for (esp_ble_power_type_t i = ESP_BLE_PWR_TYPE_CONN_HDL0; i <= ESP_BLE_PWR_TYPE_CONN_HDL8; i = esp_ble_power_type_t((int)i + 1))
        NimBLEDevice::setPower(ESP_PWR_LVL_P9, i);
    NimBLEDevice::setSecurityAuth(true, true, true);
    NimBLEDevice::setSecurityRespKey(BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID);
    NimBLEDevice::setMTU(255);

    auto pBLEScan = NimBLEDevice::getScan();
    pBLEScan->setInterval(BLE_SCAN_INTERVAL);
    pBLEScan->setWindow(BLE_SCAN_WINDOW);
    pBLEScan->setAdvertisedDeviceCallbacks(&fingerprints, true);
    pBLEScan->setActiveScan(activeScan);
    pBLEScan->setDuplicateFilter(false);
    pBLEScan->setMaxResults(0);
    if (!pBLEScan->start(0, nullptr, false))
        log_e("Error starting continuous ble scan");

    while (true)
    {
        for (auto f : *fingerprints.getNative())
            if (f->query())
                totalFpQueried++;

        if (!pBLEScan->isScanning())
        {
            if (!pBLEScan->start(0, nullptr, true))
                log_e("Error re-starting continuous ble scan");
            delay(3000); // If we stopped scanning, don't query for 3 seconds in order for us to catch any missed broadcasts
        }
        else
        {
            delay(100);
        }
    }
}

void setup()
{
    GUI::setup();

#ifdef FAST_MONITOR
    Serial.begin(1500000);
#else
    Serial.begin(115200);
#endif
    Serial.setDebugOutput(true);

#ifdef VERBOSE
    esp_log_level_set("*", ESP_LOG_DEBUG);
#else
    esp_log_level_set("*", ESP_LOG_ERROR);
#endif

    spiffsInit();
    connectToWifi();
#if NTP
    setClock();
#endif
    Motion::Setup();
#if MACCHINA_A0
    pinMode(GPIO_NUM_35, INPUT);
#endif
    if (I2C_Bus_1_SDA != 0 && I2C_Bus_1_SDA != 0) {
        Wire.begin(I2C_Bus_1_SDA, I2C_Bus_1_SCL);
        I2C_Bus_1_Enabled = true;
        Serial.println("\nInitialized I2C Bus 1");
    }

    if (I2C_Bus_2_SDA != 0 && I2C_Bus_2_SDA != 0) {
        Wire1.begin(I2C_Bus_2_SDA, I2C_Bus_2_SCL);
        I2C_Bus_2_Enabled = true;
        Serial.println("\nInitialized I2C Bus 2");
    }

    if (I2CDebug)
    {
        Serial.println("\nI2C Scanner");
    }

    xTaskCreatePinnedToCore(scanTask, "scanTask", 7168, nullptr, 2, &scanTaskHandle, CONFIG_BT_NIMBLE_PINNED_TO_CORE);
    xTaskCreatePinnedToCore(reportTask, "reportTask", 7168, nullptr, 1, &reportTaskHandle, 1);
    configureOTA();
}

void bme280Loop() {
    if (I2C_Bus_1_Enabled || I2C_Bus_2_Enabled) {

        // if (!BME280_status) {
        //     Serial.println("[BME280] Couldn't find a sensor, check your wiring and I2C address!");
        // }


        // BME280.setSampling(Adafruit_BME280::MODE_NORMAL,
        //                 Adafruit_BME280::SAMPLING_X16,  // Temperature
        //                 Adafruit_BME280::SAMPLING_X16,  // Pressure
        //                 Adafruit_BME280::SAMPLING_X16,  // Humidity
        //                 Adafruit_BME280::FILTER_X16,
        //                 //Adafruit_BME280::FILTER_OFF,
        //                 Adafruit_BME280::STANDBY_MS_1000
        // );


        // float temperature = BME280.readTemperature();
        // float humidity = BME280.readHumidity();
        // float pressure = BME280.readPressure() / 100.0F;

        if (millis() - bme280PreviousMillis >= sensorInterval) {
            uint8_t i2c_addr = 0xFF;
            TwoWire *i2c_wire = nullptr;

            if (BME280_I2c == "0x76") {
                i2c_addr = 0x76;
            }
            else if (BME280_I2c == "0x77") {
                i2c_addr = 0x77;
            }
            else {
                return;
            }

            if (BME280_I2c_Bus == 1) {
                i2c_wire = &Wire;
            }
            else if (BME280_I2c_Bus == 2) {
                i2c_wire = &Wire1;
            }
            else {
                return;
            }

            auto retries = 0;
            while (retries < 10) {
                if (bme.begin(i2c_addr, i2c_wire)) {
                    break;
                }
                retries++;
                delay(100);
            }

            auto fd = bme.readAllSensors();

            mqttClient.publish((roomsTopic + "/bme280_temperature").c_str(), 0, 1, fd.getTempStr().c_str());
            mqttClient.publish((roomsTopic + "/bme280_humidity").c_str(), 0, 1, fd.getHumStr().c_str());
            mqttClient.publish((roomsTopic + "/bme280_pressure").c_str(), 0, 1, fd.getPressStr().c_str());

            bme280PreviousMillis = millis();
        }
    }
}

void loop()
{
    uint32_t freeHeap = ESP.getFreeHeap();
    if (arduinoOta && freeHeap > 4096)
        ArduinoOTA.handle();
    if (freeHeap < 10000) Serial.printf("Low memory: %u bytes free", freeHeap);
    // firmwareUpdate();
    Motion::Loop(mqttClient);
    bme280Loop();
    WiFiSettings.httpLoop();
}
