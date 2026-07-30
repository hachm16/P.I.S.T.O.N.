#include <Arduino.h>
#include <driver/twai.h>
#include <math.h>


// ------------------------------------------------------------
// CAN configuration
// ------------------------------------------------------------

// Compile-time pin constants compatible with the ESP32 TWAI controller
constexpr gpio_num_t CAN_RX_PIN = GPIO_NUM_16;
constexpr gpio_num_t CAN_TX_PIN = GPIO_NUM_17;


// Assumes standard 11-bit OBD-II CAN operating at 500 kbit/s

// Broadcast CAN ID used to send an OBD-II request to all available ECUs
constexpr uint32_t OBD_FUNCTIONAL_REQUEST_ID = 0x7DF;

// Lowest standard CAN ID from which an ECU may send an OBD-II response
constexpr uint32_t OBD_RESPONSE_MIN_ID = 0x7E8;

// Highest standard CAN ID from which an ECU may send an OBD-II response
constexpr uint32_t OBD_RESPONSE_MAX_ID = 0x7EF;


// ------------------------------------------------------------
// Sampling configuration
// ------------------------------------------------------------

// Twenty samples are collected for each case, matching the ML training data
constexpr size_t CASE_SIZE = 20;

// Attempt to start one complete sample collection every second
constexpr uint32_t SAMPLE_PERIOD_MS = 1000;

// Maximum time to wait for the vehicle to respond to one PID request
constexpr uint32_t PID_TIMEOUT_MS = 80;


// ------------------------------------------------------------
// Vehicle sample structure
// ------------------------------------------------------------

// Stores one complete row of vehicle data
//
// Each value has a corresponding bool variable that tells us whether
// the vehicle successfully returned a valid response for that value
struct VehicleSample
{
    uint32_t timestampMs;

    float engineLoad;
    bool engineLoadValid;

    float stftB1;
    bool stftB1Valid;

    float ltftB1;
    bool ltftB1Valid;

    float stftB2;
    bool stftB2Valid;

    float ltftB2;
    bool ltftB2Valid;

    float rpm;
    bool rpmValid;

    // O2 Bank 1 Sensor 1 = upstream O2 sensor
    float o2B1S1Voltage;
    bool o2B1S1VoltageValid;

    // O2 Bank 1 Sensor 2 = downstream O2 sensor
    float o2B1S2Voltage;
    bool o2B1S2VoltageValid;

    // equivalence ratio for Bank 1 Sensor 1
    float o2B1S1EquivalenceRatio;
    bool o2B1S1EquivalenceRatioValid;

    // O2 Bank 2 Sensor 1 = upstream O2 sensor for Bank 2 (if)
    float o2B2S1Voltage;
    bool o2B2S1VoltageValid;

    // O2 Bank 2 Sensor 2 = downstream O2 sensor for Bank 2 (if)
    float o2B2S2Voltage;
    bool o2B2S2VoltageValid;

    // equivalence ratio for Bank 2 Sensor 1 (if)
    float o2B2S1EquivalenceRatio;
    bool o2B2S1EquivalenceRatioValid;

    // Voltage seen by ECU's
    float controlModuleVoltage;
    bool controlModuleVoltageValid;
};


// Stores one complete non-overlapping twenty-sample case
VehicleSample caseSamples[CASE_SIZE];

// Index of the next position in the twenty-sample case array
size_t sampleIndex = 0;

// Time at which the previous complete sample collection began
uint32_t previousSampleTime = 0;

// Response CAN ID of the ECU selected for OBD-II communication
//
// The value starts at zero because an ECU has not been selected yet
uint32_t selectedEcuResponseId = 0;


// ------------------------------------------------------------
// Sample initialization
// ------------------------------------------------------------

// Resets every value and validity flag before collecting a new sample
void initializeSample(VehicleSample& sample)
{
    sample.timestampMs = millis();

    sample.engineLoad = NAN;
    sample.engineLoadValid = false;

    sample.stftB1 = NAN;
    sample.stftB1Valid = false;

    sample.ltftB1 = NAN;
    sample.ltftB1Valid = false;

    sample.stftB2 = NAN;
    sample.stftB2Valid = false;

    sample.ltftB2 = NAN;
    sample.ltftB2Valid = false;

    sample.rpm = NAN;
    sample.rpmValid = false;

    sample.o2B1S1Voltage = NAN;
    sample.o2B1S1VoltageValid = false;

    sample.o2B1S2Voltage = NAN;
    sample.o2B1S2VoltageValid = false;

    sample.o2B1S1EquivalenceRatio = NAN;
    sample.o2B1S1EquivalenceRatioValid = false;

    sample.o2B2S1Voltage = NAN;
    sample.o2B2S1VoltageValid = false;

    sample.o2B2S2Voltage = NAN;
    sample.o2B2S2VoltageValid = false;

    sample.o2B2S1EquivalenceRatio = NAN;
    sample.o2B2S1EquivalenceRatioValid = false;

    sample.controlModuleVoltage = NAN;
    sample.controlModuleVoltageValid = false;
}


// ------------------------------------------------------------
// CAN receive queue handling
// ------------------------------------------------------------

// Removes any old CAN messages from the receive queue before sending
// a new PID request
//
// This helps prevent an older response from being mistaken for the
// response to the current request
void clearReceiveQueue()
{
    twai_message_t message = {};

    while (twai_receive(&message, 0) == ESP_OK)
    {
        // Continue removing messages until the receive queue is empty
    }
}


// ------------------------------------------------------------
// OBD-II PID request function
// ------------------------------------------------------------

// Sends a Mode 01 current (live) data PID request and waits for a response
//
// pid:
// The requested PID, 
//
// outputData:
// 
//
// requiredDataBytes:
//
// Returns true when a valid response is received
// Returns false when the request times out/fails
bool requestMode01Pid(
    uint8_t pid, // which parameter we're grabbing (e.g. 0x0C for RPM)
    uint8_t* outputData, // Where returned data bytes will be stored
    uint8_t requiredDataBytes) // # of PID data bytes expected
{
    clearReceiveQueue(); // empty queue

    twai_message_t request = {};

    request.identifier = OBD_FUNCTIONAL_REQUEST_ID;

    // standard 11-bit CAN identifier
    request.extd = 0;

    // normal data frame, not a transmission request
    request.rtr = 0;

    // OBD-II CAN frames normally contain eight bytes
    request.data_length_code = 8;

    // ISO-TP single-frame payload:
   
    request.data[0] = 0x02; // Number of meaningful bytes
    request.data[1] = 0x01; // Mode 1, live data
    request.data[2] = pid; // Requested PID
    request.data[3] = 0x00; //Below not used, set to 0
    request.data[4] = 0x00;
    request.data[5] = 0x00;
    request.data[6] = 0x00;
    request.data[7] = 0x00;

    // Send PID request onto vehicle CAN bus
    if (twai_transmit(&request, pdMS_TO_TICKS(20)) != ESP_OK)
    {
        return false;
    }

    uint32_t requestStartTime = millis();

    // Continue checking received messages until the PID timeout expires
    while (millis() - requestStartTime < PID_TIMEOUT_MS)
    {
        twai_message_t response = {};

        // Wait up to ten milliseconds for the next received CAN message
        if (twai_receive(&response, pdMS_TO_TICKS(10)) != ESP_OK)
        {
            continue;
        }

        // Ignore extended CAN frames and remote request frames
        if (response.extd || response.rtr)
        {
            continue;
        }

        // Ignore CAN identifiers outside the standard OBD-II response range
        if (
            response.identifier < OBD_RESPONSE_MIN_ID ||
            response.identifier > OBD_RESPONSE_MAX_ID)
        {
            continue;
        }

        // After an ECU has been selected, ignore responses from other ECUs
        if (
            selectedEcuResponseId != 0 && response.identifier != selectedEcuResponseId)
        {
            continue;
        }

        // The response must contain:
        // ISO-TP byte, response mode, PID and requested data bytes
        if (response.data_length_code < 3 + requiredDataBytes)
        {
            continue;
        }

        // The upper four bits being zero indicates an ISO-TP single frame
        if ((response.data[0] & 0xF0) != 0x00)
        {
            continue;
        }

        // Lower four bits contain the ISO-TP payload length
        uint8_t payloadLength = response.data[0] & 0x0F;

        // Payload contains response mode, PID and returned data bytes
        if (payloadLength < 2 + requiredDataBytes)
        {
            continue;
        }

        // A Mode 01 request returns response mode 0x41
        //
        // The returned PID must also match the requested PID
        if (
            response.data[1] != 0x41 ||
            response.data[2] != pid)
        {
            continue;
        }

        // The first ECU that returns a valid response is selected
        if (selectedEcuResponseId == 0)
        {
            selectedEcuResponseId = response.identifier;

            Serial.printf(
                "Selected ECU response ID: 0x%03lX\n",
                static_cast<unsigned long>(selectedEcuResponseId));
        }

        // Copy the returned PID data bytes into the output array
        for (uint8_t i = 0; i < requiredDataBytes; i++)
        {
            outputData[i] = response.data[3 + i];
        }

        return true;
    }

    return false;
}


// ------------------------------------------------------------
// OBD-II value decoding functions
// ------------------------------------------------------------

// PID 04:
// Converts one raw byte into calculated engine load percentage
float decodeEngineLoad(uint8_t rawValue)
{
    return rawValue * 100.0f / 255.0f;
}


// PIDs 06 through 09:
// Converts one raw byte into short-term or long-term fuel trim percentage
float decodeFuelTrim(uint8_t rawValue)
{
    return rawValue * 100.0f / 128.0f - 100.0f;
}


// PID 0C:
// Combines two raw bytes and converts them into engine RPM
float decodeRpm(uint8_t byteA, uint8_t byteB)
{
    uint16_t rawValue =
        (static_cast<uint16_t>(byteA) << 8) |
        byteB;

    return rawValue / 4.0f;
}


// PIDs 14 through 1B:
// Converts the first returned byte into narrowband O2 sensor voltage
float decodeNarrowbandO2Voltage(uint8_t rawValue)
{
    return rawValue / 200.0f;
}


// PIDs 24 through 2B:
// Converts the first two returned bytes into wideband equivalence ratio
float decodeWidebandEquivalenceRatio(
    uint8_t byteA,
    uint8_t byteB)
{
    uint16_t rawValue =
        (static_cast<uint16_t>(byteA) << 8) |
        byteB;

    return rawValue * 2.0f / 65536.0f;
}


// PIDs 24 through 2B:
// Converts the third and fourth returned bytes into wideband O2 voltage
float decodeWidebandVoltage(
    uint8_t byteC,
    uint8_t byteD)
{
    uint16_t rawValue =
        (static_cast<uint16_t>(byteC) << 8) |
        byteD;

    return rawValue * 8.0f / 65536.0f;
}


// PID 42:
// Converts two raw bytes into control-module voltage
float decodeControlModuleVoltage(
    uint8_t byteA,
    uint8_t byteB)
{
    uint16_t rawValue =
        (static_cast<uint16_t>(byteA) << 8) |
        byteB;

    return rawValue / 1000.0f;
}


// ------------------------------------------------------------
// Complete sample collection
// ------------------------------------------------------------

// Requests all required PIDs and stores their decoded values in one
// VehicleSample structure
void collectOneSample(VehicleSample& sample)
{
    initializeSample(sample);

    // Temporary array used to hold up to four returned PID data bytes
    uint8_t data[4] = {};


    // PID 04: Calculated engine load
    if (requestMode01Pid(0x04, data, 1))
    {
        sample.engineLoad = decodeEngineLoad(data[0]);
        sample.engineLoadValid = true;
    }


    // PID 06: Short-term fuel trim, Bank 1
    if (requestMode01Pid(0x06, data, 1))
    {
        sample.stftB1 = decodeFuelTrim(data[0]);
        sample.stftB1Valid = true;
    }


    // PID 07: Long-term fuel trim, Bank 1
    if (requestMode01Pid(0x07, data, 1))
    {
        sample.ltftB1 = decodeFuelTrim(data[0]);
        sample.ltftB1Valid = true;
    }


    // PID 08: Short-term fuel trim, Bank 2
    //
    // Four-cylinder engines normally do not have Bank 2, so this PID
    // may be unsupported on many vehicles
    if (requestMode01Pid(0x08, data, 1))
    {
        sample.stftB2 = decodeFuelTrim(data[0]);
        sample.stftB2Valid = true;
    }


    // PID 09: Long-term fuel trim, Bank 2
    if (requestMode01Pid(0x09, data, 1))
    {
        sample.ltftB2 = decodeFuelTrim(data[0]);
        sample.ltftB2Valid = true;
    }


    // PID 0C: Engine RPM
    if (requestMode01Pid(0x0C, data, 2))
    {
        sample.rpm = decodeRpm(data[0], data[1]);
        sample.rpmValid = true;
    }


    // PID 14: O2 Sensor 1
    //
    // In the normal two-bank layout this is Bank 1 Sensor 1,
    // which is the upstream O2 sensor
    if (requestMode01Pid(0x14, data, 2))
    {
        sample.o2B1S1Voltage =
            decodeNarrowbandO2Voltage(data[0]);

        sample.o2B1S1VoltageValid = true;
    }


    // PID 15: O2 Sensor 2
    //
    // In the normal two-bank layout this is Bank 1 Sensor 2,
    // which is the downstream O2 sensor
    if (requestMode01Pid(0x15, data, 2))
    {
        sample.o2B1S2Voltage =
            decodeNarrowbandO2Voltage(data[0]);

        sample.o2B1S2VoltageValid = true;
    }


    // PID 18: O2 Sensor 5
    //
    // In the normal two-bank layout this is Bank 2 Sensor 1,
    // which is the upstream O2 sensor for Bank 2
    if (requestMode01Pid(0x18, data, 2))
    {
        sample.o2B2S1Voltage =
            decodeNarrowbandO2Voltage(data[0]);

        sample.o2B2S1VoltageValid = true;
    }


    // PID 19: O2 Sensor 6
    //
    // In the normal two-bank layout this is Bank 2 Sensor 2,
    // which is the downstream O2 sensor for Bank 2
    if (requestMode01Pid(0x19, data, 2))
    {
        sample.o2B2S2Voltage =
            decodeNarrowbandO2Voltage(data[0]);

        sample.o2B2S2VoltageValid = true;
    }


    // PID 24: Wideband O2 Sensor 1
    //
    // Normally represents Bank 1 Sensor 1 in a two-bank layout
    if (requestMode01Pid(0x24, data, 4))
    {
        sample.o2B1S1EquivalenceRatio =
            decodeWidebandEquivalenceRatio(
                data[0],
                data[1]);

        sample.o2B1S1EquivalenceRatioValid = true;

        // Use the wideband voltage when narrowband PID 14 was unavailable
        if (!sample.o2B1S1VoltageValid)
        {
            sample.o2B1S1Voltage =
                decodeWidebandVoltage(
                    data[2],
                    data[3]);

            sample.o2B1S1VoltageValid = true;
        }
    }


    // PID 25: Wideband O2 Sensor 2
    //
    // Used as a fallback for Bank 1 downstream voltage when PID 15
    // was unavailable
    if (
        !sample.o2B1S2VoltageValid &&
        requestMode01Pid(0x25, data, 4))
    {
        sample.o2B1S2Voltage =
            decodeWidebandVoltage(
                data[2],
                data[3]);

        sample.o2B1S2VoltageValid = true;
    }


    // PID 28: Wideband O2 Sensor 5
    //
    // Normally represents Bank 2 Sensor 1 in a two-bank layout
    if (requestMode01Pid(0x28, data, 4))
    {
        sample.o2B2S1EquivalenceRatio =
            decodeWidebandEquivalenceRatio(
                data[0],
                data[1]);

        sample.o2B2S1EquivalenceRatioValid = true;

        // Use wideband voltage when narrowband PID 18 was unavailable
        if (!sample.o2B2S1VoltageValid)
        {
            sample.o2B2S1Voltage =
                decodeWidebandVoltage(
                    data[2],
                    data[3]);

            sample.o2B2S1VoltageValid = true;
        }
    }


    // PID 29: Wideband O2 Sensor 6
    //
    // Used as a fallback for Bank 2 downstream voltage when PID 19
    // was unavailable
    if (
        !sample.o2B2S2VoltageValid &&
        requestMode01Pid(0x29, data, 4))
    {
        sample.o2B2S2Voltage =
            decodeWidebandVoltage(
                data[2],
                data[3]);

        sample.o2B2S2VoltageValid = true;
    }


    // PID 42: Control-module voltage
    if (requestMode01Pid(0x42, data, 2))
    {
        sample.controlModuleVoltage =
            decodeControlModuleVoltage(
                data[0],
                data[1]);

        sample.controlModuleVoltageValid = true;
    }
}


// ------------------------------------------------------------
// Serial output
// ------------------------------------------------------------

// Prints a floating-point value when it is valid
//
// Prints NA when the vehicle did not return a valid value
void printValue(float value, bool valid)
{
    if (valid)
    {
        Serial.print(value, 4);
    }
    else
    {
        Serial.print("NA");
    }
}


// Prints one complete sample in CSV format
void printSample(
    const VehicleSample& sample,
    size_t index)
{
    Serial.print(index);
    Serial.print(',');

    Serial.print(sample.timestampMs);
    Serial.print(',');

    printValue(
        sample.engineLoad,
        sample.engineLoadValid);
    Serial.print(',');

    printValue(
        sample.stftB1,
        sample.stftB1Valid);
    Serial.print(',');

    printValue(
        sample.ltftB1,
        sample.ltftB1Valid);
    Serial.print(',');

    printValue(
        sample.stftB2,
        sample.stftB2Valid);
    Serial.print(',');

    printValue(
        sample.ltftB2,
        sample.ltftB2Valid);
    Serial.print(',');

    printValue(
        sample.rpm,
        sample.rpmValid);
    Serial.print(',');

    printValue(
        sample.o2B1S1Voltage,
        sample.o2B1S1VoltageValid);
    Serial.print(',');

    printValue(
        sample.o2B1S2Voltage,
        sample.o2B1S2VoltageValid);
    Serial.print(',');

    printValue(
        sample.o2B1S1EquivalenceRatio,
        sample.o2B1S1EquivalenceRatioValid);
    Serial.print(',');

    printValue(
        sample.o2B2S1Voltage,
        sample.o2B2S1VoltageValid);
    Serial.print(',');

    printValue(
        sample.o2B2S2Voltage,
        sample.o2B2S2VoltageValid);
    Serial.print(',');

    printValue(
        sample.o2B2S1EquivalenceRatio,
        sample.o2B2S1EquivalenceRatioValid);
    Serial.print(',');

    printValue(
        sample.controlModuleVoltage,
        sample.controlModuleVoltageValid);

    Serial.println();
}


// ------------------------------------------------------------
// Completed case processing
// ------------------------------------------------------------

// Called after all twenty samples have been collected
//
// Feature calculations and ML inference will be added here later
void processCompletedCase(
    const VehicleSample samples[CASE_SIZE])
{
    Serial.println();
    Serial.println("=== 20-SAMPLE CASE COMPLETE ===");

    // Later steps:
    //
    // 1. Count valid and invalid values for every parameter
    // 2. Calculate mean, standard deviation, minimum and maximum
    // 3. Calculate fuel-trim and catalyst features
    // 4. Create the model feature arrays
    // 5. Run the three subsystem models

    Serial.println("Twenty samples were successfully stored.");
    Serial.println("Starting a new non-overlapping case.");
    Serial.println("=================================");
    Serial.println();

    // The samples parameter is currently unused
    //
    // This line prevents a compiler warning until processing is added
    (void)samples;
}


// ------------------------------------------------------------
// TWAI controller initialization
// ------------------------------------------------------------

// Installs and starts the ESP32 TWAI controller at 500 kbit/s
bool initializeCan()
{
    twai_general_config_t generalConfig =
        TWAI_GENERAL_CONFIG_DEFAULT(
            CAN_TX_PIN,
            CAN_RX_PIN,
            TWAI_MODE_NORMAL);

    // Number of outgoing CAN messages that can wait in the transmit queue
    generalConfig.tx_queue_len = 10;

    // Number of incoming CAN messages that can wait in the receive queue
    generalConfig.rx_queue_len = 40;

    // Configure the CAN bus for 500 kbit/s
    twai_timing_config_t timingConfig =
        TWAI_TIMING_CONFIG_500KBITS();

    // Initially accept every CAN message
    //
    // The request function later checks the identifier and response contents
    twai_filter_config_t filterConfig =
        TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // Install the TWAI driver
    if (
        twai_driver_install(
            &generalConfig,
            &timingConfig,
            &filterConfig) != ESP_OK)
    {
        Serial.println("Failed to install the TWAI driver.");
        return false;
    }

    // Start CAN communication
    if (twai_start() != ESP_OK)
    {
        Serial.println("Failed to start the TWAI driver.");

        twai_driver_uninstall();
        return false;
    }

    Serial.println("TWAI started at 500 kbit/s.");
    return true;
}


// ------------------------------------------------------------
// Arduino setup
// ------------------------------------------------------------

void setup()
{
    Serial.begin(115200);
    delay(1500);

    Serial.println();
    Serial.println("=== P.I.S.T.O.N. OBD-II Collector ===");

    if (!initializeCan())
    {
        // Stop the program when the CAN controller fails to initialize
        while (true)
        {
            delay(1000);
        }
    }

    // CSV column titles for the serial monitor output
    Serial.println(
        "index,time_ms,engine_load,"
        "stft_b1,ltft_b1,stft_b2,ltft_b2,rpm,"
        "o2_b1s1_voltage,o2_b1s2_voltage,o2_b1s1_equiv,"
        "o2_b2s1_voltage,o2_b2s2_voltage,o2_b2s1_equiv,"
        "control_module_voltage");

    // Allows the first sample to begin immediately
    previousSampleTime = millis() - SAMPLE_PERIOD_MS;
}


// ------------------------------------------------------------
// Arduino main loop
// ------------------------------------------------------------

void loop()
{
    uint32_t currentTime = millis();

    // Do nothing until the next sample period begins
    if (currentTime - previousSampleTime < SAMPLE_PERIOD_MS)
    {
        delay(1);
        return;
    }

    // Record when this complete sample collection began
    previousSampleTime = currentTime;

    // Select the next location in the twenty-sample case array
    VehicleSample& currentSample =
        caseSamples[sampleIndex];

    // Request all required PIDs and store one complete sample
    collectOneSample(currentSample);

    // Print the completed sample to the serial monitor
    printSample(currentSample, sampleIndex);

    // Move to the next position in the case array
    sampleIndex++;

    // Process the case after all twenty samples have been collected
    if (sampleIndex >= CASE_SIZE)
    {
        processCompletedCase(caseSamples);

        // Reset to the beginning of the array for a new
        // non-overlapping twenty-sample case
        sampleIndex = 0;
    }
}
