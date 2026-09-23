#include <Arduino.h>
#include <driver/twai.h>
#include <math.h>
#include <ArduinoJson.h>

// BT Libraries
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <atomic>

// Our ML algs
#include "piston_iforest_runtime.h"
#include "piston_fuel_model.h"
#include "piston_catalyst_model.h"
#include "piston_charging_model.h"
 

// CAN configuration
// ------------------------------------------------------------

// Compile-time pin constants compatible with the ESP32 TWAI controller
constexpr gpio_num_t CAN_RX_PIN = GPIO_NUM_16;
constexpr gpio_num_t CAN_TX_PIN = GPIO_NUM_17;

// BLE configuration
// ------------------------------------------------------------

constexpr char BLE_DEVICE_NAME[] = "P.I.S.T.O.N.";

constexpr char BLE_SERVICE_UUID[] = "6c31c5f0-7d91-4d52-9f7b-23d2c7059a10";

constexpr char BLE_TX_CHARACTERISTIC_UUID[] = "6c31c5f1-7d91-4d52-9f7b-23d2c7059a10";

constexpr char BLE_RX_CHARACTERISTIC_UUID[] = "6c31c5f2-7d91-4d52-9f7b-23d2c7059a10";

// Assumes standard 11-bit OBD-II CAN operating at 500 kbit/s

// Broadcast CAN ID used to send an OBD-II request to all available ECUs
constexpr uint32_t OBD_FUNCTIONAL_REQUEST_ID = 0x7DF;

// Lowest standard CAN ID from which an ECU may send an OBD-II response
constexpr uint32_t OBD_RESPONSE_MIN_ID = 0x7E8;

// Highest standard CAN ID from which an ECU may send an OBD-II response
constexpr uint32_t OBD_RESPONSE_MAX_ID = 0x7EF;


 
// Sampling configuration
// ------------------------------------------------------------

// Twenty samples are collected for each case, matching the ML training data
constexpr size_t CASE_SIZE = 20;

// Attempt to start one complete sample collection every second
constexpr uint32_t SAMPLE_PERIOD_MS = 1000;

// Maximum time to wait for the vehicle to respond to one PID request
constexpr uint32_t PID_TIMEOUT_MS = 80;

// Parameter valid when at least 15 of 20 readings valid
constexpr size_t MIN_VALID_SAMPLES = 15;

constexpr size_t MAX_DTC_CODES_PER_MODE = 32;

constexpr size_t MAX_DTC_RESPONSE_BYTES = 128;

constexpr uint32_t DTC_RESPONSE_TIMEOUT_MS = 500;

constexpr size_t VIN_LENGTH = 17;
constexpr size_t VIN_RESPONSE_BUFFER_SIZE = 32;
constexpr uint32_t VIN_RESPONSE_TIMEOUT_MS = 1000;

// Check O2 correlation from lag -3 to +3
constexpr int MAX_O2_CORRELATION_LAG = 3;

// At least 8 valid O2 pairs needed for correlation
constexpr size_t MIN_O2_CORRELATION_PAIRS = 8;

// Prevents division by zero in catalyst calculations
constexpr float CATALYST_EPSILON = 0.000001f;
 


// Action level thresholds
// ------------------------------------------------------------

// Fuel
constexpr float FUEL_INSPECT_SCORE_THRESHOLD = 0.18f;

constexpr float FUEL_KEEP_EYE_TRIM_THRESHOLD = 10.0f;
constexpr float FUEL_INSPECT_TRIM_THRESHOLD = 18.0f;


// Catalyst
constexpr float CATALYST_INSPECT_SCORE_THRESHOLD = 0.18f;

constexpr float CATALYST_STRONG_CORRELATION_THRESHOLD = 0.90f;
constexpr uint8_t CATALYST_STRONG_REQUIRED_CONSECUTIVE_CASES = 3;


// Mild charging behavior
constexpr uint8_t CHARGING_KEEP_LOW_COUNT = 2;
constexpr uint8_t CHARGING_KEEP_HIGH_COUNT = 3;

constexpr float CHARGING_KEEP_MAX_STEP = 0.80f;
constexpr float CHARGING_KEEP_STEP_STD = 0.30f;

// Strong charging behavior
constexpr float CHARGING_INSPECT_LOW_MEAN = 13.0f;
constexpr float CHARGING_INSPECT_HIGH_MEAN = 14.8f;

constexpr uint8_t CHARGING_INSPECT_LOW_COUNT = 10;
constexpr uint8_t CHARGING_INSPECT_HIGH_COUNT = 5;

constexpr float CHARGING_INSPECT_MAX_STEP = 1.00f;
constexpr float CHARGING_INSPECT_STEP_STD = 0.45f;

constexpr uint8_t CHARGING_INSPECT_REQUIRED_CONSECUTIVE_CASES = 2;


// RPM at or below this value is treated as an engine-stop or cranking state.
// Normal charging-system voltage limits are not evaluated while the engine
// is below this speed because the alternator is not in normal operation.
constexpr float CHARGING_ENGINE_STOP_RPM_THRESHOLD = 300.0f;

// After RPM returns above the engine-stop threshold, ignore the restart
// sample and the following sample from charging diagnostic calculations.
// With the current approximately one-second sampling period, this suppresses
// roughly the first two seconds of the restart voltage transient.
constexpr uint8_t CHARGING_RESTART_SUPPRESSION_SAMPLES = 2;


// Vehicle sample structure
// ------------------------------------------------------------

// Stores one complete row of vehicle data
// Each value has a corresponding bool variable that tells us whether the vehicle successfully returned a valid response for that value
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

    // O2 Bank 2 Sensor 1 = upstream O2 sensor (if available)
    float o2B2S1Voltage;
    bool o2B2S1VoltageValid;

    // O2 Bank 2 Sensor 2 = downstream O2 sensor (if available)
    float o2B2S2Voltage;
    bool o2B2S2VoltageValid;

    // equivalence ratio for Bank 2 Sensor 1 (if available)
    float o2B2S1EquivalenceRatio;
    bool o2B2S1EquivalenceRatioValid;

    // Voltage seen by the ECU
    float controlModuleVoltage;
    bool controlModuleVoltageValid;
};


 
// Feature structures
// ------------------------------------------------------------

// Statistics calculated from one parameter collected
struct ValueStatistics
{
    bool available;
    size_t validCount;

    float mean;
    float standardDeviation;
    float minimum;
    float maximum;
    float range;
};


// Stores the 9 fuel model input floats
struct FuelCaseFeatures
{
    bool ready;

    // Zero means no valid bank was available
    // One means Bank 1 was selected
    // Two means Bank 2 was selected
    uint8_t selectedBank;

    float values[9];
};


// Stores the 5 catalyst model inputs
struct CatalystCaseFeatures
{
    bool ready;

    // Zero means no valid bank was available
    // One means Bank 1 was selected
    // Two means Bank 2 was selected
    uint8_t selectedBank;

    float values[5];

    float maximumLaggedCorrelation;
    bool maximumLaggedCorrelationValid;
};


// Stores the 7 charging model inputs
struct ChargingCaseFeatures
{
    bool ready;
    float values[7];
};

// Stores charging measurements that are safe to use for the
// rule-based diagnostic logic after engine-stop and restart
// samples have been removed.
struct ChargingDiagnosticContext
{
    bool ready;

    bool engineStopDetected;
    bool engineRestartDetected;

    size_t evaluatedSampleCount;

    float meanVoltage;
    float belowChargingCount;
    float aboveChargingCount;
    float maximumAbsoluteStep;
    float stepStandardDeviation;
};

// Stores catalyst calculations for one bank
struct CatalystBankFeatures
{
    bool available;

    // True means upstream voltage is used
    // False means equivalence ratio is used
    bool useUpstreamVoltage;

    float downstreamStandardDeviation;
    float downstreamRange;
    float downstreamMeanAbsoluteStep;
    float downstreamStepStandardDeviation;
};


// Results prepared to be transmitted to the GUI
// ------------------------------------------------------------

// Stores the final fuel results for one completed case
struct FuelResultsToTransmit
{
    bool available;

    uint8_t selectedBank;

    float features[9];

    float anomalyScore;
    float anomalyThreshold;
    bool anomalyAlert;

    const char* condition;
    const char* actionLevel;
    const char* evidenceSource;
    char description[512];
};


// Stores the final catalyst results for one completed case
struct CatalystResultsToTransmit
{
    bool available;

    uint8_t selectedBank;

    float features[5];

    float anomalyScore;
    float anomalyThreshold;
    bool isolationForestAlert;

    float maximumLaggedCorrelation;
    bool maximumLaggedCorrelationValid;

    bool mirroringThisCase;
    uint8_t consecutiveMirroringCases;
    bool persistentMirroring;

    bool strongMirroringThisCase;
    uint8_t consecutiveStrongMirroringCases;
    bool strongPersistentMirroring;

    bool hybridAlert;

    const char* condition;
    const char* actionLevel;
    const char* evidenceSource;
    char description[724];
};


// Stores the final charging results for one completed case
struct ChargingResultsToTransmit
{
    bool available;

    float features[7];

    float anomalyScore;
    float anomalyThreshold;
    bool anomalyAlert;

    bool strongChargingCase;
    uint8_t consecutiveStrongCases;

    const char* condition;
    const char* actionLevel;
    const char* evidenceSource;
    char description[512];
};

// Stores the DTCs returned by one OBD-II service.
struct DtcList
{
    bool responseReceived;
    size_t count;

    // Five-character DTC plus null terminator.
    char codes[MAX_DTC_CODES_PER_MODE][6];
};

// Stores everything that will eventually be placed in one case JSON message
struct CompletedCaseToTransmit
{
    uint32_t caseId;

    VehicleSample samples[CASE_SIZE];

    FuelResultsToTransmit fuel;
    CatalystResultsToTransmit catalyst;
    ChargingResultsToTransmit charging;
};


// Stores one non-overlapping 20-sample case
VehicleSample caseSamples[CASE_SIZE];

// Next position in the 20-sample array
size_t sampleIndex = 0;

// Time the last sample collection started
uint32_t previousSampleTime = 0;

// ECU response ID, starts at 0 until one responds
uint32_t selectedEcuResponseId = 0;

// Bank 2 availability is checked before logging begins.
// "Known" is kept separate from hasBank2 so a failed
// capability check is not mistaken for a single-bank engine.
bool bank2AvailabilityKnown = false;
bool hasBank2 = false;

// Consecutive catalyst mirroring cases are tracked separately by bank
uint8_t catalystB1MirroringConsecutiveCases = 0;
uint8_t catalystB2MirroringConsecutiveCases = 0;

uint8_t catalystB1StrongMirroringConsecutiveCases = 0;
uint8_t catalystB2StrongMirroringConsecutiveCases = 0;

uint8_t chargingStrongConsecutiveCases = 0;

// Carries engine-stop/restart state across 20-sample case boundaries.
bool chargingPreviousEngineStopped = false;
uint8_t chargingRestartSuppressionSamplesRemaining = 0;

// Identifies each completed 20-sample case during the current session
uint32_t currentCaseId = 0;


// BLE state
BLEServer* bleServer = nullptr;
BLECharacteristic* bleTxCharacteristic = nullptr;
BLECharacteristic* bleRxCharacteristic = nullptr;
bool bleDeviceConnected = false;

constexpr int LOGGING_COMMAND_NONE = 0;
constexpr int LOGGING_COMMAND_START = 1;
constexpr int LOGGING_COMMAND_PAUSE = 2;
constexpr int LOGGING_COMMAND_STATUS = 3;

std::atomic<int> pendingLoggingCommand{LOGGING_COMMAND_NONE};

std::atomic<bool> pendingDtcRequest{false};

std::atomic<bool> pendingVinRequest{false};

bool loggingEnabled = false;

void sendJsonPayloadOverBle(const String& jsonPayload);

// Sample initialization
// ------------------------------------------------------------

// Reset values before collecting a new sample
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


 
// CAN receive queue handling
// ------------------------------------------------------------

// Empty old CAN messages before a new PID request
void clearReceiveQueue()
{
    twai_message_t message = {};

    while (twai_receive(&message, 0) == ESP_OK) {}
     // Continue removing messages until the receive queue is empty
    
}


 
// OBD-II PID request function
// ------------------------------------------------------------

// Sends a Mode 01 current (live) data PID request and waits for a response

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
    if (twai_transmit(&request, pdMS_TO_TICKS(20)) != ESP_OK) return false;

    uint32_t requestStartTime = millis();

    // Continue checking received messages until the PID timeout expires
    while (millis() - requestStartTime < PID_TIMEOUT_MS)
    {
        twai_message_t response = {};

        // Wait up to ten milliseconds for the next received CAN message
        if (twai_receive(&response, pdMS_TO_TICKS(10)) != ESP_OK) continue;

        // Ignore extended CAN frames and remote request frames
        if (response.extd || response.rtr) continue;


        // Ignore CAN identifiers outside the standard OBD-II response range
        if (response.identifier < OBD_RESPONSE_MIN_ID || response.identifier > OBD_RESPONSE_MAX_ID) continue;


        // After an ECU has been selected, ignore responses from other ECUs
        if (selectedEcuResponseId != 0 && response.identifier != selectedEcuResponseId) continue;

        // The response must contain ISO-TP byte, response mode, PID, and requested data bytes
        if (response.data_length_code < 3 + requiredDataBytes) continue;

        // The upper four bits being zero indicates an ISO-TP single frame
        if ((response.data[0] & 0xF0) != 0x00) continue;


        // Lower four bits contain the ISO-TP payload length
        uint8_t payloadLength = response.data[0] & 0x0F;

        // Payload contains response mode, PID and returned data bytes
        if (payloadLength < 2 + requiredDataBytes) continue;

        // A Mode 01 request returns response mode 0x41
        // The returned PID must also match the requested PID
        if (response.data[1] != 0x41 || response.data[2] != pid) continue;

        // The first ECU that returns a valid response is selected
        if (selectedEcuResponseId == 0)
        {
            selectedEcuResponseId = response.identifier;

            Serial.printf( "Selected ECU response ID: 0x%03lX\n", static_cast<unsigned long>(selectedEcuResponseId));
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

// Determines whether the connected ECU reports a second engine bank.
// Positive evidence from either the fuel-trim support bitmap or the
// oxygen-sensor configuration is enough to confirm Bank 2.
void detectBank2Availability()
{
    bank2AvailabilityKnown = false;
    hasBank2 = false;

    bool bank2FuelTrimSupported = false;

    bool pid13Supported = false;
    bool pid1dSupported = false;

    bool pid13ResponseReceived = false;
    bool pid1dResponseReceived = false;

    bool bank2Pid13SensorsPresent = false;
    bool bank2Pid1dSensorsPresent = false;


    // PID 00 returns four bytes describing which Service 01
    // PIDs from 01 through 20 are supported by the ECU.
    uint8_t supportedPids[4] = {};

    bool pid00ResponseReceived = requestMode01Pid(0x00, supportedPids, 4);

    if (pid00ResponseReceived)
    {
        // Byte A covers PIDs 01 through 08.
        // Bit 0 represents PID 08, STFT Bank 2.
        bool bank2StftSupported = (supportedPids[0] & 0x01) != 0;

        // Byte B begins with PID 09 at bit 7.
        // PID 09 is LTFT Bank 2.
        bool bank2LtftSupported = (supportedPids[1] & 0x80) != 0;

        bank2FuelTrimSupported = bank2StftSupported || bank2LtftSupported;


        // Byte C covers PIDs 11 through 18.
        // Bit 5 represents PID 13.
        pid13Supported = (supportedPids[2] & 0x20) != 0;

        // Byte D covers PIDs 19 through 20.
        // Bit 3 represents PID 1D.
        pid1dSupported = (supportedPids[3] & 0x08) != 0;
    }


    // PID 13 reports the installed oxygen sensors using
    // the normal two-bank layout. Bits 4 through 7 are Bank 2.
    if (!pid00ResponseReceived || pid13Supported)
    {
        uint8_t oxygenSensorsTwoBank[1] = {};

        pid13ResponseReceived = requestMode01Pid(0x13, oxygenSensorsTwoBank, 1);

        if (pid13ResponseReceived)
        {
            bank2Pid13SensorsPresent = (oxygenSensorsTwoBank[0] & 0xF0) != 0;
        }
    }


    // PID 1D reports oxygen sensors using the four-bank layout.
    // Bits 2 and 3 represent Bank 2 Sensor 1 and Bank 2 Sensor 2.
    if (!pid00ResponseReceived || pid1dSupported)
    {
        uint8_t oxygenSensorsFourBank[1] = {};

        pid1dResponseReceived = requestMode01Pid(0x1D, oxygenSensorsFourBank, 1);

        if (pid1dResponseReceived)
        {
            bank2Pid1dSensorsPresent = (oxygenSensorsFourBank[0] & 0x0C) != 0;
        }
    }


    // Any positive Bank 2 evidence confirms that the vehicle
    // exposes a second OBD-II engine bank.
    if (bank2FuelTrimSupported || bank2Pid13SensorsPresent || bank2Pid1dSensorsPresent)
    {
        hasBank2 = true;
        bank2AvailabilityKnown = true;
    }
    else
    {
        // Only declare Bank 2 absent when the ECU support bitmap
        // was successfully received and at least one supported
        // oxygen-sensor layout was also successfully read.
        bool oxygenSensorLayoutConfirmed = false;

        if (pid13Supported && pid13ResponseReceived) oxygenSensorLayoutConfirmed = true;

        if (pid1dSupported && pid1dResponseReceived) oxygenSensorLayoutConfirmed = true;

        if (pid00ResponseReceived && oxygenSensorLayoutConfirmed)
        {
            hasBank2 = false;
            bank2AvailabilityKnown = true;
        }
    }


    Serial.println();
    Serial.println("--- BANK 2 AVAILABILITY CHECK ---");

    Serial.print("PID 00 response: ");

    if (pid00ResponseReceived) Serial.println("YES");
    else Serial.println("NO");

    Serial.print("Bank 2 fuel-trim support: ");

    if (bank2FuelTrimSupported) Serial.println("YES");
    else Serial.println("NO");

    Serial.print("PID 13 Bank 2 sensors: ");

    if (bank2Pid13SensorsPresent) Serial.println("YES");
    else Serial.println("NO");

    Serial.print("PID 1D Bank 2 sensors: ");

    if (bank2Pid1dSensorsPresent) Serial.println("YES");
    else Serial.println("NO");

    Serial.print("Bank 2 availability: ");

    if (!bank2AvailabilityKnown) Serial.println("UNKNOWN");
    else if (hasBank2) Serial.println("PRESENT");
    else Serial.println("NOT PRESENT");
}
 
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


// DTC request and ISO-TP handling
// ------------------------------------------------------------

// Converts the two raw SAE DTC bytes into a code such as P0141.
void decodeDtcCode(uint8_t byteA, uint8_t byteB, char output[6])
{
    static const char systemCharacters[4] =
    {
        'P', // powertrain module
        'C', // chassis module
        'B', // body module
        'U' // network module (CAN, wiring, etc...)
    };

    // Used to convert the remaining binary values
    // into their hexadecimal DTC characters
    static const char hexCharacters[] = "0123456789ABCDEF";

    // Bits 7-6 select the system category:
    // 00 = P, 01 = C, 10 = B, 11 = U
    output[0] = systemCharacters[(byteA >> 6) & 0x03];
    //continue shifting
    output[1] = static_cast<char>('0' + ((byteA >> 4) & 0x03));

    output[2] = hexCharacters[byteA & 0x0F];

    output[3] = hexCharacters[(byteB >> 4) & 0x0F];

    output[4] = hexCharacters[byteB & 0x0F];

    output[5] = '\0';
}


// Sends a flow-control frame after receiving an ISO-TP
// first frame from an ECU.
bool sendIsoTpFlowControl(
    uint32_t ecuResponseId)
{
    // For the standard 11-bit OBD-II addressing used here,
    // 0x7E8 responds to 0x7E0, 0x7E9 to 0x7E1, etc.
    if (ecuResponseId < OBD_RESPONSE_MIN_ID || ecuResponseId > OBD_RESPONSE_MAX_ID)
    {
        return false;
    }

    twai_message_t flowControl = {};

    flowControl.identifier = ecuResponseId - 0x08;

    flowControl.extd = 0;
    flowControl.rtr = 0;

    flowControl.data_length_code = 8;

    // ISO-TP Flow Control:
    // 0x30 = Continue To Send
    // Block size 0 = send all remaining frames
    // STmin 0 = no additional requested delay
    flowControl.data[0] = 0x30;
    flowControl.data[1] = 0x00;
    flowControl.data[2] = 0x00;

    flowControl.data[3] = 0x00;
    flowControl.data[4] = 0x00;
    flowControl.data[5] = 0x00;
    flowControl.data[6] = 0x00;
    flowControl.data[7] = 0x00;

    return twai_transmit(&flowControl, pdMS_TO_TICKS(20)) == ESP_OK;
}


// Requests one OBD-II service and returns the bytes after
// the positive response service byte.
//
// Supports both ISO-TP single-frame and multi-frame replies.
bool requestObdServicePayload(uint8_t service, uint8_t* outputData, size_t outputCapacity, size_t& outputLength)
{
    outputLength = 0;

    clearReceiveQueue();


    // --------------------------------------------------------
    // Send functional OBD-II request
    // --------------------------------------------------------

    twai_message_t request = {};

    request.identifier = OBD_FUNCTIONAL_REQUEST_ID;

    request.extd = 0;
    request.rtr = 0;

    request.data_length_code = 8;

    // One-byte OBD service request.
    request.data[0] = 0x01;
    request.data[1] = service;

    request.data[2] = 0x00;
    request.data[3] = 0x00;
    request.data[4] = 0x00;
    request.data[5] = 0x00;
    request.data[6] = 0x00;
    request.data[7] = 0x00;

    // Send the OBD-II request onto the CAN bus.
    // If transmission fails, stop the request immediately.
    if (twai_transmit(&request, pdMS_TO_TICKS(20)) != ESP_OK) return false;
    


    const uint8_t expectedResponseService = service + 0x40;
    // A positive OBD-II response uses the requested service
    // number plus 0x40.
    // Example: Mode 03 request -> 0x43 response.

    // Record when the request started so the response loop
    // can stop after the configured timeout.
    uint32_t requestStartTime = millis();

    // Tracks if ISO-TP response being reconstructed
    bool multiFrameActive = false;

    uint32_t multiFrameResponseId = 0;

    size_t expectedPayloadLength = 0;

    uint8_t expectedSequenceNumber = 1;


    // --------------------------------------------------------
    // Receive response
    // --------------------------------------------------------

    // Wait for the ECU response until the DTC response timeout is reached.
    while (millis() - requestStartTime < DTC_RESPONSE_TIMEOUT_MS)
    {
        twai_message_t response = {};

        // Try to receive the next CAN frame.
        // If no frame is available yet, keep waiting.
        if (twai_receive(&response, pdMS_TO_TICKS(10)) != ESP_OK)
        {
            continue;
        }

    // Ignore extended-ID frames and remote request frames.
    // Standard OBD-II responses use normal 11-bit data frames.
        if (response.extd ||response.rtr)
        {
            continue;
        }

    // Only accept responses from the standard OBD-II ECU
    // response ID range of 0x7E8 through 0x7EF.
        if (response.identifier < OBD_RESPONSE_MIN_ID || response.identifier > OBD_RESPONSE_MAX_ID)
        {
            continue;
        }


        // If live-data collection already selected an ECU,
        // stay with that ECU.
        if (selectedEcuResponseId != 0 && response.identifier != selectedEcuResponseId)
        {
            continue;
        }

    // At least two bytes are needed to identify
    // the ISO-TP frame type and response contents.
        if (response.data_length_code < 2)
        {
            continue;
        }

        // The upper four bits identify the ISO-TP frame type:
        // 0x0 = Single Frame, 0x1 = First Frame, 0x2 = Consecutive Frame.
        const uint8_t frameType = response.data[0] >> 4;


        // ----------------------------------------------------
        // ISO-TP single frame
        // ----------------------------------------------------

        if (!multiFrameActive && frameType == 0x00)
        {
            // Handle a complete ISO-TP Single Frame response.
            // This is used when the full DTC response fits in one CAN frame.
            const uint8_t payloadLength = response.data[0] & 0x0F;

    // The lower four bits of the first byte contain the number of payload bytes that follow.
            if (payloadLength < 1)
            {
                continue;
            }


            if (response.data_length_code < 1 + payloadLength)
            {
                continue;
            }


            if (response.data[1] != expectedResponseService)
            {
                continue;
            }


            const size_t dataLength = payloadLength - 1;


            if (dataLength > outputCapacity)
            {
                return false;
            }

            // If an ECU has not already been selected,
            // use the ECU that returned this valid response.
            if (selectedEcuResponseId == 0)
            {
                selectedEcuResponseId = response.identifier;

                Serial.printf("Selected ECU response ID: "
                    "0x%03lX\n", static_cast<unsigned long>(selectedEcuResponseId));
            }


            // Copy the DTC response data into the caller's buffer.
            // response.data[0] is ISO-TP information and
            // response.data[1] is the OBD-II response service.
            for (size_t i = 0; i < dataLength; i++)
            {
                outputData[i] = response.data[2 + i];
            }

            // Store how many useful payload bytes were returned.
            outputLength = dataLength;

            return true; // A complete valid Single Frame response was received.

        }


        // ----------------------------------------------------
        // ISO-TP first frame
        // ----------------------------------------------------

        if (!multiFrameActive && frameType == 0x01)
        {
            if (response.data_length_code < 3)
            {
                continue;
            }


            const size_t totalIsoTpLength =
                (static_cast<size_t>(response.data[0] & 0x0F) << 8) | response.data[1];


            // Total ISO-TP payload includes
            // the response service byte.
            if (totalIsoTpLength < 1)
            {
                continue;
            }


            if (response.data[2] != expectedResponseService)
            {
                continue;
            }


            expectedPayloadLength = totalIsoTpLength - 1;

        // Stop if the caller's output buffer is not large
        // enough for the complete multi-frame payload.
            if (expectedPayloadLength > outputCapacity)
            {
                return false;
            }

        // If an ECU has not already been selected,
        // use the ECU that returned this valid response.
            if (selectedEcuResponseId == 0)
            {
                selectedEcuResponseId = response.identifier;

                Serial.printf(
                    "Selected ECU response ID: "
                    "0x%03lX\n",
                    static_cast<unsigned long>(selectedEcuResponseId));
            }


            multiFrameResponseId = response.identifier;


            // First-frame application data begins
            // after PCI bytes and response service.
            size_t availableBytes = 0;

            if (response.data_length_code > 3) availableBytes = response.data_length_code - 3;
        
        
        // Start by assuming the remaining payload will fit.
        // If fewer bytes are available in this first frame,
        // only copy the bytes that are actually present.
            size_t bytesToCopy = expectedPayloadLength;

            if (availableBytes < expectedPayloadLength) bytesToCopy = availableBytes;
            

            for (size_t i = 0; i < bytesToCopy; i++)
            {
                outputData[i] = response.data[3 + i];
            }


            outputLength = bytesToCopy;


            if (outputLength >= expectedPayloadLength)
            {
                return true;
            }


            if (!sendIsoTpFlowControl(multiFrameResponseId))
            {
                return false;
            }


            multiFrameActive = true;

            expectedSequenceNumber = 1;

            continue;
        }


        // ----------------------------------------------------
        // ISO-TP consecutive frame
        // ----------------------------------------------------

        if (multiFrameActive && response.identifier == multiFrameResponseId && frameType == 0x02)
        {
            const uint8_t sequenceNumber = response.data[0] & 0x0F;


            if (sequenceNumber != expectedSequenceNumber)
            {
                Serial.println("ISO-TP sequence mismatch.");

                return false;
            }


            expectedSequenceNumber = (expectedSequenceNumber + 1) & 0x0F;


            const size_t remainingBytes = expectedPayloadLength - outputLength;


            size_t availableBytes = 0;

            if (response.data_length_code > 1) availableBytes = response.data_length_code - 1;
            


            size_t bytesToCopy = remainingBytes;

            if (availableBytes < remainingBytes) bytesToCopy = availableBytes;
            

            for (size_t i = 0; i < bytesToCopy; i++)
            {
                outputData[outputLength + i] = response.data[1 + i];
            }


            outputLength += bytesToCopy;


            if (outputLength >= expectedPayloadLength)
            {
                return true;
            }
        }
    }


    return false;
}


// Reads and decodes one DTC mode.
void readDtcMode(uint8_t service, DtcList& result)
{
    result.responseReceived = false;
    result.count = 0;


    uint8_t responseData[MAX_DTC_RESPONSE_BYTES] = {};

    size_t responseLength = 0;


    if (!requestObdServicePayload(service, responseData, MAX_DTC_RESPONSE_BYTES, responseLength))
    {
        return;
    }


    result.responseReceived = true;


    // DTCs are returned as two-byte values.
    for (size_t i = 0; i + 1 < responseLength && result.count < MAX_DTC_CODES_PER_MODE; i += 2)
    {
        const uint8_t byteA = responseData[i];

        const uint8_t byteB = responseData[i + 1];


        // 0x0000 is padding / no DTC.
        if (byteA == 0x00 && byteB == 0x00)
        {
            continue;
        }


        decodeDtcCode(byteA, byteB, result.codes[result.count]);

        result.count++;
    }
}


// Adds one DTC list to the outgoing JSON.
void addDtcListToJson(JsonDocument& doc, const char* arrayName, const char* responseName, const DtcList& dtcs)
{
    doc[responseName] = dtcs.responseReceived;

    JsonArray array = doc[arrayName].to<JsonArray>();


    for (size_t i = 0; i < dtcs.count; i++)
    {
        array.add(dtcs.codes[i]);
    }
}


// Performs the complete confirmed/pending/permanent scan
// and sends one result message to the Qt application.
void performDtcScanAndSend()
{
    Serial.println();
    Serial.println("=== DTC SCAN STARTED ===");


    DtcList confirmed = {};
    DtcList pending = {};
    DtcList permanent = {};


    Serial.println("Reading confirmed DTCs (Mode 03)...");

    readDtcMode(0x03, confirmed);


    Serial.println("Reading pending DTCs (Mode 07)...");

    readDtcMode(0x07, pending);


    Serial.println("Reading permanent DTCs (Mode 0A)...");

    readDtcMode(0x0A, permanent);


    JsonDocument doc;

    doc["type"] = "dtc_result";


    const bool anyResponse = confirmed.responseReceived || pending.responseReceived || permanent.responseReceived;


    doc["success"] = anyResponse;


    addDtcListToJson(doc, "confirmed", "confirmed_response", confirmed);

    addDtcListToJson(doc, "pending", "pending_response", pending);

    addDtcListToJson(doc, "permanent", "permanent_response", permanent);


    if (!anyResponse)
    {
        doc["error"] = "No OBD-II DTC response received";
    }


    String jsonPayload;

    serializeJson(doc, jsonPayload);


    Serial.println();
    Serial.println("--- DTC RESULT JSON ---");

    Serial.println(jsonPayload);


    sendJsonPayloadOverBle(jsonPayload);


    Serial.println();
    Serial.println("=== DTC SCAN COMPLETE ===");
} 

// VIN / vehicle information
// ------------------------------------------------------------

bool isValidVinCharacter(char value)
{
    const bool isNumber = value >= '0' && value <= '9';

    const bool isLetter = value >= 'A' && value <= 'Z';

    if (!isNumber && !isLetter)
    {
        return false;
    }

    // These letters are not used in standard VINs.
    if (value == 'I' || value == 'O' || value == 'Q')
    {
        return false;
    }

    return true;
}


// Requests OBD-II Mode 09 PID 02 and reconstructs
// the 17-character VIN from the ISO-TP response.
bool requestVehicleVin(char outputVin[VIN_LENGTH + 1])
{
    clearReceiveQueue();


    twai_message_t request = {};

    request.identifier = OBD_FUNCTIONAL_REQUEST_ID;

    request.extd = 0;
    request.rtr = 0;
    request.data_length_code = 8;


    // ISO-TP single-frame request:
    //
    // 02 09 02
    //
    // 02 = two OBD payload bytes follow
    // 09 = Request Vehicle Information
    // 02 = VIN
    request.data[0] = 0x02;
    request.data[1] = 0x09;
    request.data[2] = 0x02;

    request.data[3] = 0x00;
    request.data[4] = 0x00;
    request.data[5] = 0x00;
    request.data[6] = 0x00;
    request.data[7] = 0x00;


    if (twai_transmit(&request, pdMS_TO_TICKS(20)) != ESP_OK)
    {
        return false;
    }


    // Bytes collected after the positive
    // response service byte 0x49.
    uint8_t responsePayload[VIN_RESPONSE_BUFFER_SIZE] = {};

    size_t responseLength = 0;

    size_t expectedResponseLength = 0;

    bool multiFrameActive = false;

    uint32_t multiFrameResponseId = 0;

    uint8_t expectedSequenceNumber = 1;

    const uint32_t requestStartTime = millis();


    while (millis() - requestStartTime < VIN_RESPONSE_TIMEOUT_MS)
    {
        twai_message_t response = {};


        if (twai_receive(&response, pdMS_TO_TICKS(10)) != ESP_OK)
        {
            continue;
        }


        if (response.extd || response.rtr)
        {
            continue;
        }


        if (response.identifier < OBD_RESPONSE_MIN_ID || response.identifier > OBD_RESPONSE_MAX_ID)
        {
            continue;
        }


        if (selectedEcuResponseId != 0 && response.identifier != selectedEcuResponseId)
        {
            continue;
        }


        if (response.data_length_code < 2)
        {
            continue;
        }


        const uint8_t frameType = response.data[0] >> 4;


        // ----------------------------------------------------
        // Single frame
        // ----------------------------------------------------

        if (!multiFrameActive && frameType == 0x00)
        {
            const uint8_t isoTpLength = response.data[0] & 0x0F;


            if (isoTpLength < 1 || response.data_length_code < 1 + isoTpLength)
            {
                continue;
            }


            // Positive response to Mode 09.
            if (response.data[1] != 0x49)
            {
                continue;
            }


            const size_t bytesAfterService = isoTpLength - 1;


            if (bytesAfterService > VIN_RESPONSE_BUFFER_SIZE)
            {
                return false;
            }


            if (selectedEcuResponseId == 0)
            {
                selectedEcuResponseId = response.identifier;

                Serial.printf("Selected ECU response ID: " "0x%03lX\n", static_cast<unsigned long>(selectedEcuResponseId));
            }


            for (size_t i = 0; i < bytesAfterService; i++)
            {
                responsePayload[i] = response.data[2 + i];
            }


            responseLength = bytesAfterService;

            break;
        }


        // ----------------------------------------------------
        // First frame
        // ----------------------------------------------------

        if (!multiFrameActive && frameType == 0x01)
        {
            if (response.data_length_code < 4)
            {
                continue;
            }


            const size_t totalIsoTpLength = (static_cast<size_t>(response.data[0] & 0x0F) << 8) | response.data[1];


            if (totalIsoTpLength < 1)
            {
                continue;
            }


            // Positive Mode 09 response.
            if (response.data[2] != 0x49)
            {
                continue;
            }


            // We strip the 0x49 service byte.
            expectedResponseLength = totalIsoTpLength - 1;


            if (expectedResponseLength > VIN_RESPONSE_BUFFER_SIZE)
            {
                return false;
            }


            if (selectedEcuResponseId == 0)
            {
                selectedEcuResponseId = response.identifier;

                Serial.printf("Selected ECU response ID: " "0x%03lX\n", static_cast<unsigned long>(selectedEcuResponseId));
            }


            multiFrameResponseId = response.identifier;


            // First-frame bytes after 0x49 begin
            // at response.data[3].
            size_t availableBytes = 0;

            if (response.data_length_code > 3) availableBytes = response.data_length_code - 3;


            size_t bytesToCopy = expectedResponseLength;

            if (availableBytes < expectedResponseLength) bytesToCopy = availableBytes;


            for (size_t i = 0; i < bytesToCopy; i++)
            {
                responsePayload[i] = response.data[3 + i];
            }


            responseLength = bytesToCopy;


            if (!sendIsoTpFlowControl(multiFrameResponseId))
            {
                return false;
            }


            multiFrameActive = true;

            expectedSequenceNumber = 1;

            continue;
        }


        // ----------------------------------------------------
        // Consecutive frames
        // ----------------------------------------------------

        if (multiFrameActive && response.identifier == multiFrameResponseId && frameType == 0x02)
        {
            const uint8_t sequenceNumber = response.data[0] & 0x0F;


            if (sequenceNumber != expectedSequenceNumber)
            {
                Serial.println("VIN ISO-TP sequence mismatch.");

                return false;
            }


            expectedSequenceNumber = (expectedSequenceNumber + 1) & 0x0F;


            const size_t remainingBytes = expectedResponseLength - responseLength;


            size_t availableBytes = 0;

            if (response.data_length_code > 1) availableBytes = response.data_length_code - 1;


            size_t bytesToCopy = remainingBytes;

            if (availableBytes < remainingBytes) bytesToCopy = availableBytes;


            for (size_t i = 0; i < bytesToCopy; i++)
            {
                responsePayload[responseLength + i] =
                    response.data[1 + i];
            }


            responseLength += bytesToCopy;


            if (responseLength >= expectedResponseLength)
            {
                break;
            }
        }
    }


    // Expected Mode 09 PID 02 application data:
    //
    // 02 01 [17 VIN ASCII bytes]
    //
    // 02 = PID
    // 01 = one VIN data item
    if (responseLength < VIN_LENGTH + 2)
    {
        return false;
    }


    if (responsePayload[0] != 0x02)
    {
        return false;
    }


    if (responsePayload[1] != 0x01)
    {
        return false;
    }


    for (size_t i = 0; i < VIN_LENGTH; i++)
    {
        char vinCharacter = static_cast<char>(responsePayload[2 + i]);


        // Normalize lowercase ASCII if an unusual
        // ECU returns it.
        if (vinCharacter >= 'a' && vinCharacter <= 'z')
        {
            vinCharacter = static_cast<char>(vinCharacter - 'a' + 'A');
        }


        if (!isValidVinCharacter(vinCharacter))
        {
            return false;
        }


        outputVin[i] = vinCharacter;
    }


    outputVin[VIN_LENGTH] = '\0';


    return true;
}


// Reads the VIN and sends it to the Qt application.
//
// The ESP32 deliberately does not decode the
// model year or make. That is handled locally
// in the Android application.
void performVehicleInfoReadAndSend()
{
    Serial.println();
    Serial.println("=== VEHICLE INFORMATION READ STARTED ===");


    char vin[VIN_LENGTH + 1] = {};


    const bool success = requestVehicleVin(vin);


    JsonDocument doc;

    doc["type"] = "vehicle_info";

    doc["success"] = success;


    if (success)
    {
        doc["vin"] = vin;


        Serial.print("VIN: ");

        Serial.println(vin);
    }
    else
    {
        doc["vin"] = nullptr;

        doc["error"] = "No valid VIN response received";


        Serial.println("VIN unavailable.");
    }


    String jsonPayload;

    serializeJson(doc, jsonPayload);


    Serial.println();
    Serial.println("--- VEHICLE INFO JSON ---");

    Serial.println(jsonPayload);


    sendJsonPayloadOverBle(jsonPayload);


    Serial.println();
    Serial.println("=== VEHICLE INFORMATION READ COMPLETE ===");
}

// Complete sample collection
// ------------------------------------------------------------

// Requests all required PIDs and stores their decoded values in one
// VehicleSample structure
void collectOneSample(VehicleSample& sample)
{
    initializeSample(sample);

    // Temporary array used to hold up to four returned PID data bytes
    uint8_t data[4] = {};

    // Continue requesting Bank 2 data when availability is unknown.
    // Skip Bank 2 requests only when the vehicle has been
    // positively identified as a single-bank vehicle.
    bool shouldRequestBank2 = !bank2AvailabilityKnown || hasBank2;

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
    // Four-cylinder engines normally do not have Bank 2, so this PID may be unsupported on many vehicles
    if (shouldRequestBank2 && requestMode01Pid(0x08, data, 1))
    {
        sample.stftB2 = decodeFuelTrim(data[0]);
        sample.stftB2Valid = true;
    }


    // PID 09: Long-term fuel trim, Bank 2
    if (shouldRequestBank2 && requestMode01Pid(0x09, data, 1))
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
    // In the normal two-bank layout this is Bank 1 Sensor 1, which is the upstream O2 sensor
    if (requestMode01Pid(0x14, data, 2))
    {
        sample.o2B1S1Voltage = decodeNarrowbandO2Voltage(data[0]);

        sample.o2B1S1VoltageValid = true;
    }


    // PID 15: O2 Sensor 2
    // In the normal two-bank layout this is Bank 1 Sensor 2, which is the downstream O2 sensor
    if (requestMode01Pid(0x15, data, 2))
    {
        sample.o2B1S2Voltage = decodeNarrowbandO2Voltage(data[0]);

        sample.o2B1S2VoltageValid = true;
    }


    // PID 18: O2 Sensor 5
    // In the normal two-bank layout this is Bank 2 Sensor 1, which is the upstream O2 sensor for Bank 2
    if (shouldRequestBank2 && requestMode01Pid(0x18, data, 2))
    {
        sample.o2B2S1Voltage = decodeNarrowbandO2Voltage(data[0]);

        sample.o2B2S1VoltageValid = true;
    }


    // PID 19: O2 Sensor 6
    // In the normal two-bank layout this is Bank 2 Sensor 2, which is the downstream O2 sensor for Bank 2
    if (shouldRequestBank2 && requestMode01Pid(0x19, data, 2))
    {
        sample.o2B2S2Voltage = decodeNarrowbandO2Voltage(data[0]);

        sample.o2B2S2VoltageValid = true;
    }


    // PID 24: Wideband O2 Sensor 1
    // Normally represents Bank 1 Sensor 1 in a two-bank layout
    if (requestMode01Pid(0x24, data, 4))
    {
        sample.o2B1S1EquivalenceRatio = decodeWidebandEquivalenceRatio( data[0], data[1]);

        sample.o2B1S1EquivalenceRatioValid = true;

        // Use the wideband voltage when narrowband PID 14 was unavailable
        if (!sample.o2B1S1VoltageValid)
        {
            sample.o2B1S1Voltage = decodeWidebandVoltage(data[2], data[3]);

            sample.o2B1S1VoltageValid = true;
        }
    }


    // PID 25: Wideband O2 Sensor 2
    // Used as a fallback for Bank 1 downstream voltage when PID 15 was unavailable
    if (!sample.o2B1S2VoltageValid && requestMode01Pid(0x25, data, 4))
    {
        sample.o2B1S2Voltage = decodeWidebandVoltage(data[2], data[3]);

        sample.o2B1S2VoltageValid = true;
    }


    // PID 28: Wideband O2 Sensor 5
    // Normally represents Bank 2 Sensor 1 in a two-bank layout
    if (shouldRequestBank2 && requestMode01Pid(0x28, data, 4))
    {
        sample.o2B2S1EquivalenceRatio = decodeWidebandEquivalenceRatio(data[0], data[1]);

        sample.o2B2S1EquivalenceRatioValid = true;

        // Use wideband voltage when narrowband PID 18 was unavailable
        if (!sample.o2B2S1VoltageValid)
        {
            sample.o2B2S1Voltage = decodeWidebandVoltage(data[2], data[3]);

            sample.o2B2S1VoltageValid = true;
        }
    }


    // PID 29: Wideband O2 Sensor 6
    // Used as a fallback for Bank 2 downstream voltage when PID 19 was unavailable
    if (shouldRequestBank2 && !sample.o2B2S2VoltageValid && requestMode01Pid(0x29, data, 4))
    {
        sample.o2B2S2Voltage = decodeWidebandVoltage(data[2], data[3]);

        sample.o2B2S2VoltageValid = true;
    }


    // PID 42: Control-module voltage
    if (requestMode01Pid(0x42, data, 2))
    {
        sample.controlModuleVoltage = decodeControlModuleVoltage(data[0],data[1]);

        sample.controlModuleVoltageValid = true;
    }
}


 
// Serial output
// ------------------------------------------------------------

// Prints a floating-point value when it is valid
// Prints NA when the vehicle did not return a valid value
void printValue(float value, bool valid)
{
    if (valid) Serial.print(value, 6);
    else Serial.print("NA");
}


// Prints one complete sample in CSV format
void printSample(const VehicleSample& sample, size_t index)
{
    Serial.print(index);
    Serial.print(',');

    Serial.print(sample.timestampMs);
    Serial.print(',');

    printValue(sample.engineLoad, sample.engineLoadValid);
    Serial.print(',');

    printValue(sample.stftB1, sample.stftB1Valid);
    Serial.print(',');

    printValue(sample.ltftB1, sample.ltftB1Valid);
    Serial.print(',');

    printValue(sample.stftB2, sample.stftB2Valid);
    Serial.print(',');

    printValue(sample.ltftB2, sample.ltftB2Valid);
    Serial.print(',');

    printValue(sample.rpm, sample.rpmValid);
    Serial.print(',');

    printValue(sample.o2B1S1Voltage, sample.o2B1S1VoltageValid);
    Serial.print(',');

    printValue(sample.o2B1S2Voltage, sample.o2B1S2VoltageValid);
    Serial.print(',');

    printValue(sample.o2B1S1EquivalenceRatio, sample.o2B1S1EquivalenceRatioValid);
    Serial.print(',');

    printValue(sample.o2B2S1Voltage, sample.o2B2S1VoltageValid);
    Serial.print(',');

    printValue(sample.o2B2S2Voltage, sample.o2B2S2VoltageValid);
    Serial.print(',');

    printValue(sample.o2B2S1EquivalenceRatio, sample.o2B2S1EquivalenceRatioValid);
    Serial.print(',');

    printValue(sample.controlModuleVoltage, sample.controlModuleVoltageValid);

    Serial.println();
}


 
// General statistics calculation
// ------------------------------------------------------------

// Calculates mean, population STD, min, max, and range
ValueStatistics calculateStatistics(const float values[CASE_SIZE], const bool validValues[CASE_SIZE])
{
    ValueStatistics result = {};

    result.available = false;
    result.validCount = 0;

    result.mean = NAN;
    result.standardDeviation = NAN;
    result.minimum = NAN;
    result.maximum = NAN;
    result.range = NAN;

    double sum = 0.0;

    // First pass: count, sum, min, and max
    for (size_t i = 0; i < CASE_SIZE; i++)
    {
        if (!validValues[i] || !isfinite(values[i])) continue;

        float currentValue = values[i];

        if (result.validCount == 0)
        {
            result.minimum = currentValue;
            result.maximum = currentValue;
        }
        else
        {
            if (currentValue < result.minimum) result.minimum = currentValue;

            if (currentValue > result.maximum) result.maximum = currentValue;
        }

        sum += currentValue;
        result.validCount++;
    }

    // No valid readings
    if (result.validCount == 0) return result;

    result.mean = static_cast<float>(sum / result.validCount);

    double squaredDifferenceSum = 0.0;

    // Second pass calculates population standard deviation
    // The training data used ddof = 0, so the divisor is the number of valid readings rather than valid readings minus one
    for (size_t i = 0; i < CASE_SIZE; i++)
    {
        if (!validValues[i] || !isfinite(values[i])) continue;

        double difference = values[i] - result.mean;

        squaredDifferenceSum += difference * difference;
    }

    result.standardDeviation = static_cast<float>(sqrt(squaredDifferenceSum / result.validCount));

    result.range = result.maximum - result.minimum;

    result.available = true;

    return result;
}


 
// Fuel feature calculation
// ------------------------------------------------------------

// Creates the 9 fuel features
FuelCaseFeatures calculateFuelFeatures(const VehicleSample samples[CASE_SIZE])
{
    FuelCaseFeatures result = {};

    result.ready = false;
    result.selectedBank = 0;

    float rpmValues[CASE_SIZE];
    bool rpmValid[CASE_SIZE];

    float loadValues[CASE_SIZE];
    bool loadValid[CASE_SIZE];

    float stftB1Values[CASE_SIZE];
    bool stftB1Valid[CASE_SIZE];

    float ltftB1Values[CASE_SIZE];
    bool ltftB1Valid[CASE_SIZE];

    float totalTrimB1Values[CASE_SIZE];
    bool totalTrimB1Valid[CASE_SIZE];

    float absoluteTotalTrimB1Values[CASE_SIZE];
    bool absoluteTotalTrimB1Valid[CASE_SIZE];

    float stftB2Values[CASE_SIZE];
    bool stftB2Valid[CASE_SIZE];

    float ltftB2Values[CASE_SIZE];
    bool ltftB2Valid[CASE_SIZE];

    float totalTrimB2Values[CASE_SIZE];
    bool totalTrimB2Valid[CASE_SIZE];

    float absoluteTotalTrimB2Values[CASE_SIZE];
    bool absoluteTotalTrimB2Valid[CASE_SIZE];


    // Copy raw values into arrays. Total trim = STFT + LTFT from the same bank
    for (size_t i = 0; i < CASE_SIZE; i++)
    {
        rpmValues[i] = samples[i].rpm;
        rpmValid[i] = samples[i].rpmValid;

        loadValues[i] = samples[i].engineLoad;
        loadValid[i] = samples[i].engineLoadValid;


        stftB1Values[i] = samples[i].stftB1;
        stftB1Valid[i] = samples[i].stftB1Valid;

        ltftB1Values[i] = samples[i].ltftB1;
        ltftB1Valid[i] = samples[i].ltftB1Valid;

        totalTrimB1Valid[i] = samples[i].stftB1Valid && samples[i].ltftB1Valid;

        absoluteTotalTrimB1Valid[i] = totalTrimB1Valid[i];

        if (totalTrimB1Valid[i])
        {
            totalTrimB1Values[i] = samples[i].stftB1 + samples[i].ltftB1;

            absoluteTotalTrimB1Values[i] = fabs(totalTrimB1Values[i]);
        }
        else
        {
            totalTrimB1Values[i] = NAN;
            absoluteTotalTrimB1Values[i] = NAN;
        }


        stftB2Values[i] = samples[i].stftB2;
        stftB2Valid[i] = samples[i].stftB2Valid;

        ltftB2Values[i] = samples[i].ltftB2;
        ltftB2Valid[i] = samples[i].ltftB2Valid;

        totalTrimB2Valid[i] = samples[i].stftB2Valid && samples[i].ltftB2Valid;

        absoluteTotalTrimB2Valid[i] = totalTrimB2Valid[i];

        if (totalTrimB2Valid[i])
        {
            totalTrimB2Values[i] = samples[i].stftB2 + samples[i].ltftB2;

            absoluteTotalTrimB2Values[i] = fabs(totalTrimB2Values[i]);
        }
        else
        {
            totalTrimB2Values[i] = NAN;
            absoluteTotalTrimB2Values[i] = NAN;
        }
    }


    ValueStatistics rpm = calculateStatistics(rpmValues, rpmValid);

    ValueStatistics load = calculateStatistics(loadValues, loadValid);

    ValueStatistics stftB1 = calculateStatistics(stftB1Values, stftB1Valid);

    ValueStatistics ltftB1 = calculateStatistics(ltftB1Values, ltftB1Valid);

    ValueStatistics totalTrimB1 = calculateStatistics(totalTrimB1Values, totalTrimB1Valid);

    ValueStatistics absoluteTotalTrimB1 = calculateStatistics(absoluteTotalTrimB1Values, absoluteTotalTrimB1Valid);


    ValueStatistics stftB2 = calculateStatistics(stftB2Values, stftB2Valid);

    ValueStatistics ltftB2 = calculateStatistics(ltftB2Values, ltftB2Valid);

    ValueStatistics totalTrimB2 = calculateStatistics(totalTrimB2Values, totalTrimB2Valid);

    ValueStatistics absoluteTotalTrimB2 = calculateStatistics(absoluteTotalTrimB2Values, absoluteTotalTrimB2Valid);


    bool bank1Reliable = absoluteTotalTrimB1.validCount >= MIN_VALID_SAMPLES;

    bool bank2Reliable = absoluteTotalTrimB2.validCount >= MIN_VALID_SAMPLES;


    // Bank 2 is selected only when it is available and its average absolute total trim is worse than Bank 1
    bool useBank2 = bank2Reliable && (!bank1Reliable || absoluteTotalTrimB2.mean > absoluteTotalTrimB1.mean);


    if (!bank1Reliable && !bank2Reliable) return result;


    ValueStatistics selectedStft;
    ValueStatistics selectedLtft;
    ValueStatistics selectedTotalTrim;
    ValueStatistics selectedAbsoluteTotalTrim;


    // Select all fuel trim statistics from the same bank
    if (useBank2)
    {
        selectedStft = stftB2;
        selectedLtft = ltftB2;
        selectedTotalTrim = totalTrimB2;
        selectedAbsoluteTotalTrim = absoluteTotalTrimB2;

        result.selectedBank = 2;
    }
    else
    {
        selectedStft = stftB1;
        selectedLtft = ltftB1;
        selectedTotalTrim = totalTrimB1;
        selectedAbsoluteTotalTrim = absoluteTotalTrimB1;

        result.selectedBank = 1;
    }


    // Final V4 fuel feature order

    result.values[0] = rpm.mean;

    result.values[1] = load.mean;

    result.values[2] = selectedStft.mean;

    result.values[3] = selectedStft.standardDeviation;

    result.values[4] = selectedLtft.mean;

    result.values[5] = selectedLtft.standardDeviation;

    result.values[6] = selectedTotalTrim.mean;

    result.values[7] = selectedAbsoluteTotalTrim.mean;

    result.values[8] = selectedAbsoluteTotalTrim.minimum;


    result.ready =
        rpm.validCount >= MIN_VALID_SAMPLES &&
        load.validCount >= MIN_VALID_SAMPLES &&
        selectedStft.validCount >= MIN_VALID_SAMPLES && selectedLtft.validCount >= MIN_VALID_SAMPLES && selectedTotalTrim.validCount >= MIN_VALID_SAMPLES;

    return result;
}

 
// Catalyst bank feature calculation
// ------------------------------------------------------------

// Calculates catalyst features for one bank
CatalystBankFeatures calculateCatalystBankFeatures(const ValueStatistics& upstreamVoltage, const ValueStatistics& upstreamEquivalenceRatio, const float downstreamValues[CASE_SIZE], const bool downstreamValid[CASE_SIZE], const ValueStatistics& downstreamVoltage)
{
    CatalystBankFeatures result = {};

    result.available = false;
    result.useUpstreamVoltage = false;

    result.downstreamStandardDeviation = NAN;
    result.downstreamRange = NAN;
    result.downstreamMeanAbsoluteStep = NAN;
    result.downstreamStepStandardDeviation = NAN;


    // The final V4 catalyst data requires enough samples and enough
    // signal movement for the O2 readings to be diagnostically useful
    bool voltageReliable = upstreamVoltage.validCount >= MIN_VALID_SAMPLES && upstreamVoltage.standardDeviation >= 0.01f;

    bool equivalenceRatioReliable = upstreamEquivalenceRatio.validCount >= MIN_VALID_SAMPLES && upstreamEquivalenceRatio.standardDeviation >= 0.01f;

    bool downstreamReliable = downstreamVoltage.validCount >= MIN_VALID_SAMPLES && downstreamVoltage.standardDeviation >= 0.005f;


    // Upstream voltage is preferred
    // Equivalence ratio is used when upstream voltage is not useful
    if (!voltageReliable && !equivalenceRatioReliable)
    {
        return result;
    }

    if (!downstreamReliable)
    {
        return result;
    }


    if (voltageReliable)
    {
        result.useUpstreamVoltage = true;
    }
    else
    {
        result.useUpstreamVoltage = false;
    }


    // Store the valid downstream readings in order
    // Missing readings are skipped before calculating the step features
    float validDownstreamValues[CASE_SIZE];

    size_t validDownstreamCount = 0;

    for (size_t i = 0; i < CASE_SIZE; i++)
    {
        if (!downstreamValid[i] || !isfinite(downstreamValues[i]))
        {
            continue;
        }

        validDownstreamValues[validDownstreamCount] = downstreamValues[i];

        validDownstreamCount++;
    }


    if (validDownstreamCount < MIN_VALID_SAMPLES)
    {
        return result;
    }


    float downstreamSteps[CASE_SIZE - 1];

    size_t stepCount = 0;

    double absoluteStepSum = 0.0;
    double stepSum = 0.0;


    // Calculate changes between consecutive valid downstream readings
    for (size_t i = 1; i < validDownstreamCount; i++)
    {
        float currentStep = validDownstreamValues[i] - validDownstreamValues[i - 1];

        downstreamSteps[stepCount] = currentStep;

        absoluteStepSum += fabs(currentStep);

        stepSum += currentStep;

        stepCount++;
    }


    if (stepCount == 0)
    {
        return result;
    }


    float meanAbsoluteStep = static_cast<float>(absoluteStepSum / stepCount);

    double meanStep = stepSum / stepCount;

    double squaredStepDifferenceSum = 0.0;


    // Population STD of the signed downstream changes
    for (size_t i = 0; i < stepCount; i++)
    {
        double difference = downstreamSteps[i] - meanStep;

        squaredStepDifferenceSum += difference * difference;
    }


    float stepStandardDeviation = static_cast<float>(sqrt(squaredStepDifferenceSum / stepCount));


    result.downstreamStandardDeviation = downstreamVoltage.standardDeviation;

    result.downstreamRange = downstreamVoltage.range;

    result.downstreamMeanAbsoluteStep = meanAbsoluteStep;

    result.downstreamStepStandardDeviation = stepStandardDeviation;

    result.available = true;

    return result;
}



// O2 correlation calculation
// ------------------------------------------------------------

// Calculates upstream/downstream O2 correlation at one lag
bool calculateCorrelationAtLag(const float upstreamValues[CASE_SIZE], const bool upstreamValid[CASE_SIZE], const float downstreamValues[CASE_SIZE], const bool downstreamValid[CASE_SIZE], int downstreamLag, float& correlationOutput)
{
    double upstreamSum = 0.0;
    double downstreamSum = 0.0;
    size_t pairCount = 0;

    // First pass gets the paired averages
    for (int upstreamIndex = 0; upstreamIndex < static_cast<int>(CASE_SIZE); upstreamIndex++)
    {
        int downstreamIndex = upstreamIndex - downstreamLag;

        // Skip rows shifted outside the 20-sample case
        if (downstreamIndex < 0 || downstreamIndex >= static_cast<int>(CASE_SIZE))
        {
            continue;
        }

        // Only use rows where both O2 readings are valid
        if (!upstreamValid[upstreamIndex] || !downstreamValid[downstreamIndex] || !isfinite(upstreamValues[upstreamIndex]) || !isfinite(downstreamValues[downstreamIndex]))
        {
            continue;
        }

        upstreamSum += upstreamValues[upstreamIndex];
        downstreamSum += downstreamValues[downstreamIndex];
        pairCount++;
    }

    if (pairCount < MIN_O2_CORRELATION_PAIRS)
    {
        return false;
    }

    double upstreamMean = upstreamSum / pairCount;

    double downstreamMean = downstreamSum / pairCount;

    double covarianceSum = 0.0;
    double upstreamDifferenceSum = 0.0;
    double downstreamDifferenceSum = 0.0;

    // Second pass calculates Pearson correlation
    for (int upstreamIndex = 0; upstreamIndex < static_cast<int>(CASE_SIZE); upstreamIndex++)
    {
        int downstreamIndex = upstreamIndex - downstreamLag;

        if (downstreamIndex < 0 || downstreamIndex >= static_cast<int>(CASE_SIZE))
        {
            continue;
        }

        if (!upstreamValid[upstreamIndex] || !downstreamValid[downstreamIndex] || !isfinite(upstreamValues[upstreamIndex]) || !isfinite(downstreamValues[downstreamIndex]))
        {
            continue;
        }

        double upstreamDifference = upstreamValues[upstreamIndex] - upstreamMean;

        double downstreamDifference = downstreamValues[downstreamIndex] - downstreamMean;

        covarianceSum += upstreamDifference * downstreamDifference;

        upstreamDifferenceSum += upstreamDifference * upstreamDifference;

        downstreamDifferenceSum += downstreamDifference * downstreamDifference;
    }

    // Cannot calculate correlation if one signal did not change
    if (upstreamDifferenceSum <= 0.0 || downstreamDifferenceSum <= 0.0)
    {
        return false;
    }

    correlationOutput = static_cast<float>(covarianceSum / sqrt(upstreamDifferenceSum * downstreamDifferenceSum));

    return isfinite(correlationOutput);
}


// Checks lag -3 through +3 and keeps the largest absolute correlation
bool calculateMaximumLaggedCorrelation(const float upstreamValues[CASE_SIZE], const bool upstreamValid[CASE_SIZE], const float downstreamValues[CASE_SIZE], const bool downstreamValid[CASE_SIZE], float& maximumCorrelationOutput)
{
    bool correlationFound = false;

    float maximumCorrelation = 0.0f;


    for (int lag = -MAX_O2_CORRELATION_LAG; lag <= MAX_O2_CORRELATION_LAG; lag++)
    {
        float currentCorrelation = NAN;

        if (!calculateCorrelationAtLag(upstreamValues, upstreamValid, downstreamValues, downstreamValid, lag, currentCorrelation))
        {
            continue;
        }


        float absoluteCorrelation = fabs(currentCorrelation);


        if (!correlationFound || absoluteCorrelation > maximumCorrelation)
        {
            maximumCorrelation = absoluteCorrelation;

            correlationFound = true;
        }
    }


    if (!correlationFound)
    {
        return false;
    }


    maximumCorrelationOutput = maximumCorrelation;

    return true;
}



// Catalyst feature calculation
// ------------------------------------------------------------

// Creates the 5 final V4 catalyst model features
CatalystCaseFeatures calculateCatalystFeatures(const VehicleSample samples[CASE_SIZE])
{
    CatalystCaseFeatures result = {};

    result.ready = false;
    result.selectedBank = 0;

    result.maximumLaggedCorrelation = NAN;
    result.maximumLaggedCorrelationValid = false;


    float upstreamB1VoltageValues[CASE_SIZE];
    bool upstreamB1VoltageValid[CASE_SIZE];

    float upstreamB1EquivalenceValues[CASE_SIZE];
    bool upstreamB1EquivalenceValid[CASE_SIZE];

    float downstreamB1VoltageValues[CASE_SIZE];
    bool downstreamB1VoltageValid[CASE_SIZE];


    float upstreamB2VoltageValues[CASE_SIZE];
    bool upstreamB2VoltageValid[CASE_SIZE];

    float upstreamB2EquivalenceValues[CASE_SIZE];
    bool upstreamB2EquivalenceValid[CASE_SIZE];

    float downstreamB2VoltageValues[CASE_SIZE];
    bool downstreamB2VoltageValid[CASE_SIZE];


    for (size_t i = 0; i < CASE_SIZE; i++)
    {
        upstreamB1VoltageValues[i] = samples[i].o2B1S1Voltage;
        upstreamB1VoltageValid[i] = samples[i].o2B1S1VoltageValid;

        upstreamB1EquivalenceValues[i] = samples[i].o2B1S1EquivalenceRatio;
        upstreamB1EquivalenceValid[i] = samples[i].o2B1S1EquivalenceRatioValid;

        downstreamB1VoltageValues[i] = samples[i].o2B1S2Voltage;
        downstreamB1VoltageValid[i] = samples[i].o2B1S2VoltageValid;


        upstreamB2VoltageValues[i] = samples[i].o2B2S1Voltage;
        upstreamB2VoltageValid[i] = samples[i].o2B2S1VoltageValid;

        upstreamB2EquivalenceValues[i] = samples[i].o2B2S1EquivalenceRatio;
        upstreamB2EquivalenceValid[i] = samples[i].o2B2S1EquivalenceRatioValid;

        downstreamB2VoltageValues[i] = samples[i].o2B2S2Voltage;
        downstreamB2VoltageValid[i] = samples[i].o2B2S2VoltageValid;
    }


    ValueStatistics upstreamB1Voltage = calculateStatistics(upstreamB1VoltageValues, upstreamB1VoltageValid);

    ValueStatistics upstreamB1Equivalence = calculateStatistics(upstreamB1EquivalenceValues, upstreamB1EquivalenceValid);

    ValueStatistics downstreamB1Voltage = calculateStatistics(downstreamB1VoltageValues, downstreamB1VoltageValid);


    ValueStatistics upstreamB2Voltage = calculateStatistics(upstreamB2VoltageValues, upstreamB2VoltageValid);

    ValueStatistics upstreamB2Equivalence = calculateStatistics(upstreamB2EquivalenceValues, upstreamB2EquivalenceValid);

    ValueStatistics downstreamB2Voltage = calculateStatistics(downstreamB2VoltageValues, downstreamB2VoltageValid);


    CatalystBankFeatures bank1 = calculateCatalystBankFeatures(upstreamB1Voltage, upstreamB1Equivalence, downstreamB1VoltageValues, downstreamB1VoltageValid, downstreamB1Voltage);


    CatalystBankFeatures bank2 = calculateCatalystBankFeatures(upstreamB2Voltage, upstreamB2Equivalence, downstreamB2VoltageValues, downstreamB2VoltageValid, downstreamB2Voltage);


    if (!bank1.available && !bank2.available)
    {
        return result;
    }


    CatalystBankFeatures selectedBank;

    const float* selectedUpstreamValues = nullptr;
    const bool* selectedUpstreamValid = nullptr;

    const float* selectedDownstreamValues = nullptr;
    const bool* selectedDownstreamValid = nullptr;


    // V4 was validated using Bank 1 catalyst data
    // Bank 2 is used only when Bank 1 does not have a useful signal
    if (bank1.available)
    {
        selectedBank = bank1;

        result.selectedBank = 1;

        if (bank1.useUpstreamVoltage)
        {
            selectedUpstreamValues = upstreamB1VoltageValues;

            selectedUpstreamValid = upstreamB1VoltageValid;
        }
        else
        {
            selectedUpstreamValues = upstreamB1EquivalenceValues;

            selectedUpstreamValid = upstreamB1EquivalenceValid;
        }

        selectedDownstreamValues = downstreamB1VoltageValues;

        selectedDownstreamValid = downstreamB1VoltageValid;
    }
    else
    {
        selectedBank = bank2;

        result.selectedBank = 2;

        if (bank2.useUpstreamVoltage)
        {
            selectedUpstreamValues = upstreamB2VoltageValues;

            selectedUpstreamValid = upstreamB2VoltageValid;
        }
        else
        {
            selectedUpstreamValues = upstreamB2EquivalenceValues;

            selectedUpstreamValid = upstreamB2EquivalenceValid;
        }

        selectedDownstreamValues = downstreamB2VoltageValues;

        selectedDownstreamValid = downstreamB2VoltageValid;
    }


    float maximumCorrelation = NAN;


    if (!calculateMaximumLaggedCorrelation(selectedUpstreamValues, selectedUpstreamValid, selectedDownstreamValues, selectedDownstreamValid, maximumCorrelation))
    {
        return result;
    }


    result.maximumLaggedCorrelation = maximumCorrelation;

    result.maximumLaggedCorrelationValid = true;


    // Final V4 catalyst feature order

    result.values[0] = maximumCorrelation;

    result.values[1] = selectedBank.downstreamStandardDeviation;

    result.values[2] = selectedBank.downstreamRange;

    result.values[3] = selectedBank.downstreamMeanAbsoluteStep;

    result.values[4] = selectedBank.downstreamStepStandardDeviation;


    result.ready = true;

    return result;
}


// Charging feature calculation
// ------------------------------------------------------------

// Creates the 7 charging features
ChargingCaseFeatures calculateChargingFeatures(const VehicleSample samples[CASE_SIZE])
{
    ChargingCaseFeatures result = {};

    result.ready = false;

    float voltageValues[CASE_SIZE];
    bool voltageValid[CASE_SIZE];

    int belowChargingCount = 0;
    int aboveChargingCount = 0;


    for (size_t i = 0; i < CASE_SIZE; i++)
    {
        voltageValues[i] = samples[i].controlModuleVoltage;

        voltageValid[i] = samples[i].controlModuleVoltageValid;


        if (!voltageValid[i] || !isfinite(voltageValues[i]))
        {
            continue;
        }


        float voltage = voltageValues[i];


        if (voltage < 13.0f)
        {
            belowChargingCount++;
        }


        if (voltage > 14.8f)
        {
            aboveChargingCount++;
        }
    }


    ValueStatistics voltage = calculateStatistics(voltageValues, voltageValid);


    if (voltage.validCount < MIN_VALID_SAMPLES)
    {
        return result;
    }


    // Store the valid voltage readings in order
    float validVoltageValues[CASE_SIZE];

    size_t validVoltageCount = 0;


    for (size_t i = 0; i < CASE_SIZE; i++)
    {
        if (!voltageValid[i] || !isfinite(voltageValues[i]))
        {
            continue;
        }


        validVoltageValues[validVoltageCount] = voltageValues[i];

        validVoltageCount++;
    }


    float voltageSteps[CASE_SIZE - 1];

    size_t stepCount = 0;

    float maximumAbsoluteStep = 0.0f;

    double stepSum = 0.0;


    // Calculate the change between consecutive valid voltage readings
    for (size_t i = 1; i < validVoltageCount; i++)
    {
        float currentStep = validVoltageValues[i] - validVoltageValues[i - 1];


        voltageSteps[stepCount] = currentStep;


        float absoluteStep = fabs(currentStep);


        if (absoluteStep > maximumAbsoluteStep)
        {
            maximumAbsoluteStep = absoluteStep;
        }


        stepSum += currentStep;

        stepCount++;
    }


    if (stepCount == 0)
    {
        return result;
    }


    double meanStep = stepSum / stepCount;

    double squaredStepDifferenceSum = 0.0;


    // Population STD of the signed voltage changes
    for (size_t i = 0; i < stepCount; i++)
    {
        double difference = voltageSteps[i] - meanStep;


        squaredStepDifferenceSum += difference * difference;
    }


    float stepStandardDeviation = static_cast<float>(sqrt(squaredStepDifferenceSum / stepCount));


    // Final V4 charging feature order

    result.values[0] = voltage.mean;

    result.values[1] = voltage.minimum;

    result.values[2] = voltage.maximum;

    result.values[3] = static_cast<float>(belowChargingCount);

    result.values[4] = static_cast<float>(aboveChargingCount);

    result.values[5] = maximumAbsoluteStep;

    result.values[6] = stepStandardDeviation;


    result.ready = true;

    return result;
}

// Creates a second set of charging measurements used only by
// the rule-based diagnostic logic. The Isolation Forest still
// receives the original unmodified V4 charging features.
ChargingDiagnosticContext calculateChargingDiagnosticContext(const VehicleSample samples[CASE_SIZE])
{
    ChargingDiagnosticContext result = {};

    result.ready = false;

    result.engineStopDetected = false;
    result.engineRestartDetected = false;

    result.evaluatedSampleCount = 0;

    result.meanVoltage = NAN;
    result.belowChargingCount = 0.0f;
    result.aboveChargingCount = 0.0f;
    result.maximumAbsoluteStep = 0.0f;
    result.stepStandardDeviation = NAN;


    double voltageSum = 0.0;

    float voltageSteps[CASE_SIZE - 1];

    size_t stepCount = 0;

    double stepSum = 0.0;

    bool previousSampleEligible = false;

    float previousVoltage = NAN;


    for (size_t i = 0; i < CASE_SIZE; i++)
    {
        const VehicleSample& sample = samples[i];

        bool rpmAvailable = sample.rpmValid && isfinite(sample.rpm);

        bool voltageAvailable = sample.controlModuleVoltageValid && isfinite(sample.controlModuleVoltage);


        // Without RPM, the firmware cannot safely determine whether
        // the engine is running or currently in an auto-stop state.
        // Do not use that sample for rule-based charging decisions.
        if (!rpmAvailable)
        {
            previousSampleEligible = false;
            continue;
        }


        // RPM at or below the threshold means the engine is stopped
        // or cranking. Battery-level voltage during this state is
        // expected and must not be treated as low charging voltage.
        if (sample.rpm <= CHARGING_ENGINE_STOP_RPM_THRESHOLD)
        {
            result.engineStopDetected = true;

            chargingPreviousEngineStopped = true;

            previousSampleEligible = false;

            continue;
        }


        // The first running-RPM sample after an engine-stop state
        // identifies an engine restart. Begin a short suppression
        // period so the normal restart voltage transient is ignored.
        if (chargingPreviousEngineStopped)
        {
            result.engineRestartDetected = true;

            chargingPreviousEngineStopped = false;

            chargingRestartSuppressionSamplesRemaining = CHARGING_RESTART_SUPPRESSION_SAMPLES;
        }


        // Ignore the restart sample and the configured number of
        // restart-recovery samples before evaluating charging behavior.
        if (chargingRestartSuppressionSamplesRemaining > 0)
        {
            chargingRestartSuppressionSamplesRemaining--;

            previousSampleEligible = false;

            continue;
        }


        if (!voltageAvailable)
        {
            previousSampleEligible = false;
            continue;
        }


        float voltage = sample.controlModuleVoltage;

        voltageSum += voltage;

        result.evaluatedSampleCount++;


        if (voltage < CHARGING_INSPECT_LOW_MEAN)
        {
            result.belowChargingCount++;
        }


        if (voltage > CHARGING_INSPECT_HIGH_MEAN)
        {
            result.aboveChargingCount++;
        }


        // Only calculate voltage steps between adjacent samples that
        // were both eligible. This prevents comparing a pre-stop
        // sample directly with a post-restart sample.
        if (previousSampleEligible)
        {
            float currentStep = voltage - previousVoltage;

            voltageSteps[stepCount] = currentStep;

            float absoluteStep = fabs(currentStep);

            if (absoluteStep > result.maximumAbsoluteStep)
            {
                result.maximumAbsoluteStep = absoluteStep;
            }

            stepSum += currentStep;

            stepCount++;
        }


        previousVoltage = voltage;

        previousSampleEligible = true;
    }


    // Use the same minimum-valid-sample requirement already used
    // elsewhere in P.I.S.T.O.N. If auto-stop activity leaves too
    // little steady engine-running data, the diagnostic logic waits
    // for another case instead of making a charging fault decision.
    if (result.evaluatedSampleCount < MIN_VALID_SAMPLES)
    {
        return result;
    }


    result.meanVoltage = static_cast<float>(voltageSum / result.evaluatedSampleCount);


    if (stepCount == 0)
    {
        return result;
    }


    double meanStep = stepSum / stepCount;

    double squaredStepDifferenceSum = 0.0;


    for (size_t i = 0; i < stepCount; i++)
    {
        double difference = voltageSteps[i] - meanStep;

        squaredStepDifferenceSum += difference * difference;
    }


    result.stepStandardDeviation = static_cast<float>(sqrt(squaredStepDifferenceSum / stepCount));

    result.ready = true;

    return result;
}

// Feature printing
// ------------------------------------------------------------

// Prints one feature
void printFeature(const char* featureName, float featureValue)
{
    Serial.print(featureName);
    Serial.print(": ");
    Serial.println(featureValue, 6);
}

// BLE connection callbacks
// ------------------------------------------------------------

class PistonBleServerCallbacks : public BLEServerCallbacks
{
    void onConnect(BLEServer* server) override
    {
        bleDeviceConnected = true;

        Serial.println();
        Serial.println("BLE client connected.");
    }

    void onDisconnect(BLEServer* server) override
    {
        bleDeviceConnected = false;

        // A lost Bluetooth connection automatically
        // pauses logging so completed cases cannot be
        // silently lost while the app is disconnected.
        pendingLoggingCommand.store(LOGGING_COMMAND_PAUSE);

        Serial.println();
        Serial.println("BLE client disconnected.");
        Serial.println("Logging pause queued because BLE disconnected.");

        server->startAdvertising();

        Serial.println("BLE advertising restarted.");
    }
};


// BLE command callbacks
// ------------------------------------------------------------
class PistonBleCommandCallbacks : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic* characteristic) override
    {
        std::string command = characteristic->getValue();

        if (command.empty())
        {
            return;
        }

        Serial.println();
        Serial.print("BLE command received: ");
        Serial.println(command.c_str());


        if (command == "start_logging")
        {
            pendingLoggingCommand.store(LOGGING_COMMAND_START);

            Serial.println("Start logging command queued.");
        }
        else if (command == "pause_logging")
        {
            pendingLoggingCommand.store(LOGGING_COMMAND_PAUSE);

            Serial.println("Pause logging command queued.");
        }
        else if (command == "get_logging_status")
        {
            pendingLoggingCommand.store(LOGGING_COMMAND_STATUS);

            Serial.println("Logging status request queued.");
        }
        else if (command == "read_dtcs")
        {
            pendingDtcRequest.store(true);
            Serial.println("DTC scan request queued.");
        }
        else if (command == "read_vehicle_info")
        {
            pendingVinRequest.store(true);
            Serial.println("Vehicle information request queued.");
        }
        else
        {
            Serial.print("Unknown BLE command: ");

            Serial.println(command.c_str());
        }
    }
};

// JSON conversion
// ------------------------------------------------------------

// Adds a floating-point value to JSON.
// Invalid or non-finite values are written as JSON null.
void addJsonFloat(JsonObject object, const char* key, float value, bool valid)
{
    if (valid && isfinite(value))
    {
        object[key] = value;
    }
    else
    {
        object[key] = nullptr;
    }
}


// Adds a floating-point value when no separate valid flag exists.
void addJsonFloat(JsonObject object, const char* key, float value)
{
    if (isfinite(value))
    {
        object[key] = value;
    }
    else
    {
        object[key] = nullptr;
    }
}


// Builds one complete JSON document from a completed case.
void buildCompletedCaseJson(const CompletedCaseToTransmit& completedCase, JsonDocument& doc)
{
    doc.clear();

    doc["type"] = "case_complete";
    doc["protocol_version"] = 1;
    doc["case_id"] = completedCase.caseId;
    doc["bank2_availability_known"] = bank2AvailabilityKnown;
    doc["has_bank_2"] = hasBank2;


    // Raw 20-sample case
    // --------------------------------------------------------

    JsonArray samplesJson = doc["samples"].to<JsonArray>();

    for (size_t i = 0; i < CASE_SIZE; i++)
    {
        const VehicleSample& sample = completedCase.samples[i];

        JsonObject sampleJson = samplesJson.add<JsonObject>();

        sampleJson["sample_index"] = i;
        sampleJson["time_ms"] = sample.timestampMs;

        addJsonFloat(sampleJson, "engine_load", sample.engineLoad, sample.engineLoadValid);

        addJsonFloat(sampleJson, "stft_b1", sample.stftB1, sample.stftB1Valid);

        addJsonFloat(sampleJson, "ltft_b1", sample.ltftB1, sample.ltftB1Valid);

        addJsonFloat(sampleJson, "stft_b2", sample.stftB2, sample.stftB2Valid);

        addJsonFloat(sampleJson, "ltft_b2", sample.ltftB2, sample.ltftB2Valid);

        addJsonFloat(sampleJson, "rpm", sample.rpm, sample.rpmValid);

        addJsonFloat(sampleJson, "o2_b1s1_voltage", sample.o2B1S1Voltage, sample.o2B1S1VoltageValid);

        addJsonFloat(sampleJson, "o2_b1s2_voltage", sample.o2B1S2Voltage, sample.o2B1S2VoltageValid);

        addJsonFloat(sampleJson, "o2_b1s1_equiv", sample.o2B1S1EquivalenceRatio, sample.o2B1S1EquivalenceRatioValid);

        addJsonFloat(sampleJson, "o2_b2s1_voltage", sample.o2B2S1Voltage, sample.o2B2S1VoltageValid);

        addJsonFloat(sampleJson, "o2_b2s2_voltage", sample.o2B2S2Voltage, sample.o2B2S2VoltageValid);

        addJsonFloat(sampleJson, "o2_b2s1_equiv", sample.o2B2S1EquivalenceRatio, sample.o2B2S1EquivalenceRatioValid);

        addJsonFloat(sampleJson, "control_module_voltage", sample.controlModuleVoltage, sample.controlModuleVoltageValid);
    }


    // Diagnostic outputs
    // --------------------------------------------------------

    JsonObject outputsJson = doc["outputs"].to<JsonObject>();


    // Fuel
    // --------------------------------------------------------

    JsonObject fuelJson = outputsJson["fuel"].to<JsonObject>();

    fuelJson["available"] = completedCase.fuel.available;

    fuelJson["selected_bank"] = completedCase.fuel.selectedBank;

    JsonObject fuelFeaturesJson = fuelJson["features"].to<JsonObject>();

    for (size_t i = 0; i < 9; i++)
    {
        addJsonFloat(fuelFeaturesJson, FUEL_FEATURE_NAMES[i], completedCase.fuel.features[i]);
    }

    addJsonFloat(fuelJson, "anomaly_score", completedCase.fuel.anomalyScore);

    fuelJson["anomaly_threshold"] = completedCase.fuel.anomalyThreshold;

    fuelJson["anomaly_alert"] = completedCase.fuel.anomalyAlert;

    JsonObject fuelThresholdsJson = fuelJson["thresholds"].to<JsonObject>();

    fuelThresholdsJson["monitor_total_trim"] = FUEL_KEEP_EYE_TRIM_THRESHOLD;

    fuelThresholdsJson["inspect_total_trim"] = FUEL_INSPECT_TRIM_THRESHOLD;

    fuelThresholdsJson["inspect_score"] = FUEL_INSPECT_SCORE_THRESHOLD;
        
    fuelJson["condition"] = completedCase.fuel.condition;

    fuelJson["action_level"] = completedCase.fuel.actionLevel;

    fuelJson["evidence_source"] = completedCase.fuel.evidenceSource;

    fuelJson["description"] = completedCase.fuel.description;


    // Catalyst
    // --------------------------------------------------------

    JsonObject catalystJson = outputsJson["catalyst"].to<JsonObject>();

    catalystJson["available"] = completedCase.catalyst.available;

    catalystJson["selected_bank"] = completedCase.catalyst.selectedBank;

    JsonObject catalystFeaturesJson = catalystJson["features"].to<JsonObject>();

    for (size_t i = 0; i < 5; i++)
    {
        addJsonFloat(catalystFeaturesJson, CATALYST_FEATURE_NAMES[i], completedCase.catalyst.features[i]);
    }

    addJsonFloat(catalystJson, "anomaly_score", completedCase.catalyst.anomalyScore);

    catalystJson["anomaly_threshold"] = completedCase.catalyst.anomalyThreshold;

    catalystJson["anomaly_alert"] = completedCase.catalyst.isolationForestAlert;

    addJsonFloat(catalystJson, "maximum_lagged_correlation", completedCase.catalyst.maximumLaggedCorrelation, completedCase.catalyst.maximumLaggedCorrelationValid);

    catalystJson["mirroring_this_case"] = completedCase.catalyst.mirroringThisCase;

    catalystJson["consecutive_mirroring_cases"] = completedCase.catalyst.consecutiveMirroringCases;

    catalystJson["persistent_mirroring"] = completedCase.catalyst.persistentMirroring;

    catalystJson["strong_mirroring_this_case"] = completedCase.catalyst.strongMirroringThisCase;

    catalystJson["consecutive_strong_mirroring_cases"] = completedCase.catalyst.consecutiveStrongMirroringCases;

    catalystJson["strong_persistent_mirroring"] = completedCase.catalyst.strongPersistentMirroring;

    catalystJson["hybrid_alert"] = completedCase.catalyst.hybridAlert;

    JsonObject catalystThresholdsJson = catalystJson["thresholds"].to<JsonObject>();

    catalystThresholdsJson["mirroring"] = CATALYST_CORRELATION_THRESHOLD;

    catalystThresholdsJson["strong_mirroring"] =  CATALYST_STRONG_CORRELATION_THRESHOLD;

    catalystThresholdsJson["required_consecutive_cases"] =  CATALYST_REQUIRED_CONSECUTIVE_CASES;

    catalystThresholdsJson["strong_required_consecutive_cases"] = CATALYST_STRONG_REQUIRED_CONSECUTIVE_CASES;

    catalystThresholdsJson["inspect_score"] = CATALYST_INSPECT_SCORE_THRESHOLD;

    catalystJson["condition"] = completedCase.catalyst.condition;

    catalystJson["action_level"] = completedCase.catalyst.actionLevel;

    catalystJson["evidence_source"] = completedCase.catalyst.evidenceSource;

    catalystJson["description"] = completedCase.catalyst.description;


    // Charging
    // --------------------------------------------------------

    JsonObject chargingJson = outputsJson["charging"].to<JsonObject>();

    chargingJson["available"] = completedCase.charging.available;

    JsonObject chargingFeaturesJson = chargingJson["features"].to<JsonObject>();

    for (size_t i = 0; i < 7; i++)
    {
        addJsonFloat(chargingFeaturesJson, CHARGING_FEATURE_NAMES[i], completedCase.charging.features[i]);
    }

    addJsonFloat(chargingJson, "anomaly_score", completedCase.charging.anomalyScore);

    chargingJson["anomaly_threshold"] = completedCase.charging.anomalyThreshold;

    chargingJson["anomaly_alert"] = completedCase.charging.anomalyAlert;

    chargingJson["strong_charging_case"] = completedCase.charging.strongChargingCase;

    chargingJson["consecutive_strong_cases"] = completedCase.charging.consecutiveStrongCases;

    JsonObject chargingThresholdsJson = chargingJson["thresholds"].to<JsonObject>();

    chargingThresholdsJson["monitor_low_count"] = CHARGING_KEEP_LOW_COUNT;

    chargingThresholdsJson["monitor_high_count"] = CHARGING_KEEP_HIGH_COUNT;

    chargingThresholdsJson["monitor_max_step"] = CHARGING_KEEP_MAX_STEP;

    chargingThresholdsJson["monitor_step_std"] = CHARGING_KEEP_STEP_STD;

    chargingThresholdsJson["inspect_low_mean"] = CHARGING_INSPECT_LOW_MEAN;

    chargingThresholdsJson["inspect_high_mean"] = CHARGING_INSPECT_HIGH_MEAN;

    chargingThresholdsJson["inspect_low_count"] = CHARGING_INSPECT_LOW_COUNT;

    chargingThresholdsJson["inspect_high_count"] = CHARGING_INSPECT_HIGH_COUNT;

    chargingThresholdsJson["inspect_max_step"] = CHARGING_INSPECT_MAX_STEP;

    chargingThresholdsJson["inspect_step_std"] = CHARGING_INSPECT_STEP_STD;

    chargingThresholdsJson["inspect_required_consecutive_cases"] = CHARGING_INSPECT_REQUIRED_CONSECUTIVE_CASES;

    chargingJson["condition"] = completedCase.charging.condition;

    chargingJson["action_level"] = completedCase.charging.actionLevel;

    chargingJson["evidence_source"] = completedCase.charging.evidenceSource; 
    
    chargingJson["description"] = completedCase.charging.description;
}

// Sends one complete JSON payload through BLE notifications
void sendJsonPayloadOverBle(const String& jsonPayload)
{
    if (!bleDeviceConnected)
    {
        Serial.println("BLE client not connected. JSON not sent.");
        return;
    }

    if (bleServer == nullptr || bleTxCharacteristic == nullptr)
    {
        Serial.println("BLE is not initialized. JSON not sent.");
        return;
    }


    uint16_t connectionId = bleServer->getConnId();

    uint16_t negotiatedMtu = bleServer->getPeerMTU(connectionId);


    // ATT notifications can carry MTU - 3 bytes.
    // Default BLE MTU is 23, which gives 20 payload bytes.
    constexpr size_t BLE_SAFE_CHUNK_SIZE = 180;

    size_t maximumChunkSize = 20;

    if (negotiatedMtu > 3) maximumChunkSize = negotiatedMtu - 3;

    if (maximumChunkSize > BLE_SAFE_CHUNK_SIZE) maximumChunkSize = BLE_SAFE_CHUNK_SIZE;
    
    Serial.println();
    Serial.println("--- BLE JSON TRANSMISSION ---");

    Serial.print("JSON length: ");
    Serial.println(jsonPayload.length());

    Serial.print("Negotiated MTU: ");
    Serial.println(negotiatedMtu);

    Serial.print("Maximum chunk size: ");
    Serial.println(maximumChunkSize);


    size_t offset = 0;

    static uint8_t chunkBuffer[BLE_SAFE_CHUNK_SIZE];

    while (offset < jsonPayload.length())
    {
        if (!bleDeviceConnected)
        {
            Serial.println("BLE disconnected during transmission.");
            return;
        }


        size_t remainingBytes = jsonPayload.length() - offset;

        size_t chunkSize = maximumChunkSize;

        if (remainingBytes < maximumChunkSize) chunkSize = remainingBytes;


        memcpy(chunkBuffer, jsonPayload.c_str() + offset, chunkSize);

        bleTxCharacteristic->setValue(chunkBuffer, chunkSize);

        bleTxCharacteristic->notify();


        offset += chunkSize;


        // Give the BLE stack time to process the notification
        delay(10);
    }


    // Newline marks the end of one complete JSON message.
    uint8_t endOfMessage = '\n';

    bleTxCharacteristic->setValue(&endOfMessage, 1);

    bleTxCharacteristic->notify();


    Serial.println("BLE JSON transmission complete.");
}

// Completed case processing
// ------------------------------------------------------------

// Calculates and prints all features after 20 samples
void processCompletedCase(const VehicleSample samples[CASE_SIZE])
{

    Serial.println();
    Serial.println("=== 20-SAMPLE CASE COMPLETE ===");


    FuelCaseFeatures fuelFeatures = calculateFuelFeatures(samples);

    CatalystCaseFeatures catalystFeatures = calculateCatalystFeatures(samples);

    ChargingCaseFeatures chargingFeatures = calculateChargingFeatures(samples);



    // Holds the final fuel results that will eventually be sent to the GUI
    FuelResultsToTransmit fuelResults = {};

    fuelResults.available = false;
    fuelResults.selectedBank = 0;

    for (size_t i = 0; i < 9; i++)
    {
        fuelResults.features[i] = NAN;
    }

    fuelResults.anomalyScore = NAN;
    fuelResults.anomalyThreshold = FUEL_ANOMALY_THRESHOLD;
    fuelResults.anomalyAlert = false;

    fuelResults.condition = "Unavailable";
    fuelResults.actionLevel = "Unavailable";

    fuelResults.evidenceSource = "None";
    fuelResults.description[0] = '\0';

    CatalystResultsToTransmit catalystResults = {};

    catalystResults.available = false;
    catalystResults.selectedBank = 0;

    for (size_t i = 0; i < 5; i++) catalystResults.features[i] = NAN;

    catalystResults.anomalyScore = NAN;
    catalystResults.anomalyThreshold = CATALYST_ANOMALY_THRESHOLD;
    catalystResults.isolationForestAlert = false;

    catalystResults.maximumLaggedCorrelation = NAN;
    catalystResults.maximumLaggedCorrelationValid = false;

    catalystResults.mirroringThisCase = false;
    catalystResults.consecutiveMirroringCases = 0;
    catalystResults.persistentMirroring = false;

    catalystResults.strongMirroringThisCase = false;
    catalystResults.consecutiveStrongMirroringCases = 0;
    catalystResults.strongPersistentMirroring = false;

    catalystResults.hybridAlert = false;

    catalystResults.condition = "Unavailable";
    catalystResults.actionLevel = "Unavailable";
    catalystResults.evidenceSource = "None";
    catalystResults.description[0] = '\0';

    ChargingResultsToTransmit chargingResults = {};

    chargingResults.available = false;

    for (size_t i = 0; i < 7; i++)
    {
        chargingResults.features[i] = NAN;
    }

    chargingResults.anomalyScore = NAN;
    chargingResults.anomalyThreshold = CHARGING_ANOMALY_THRESHOLD;
    chargingResults.anomalyAlert = false;

    chargingResults.strongChargingCase = false;
    chargingResults.consecutiveStrongCases = 0;

    chargingResults.condition = "Unavailable";
    chargingResults.actionLevel = "Unavailable";
    chargingResults.evidenceSource = "None";
    chargingResults.description[0] = '\0';


    // Fuel model features
    // --------------------------------------------------------

    Serial.println();
    Serial.println("--- FUEL FEATURES ---");

    if (!fuelFeatures.ready)
    {
        Serial.println("Fuel feature data is incomplete.");
    }

    else
    {

        fuelResults.available = true;
        fuelResults.selectedBank = fuelFeatures.selectedBank;

        for (size_t i = 0; i < 9; i++) fuelResults.features[i] = fuelFeatures.values[i];

        Serial.print("Selected fuel bank: B");
        Serial.println(fuelFeatures.selectedBank);

        const char* fuelFeatureNames[9] =
        {
            "ENGINE_RPM_MEAN",
            "ENGINE_LOAD_PCT_MEAN",

            "SELECTED_BANK_STFT_PCT_MEAN",
            "SELECTED_BANK_STFT_PCT_STD",

            "SELECTED_BANK_LTFT_PCT_MEAN",
            "SELECTED_BANK_LTFT_PCT_STD",

            "SELECTED_BANK_TOTAL_TRIM_PCT_MEAN",

            "SELECTED_BANK_ABS_TOTAL_TRIM_PCT_MEAN",
            "SELECTED_BANK_ABS_TOTAL_TRIM_PCT_MIN"
        };

    for (size_t i = 0; i < 9; i++) printFeature(fuelFeatureNames[i], fuelFeatures.values[i]);
    
    }


    // Fuel model result
    // --------------------------------------------------------

    if (fuelFeatures.ready)
    {
        float fuelScore = pistonIsolationForestScore(
            FUEL_NODES,
            FUEL_TREE_OFFSETS,
            FUEL_TREE_COUNT,
            FUEL_NORMALIZATION,
            FUEL_MODEL_OFFSET,
            fuelFeatures.values,
            FUEL_IMPUTER_MEDIANS);


        bool fuelAlert = pistonIsolationForestAlert(fuelScore, FUEL_ANOMALY_THRESHOLD);
        
        // Store the Isolation Forest result for later transmission
        fuelResults.anomalyScore = fuelScore;
        fuelResults.anomalyAlert = fuelAlert;

        float totalTrimMagnitude = fabs(fuelFeatures.values[6]);


        // Diagnostic logic evidence begins when total fuel trim
        // reaches the monitoring threshold
        bool fuelDiagnosticEvidence = totalTrimMagnitude >= FUEL_KEEP_EYE_TRIM_THRESHOLD;


        // Keep an Eye Out requires agreement between
        // the Isolation Forest and diagnostic logic
        bool fuelKeepEvidence = fuelAlert && fuelDiagnosticEvidence;


        // Inspect Soon also requires both evidence sources.
        // Severity is reached by either a large fuel-trim
        // deviation or a stronger Isolation Forest score.
        bool fuelInspectEvidence = fuelAlert && (totalTrimMagnitude >= FUEL_INSPECT_TRIM_THRESHOLD || (fuelScore >= FUEL_INSPECT_SCORE_THRESHOLD && fuelDiagnosticEvidence));


    if (fuelAlert && fuelDiagnosticEvidence)
    {
        fuelResults.evidenceSource = "Both";
    }
    else if (fuelAlert)
    {
        fuelResults.evidenceSource = "Isolation Forest";
    }
    else if (fuelDiagnosticEvidence)
    {
        fuelResults.evidenceSource = "Diagnostic Logic";
    }
    else
    {
        fuelResults.evidenceSource = "None";
    }

        Serial.println();
        Serial.println("--- FUEL MODEL RESULT ---");

        Serial.print("Anomaly score: ");
        Serial.println(fuelScore, 6);

        Serial.print("Threshold: ");
        Serial.println(FUEL_ANOMALY_THRESHOLD, 6);

        Serial.print("Alert: ");

        if (fuelAlert)
        {
            Serial.println("YES");
        }
        else
        {
            Serial.println("NO");
        }


        // Determine the fuel condition and store it for transmission
        if (!fuelKeepEvidence) fuelResults.condition = "Normal";
        
        else if (totalTrimMagnitude < FUEL_KEEP_EYE_TRIM_THRESHOLD)
        {
            // The model saw something unusual, but fuel trim itself
            // is still inside the healthy range
            fuelResults.condition = "Abnormal fuel behavior";
        }

        else if (fuelFeatures.values[6] > 0.0f) fuelResults.condition = "Lean fuel behavior";
        
        else if (fuelFeatures.values[6] < 0.0f) fuelResults.condition = "Rich fuel behavior";
        
        else fuelResults.condition = "Abnormal fuel behavior";
        


        // Keep the same Serial output
        Serial.print("Condition: ");
        Serial.println(fuelResults.condition);


        // Determine the fuel action level and store it for transmission
        if (fuelInspectEvidence)
        {
            fuelResults.actionLevel = "Inspect Soon";
        }
        else if (fuelKeepEvidence)
        {
            fuelResults.actionLevel = "Keep an Eye Out";
        }
        else
        {
            fuelResults.actionLevel = "Normal Operation";
        }


        // Keep the same Serial output
        Serial.print("Action Level: ");
        Serial.println(fuelResults.actionLevel);

        float signedTotalTrim = fuelFeatures.values[6];
        float diagnosticThreshold = FUEL_KEEP_EYE_TRIM_THRESHOLD;

        if (totalTrimMagnitude >= FUEL_INSPECT_TRIM_THRESHOLD) diagnosticThreshold = FUEL_INSPECT_TRIM_THRESHOLD;
        float trimAmountPastThreshold = totalTrimMagnitude - diagnosticThreshold;

        // 00 - Neither method detected abnormal behavior
        if (!fuelAlert && !fuelDiagnosticEvidence)
        {
            snprintf(
                fuelResults.description,
                sizeof(fuelResults.description),
                "Fuel delivery remained within the expected operating range. Average fuel correction was %+.1f%%, which remained within the +/-%.1f%% monitoring range.",
                signedTotalTrim,
                FUEL_KEEP_EYE_TRIM_THRESHOLD);
        }

        // 10 - Isolation Forest only
        else if (fuelAlert && !fuelDiagnosticEvidence)
        {
            snprintf(
                fuelResults.description,
                sizeof(fuelResults.description),
                "The predictive model noticed a fuel-delivery pattern that differed somewhat from learned normal behavior. Average fuel correction was %+.1f%% and remained within the expected +/-%.1f%% range, so the result remains Normal Operation.",
                signedTotalTrim,
                FUEL_KEEP_EYE_TRIM_THRESHOLD);
        }

        // 01 - Diagnostic logic only
        else if (!fuelAlert && fuelDiagnosticEvidence)
        {
            if (signedTotalTrim >= 0.0f)
            {
                snprintf(
                    fuelResults.description,
                    sizeof(fuelResults.description),
                    "The engine computer added an average of %.1f%% fuel, which was %.1f percentage points beyond the %.1f%% fuel-correction threshold for this case. The predictive model did not detect an abnormal overall fuel pattern, so the result remains Normal Operation.",
                    signedTotalTrim,
                    trimAmountPastThreshold,
                    diagnosticThreshold);
            }
            else
            {
                snprintf(
                    fuelResults.description,
                    sizeof(fuelResults.description),
                    "The engine computer removed an average of %.1f%% fuel, which was %.1f percentage points beyond the %.1f%% fuel-correction threshold for this case. The predictive model did not detect an abnormal overall fuel pattern, so the result remains Normal Operation.",
                    fabs(signedTotalTrim),
                    trimAmountPastThreshold,
                    diagnosticThreshold);
            }
        }

        // 11 - Both methods agree
        else
        {
            if (signedTotalTrim >= 0.0f)
            {
                snprintf(
                    fuelResults.description,
                    sizeof(fuelResults.description),
                    "Both the predictive model and measured fuel correction detected lean fuel behavior. The engine computer added an average of %.1f%% fuel, exceeding the %.1f%% threshold by %.1f percentage points.",
                    signedTotalTrim,
                    diagnosticThreshold,
                    trimAmountPastThreshold);
            }
            else
            {
                snprintf(
                    fuelResults.description,
                    sizeof(fuelResults.description),
                    "Both the predictive model and measured fuel correction detected rich fuel behavior. The engine computer removed an average of %.1f%% fuel, exceeding the %.1f%% threshold by %.1f percentage points.",
                    fabs(signedTotalTrim),
                    diagnosticThreshold,
                    trimAmountPastThreshold);
            }
        }

        Serial.println();
        Serial.println("--- FUEL TRANSMIT STRUCT CHECK ---");

        Serial.print("Available: ");
        if (fuelResults.available) Serial.println("YES");
        else Serial.println("NO");

        Serial.print("Selected bank: B");
        Serial.println(fuelResults.selectedBank);

        Serial.print("Stored anomaly score: ");
        Serial.println(fuelResults.anomalyScore, 6);

        Serial.print("Stored anomaly threshold: ");
        Serial.println(fuelResults.anomalyThreshold, 6);

        Serial.print("Stored alert: ");
        if (fuelResults.anomalyAlert) Serial.println("YES");
        else Serial.println("NO");

        Serial.print("Stored condition: ");
        Serial.println(fuelResults.condition);

        Serial.print("Stored action level: ");
        Serial.println(fuelResults.actionLevel);

    }



    // Catalyst model features
    // --------------------------------------------------------

    Serial.println();
    Serial.println("--- CATALYST FEATURES ---");

    if (!catalystFeatures.ready)
    {
        Serial.println("Catalyst feature data is incomplete.");
    }
    else
    {

        catalystResults.available = true;
        catalystResults.selectedBank = catalystFeatures.selectedBank;

        for (size_t i = 0; i < 5; i++) catalystResults.features[i] = catalystFeatures.values[i];

        catalystResults.maximumLaggedCorrelation = catalystFeatures.maximumLaggedCorrelation;

        catalystResults.maximumLaggedCorrelationValid = catalystFeatures.maximumLaggedCorrelationValid;
        Serial.print("Selected catalyst bank: B");
        Serial.println(catalystFeatures.selectedBank);

        const char* catalystFeatureNames[5] = {"CAT_MAX_LAGGED_CORR", "CAT_DOWNSTREAM_STD", "CAT_DOWNSTREAM_RANGE", "CAT_DOWNSTREAM_MEAN_ABS_STEP", "CAT_DOWNSTREAM_STEP_STD"};


        for (size_t i = 0; i < 5; i++)
        {
            printFeature(catalystFeatureNames[i], catalystFeatures.values[i]);
        }
    }


    // Catalyst model result
    // --------------------------------------------------------

    if (catalystFeatures.ready)
    {
        float catalystScore = pistonIsolationForestScore(
            CATALYST_NODES,
            CATALYST_TREE_OFFSETS,
            CATALYST_TREE_COUNT,
            CATALYST_NORMALIZATION,
            CATALYST_MODEL_OFFSET,
            catalystFeatures.values,
            CATALYST_IMPUTER_MEDIANS);


        bool catalystIfAlert = pistonIsolationForestAlert(catalystScore, CATALYST_ANOMALY_THRESHOLD);

        catalystResults.anomalyScore = catalystScore;
        catalystResults.isolationForestAlert = catalystIfAlert;

        // Basic mirroring rule
        bool catalystMirroringThisCase = catalystFeatures.maximumLaggedCorrelationValid && catalystFeatures.maximumLaggedCorrelation >= CATALYST_CORRELATION_THRESHOLD;


        uint8_t currentMirroringCount = 0;


        if (catalystFeatures.selectedBank == 2)
        {
            if (catalystMirroringThisCase)
            {
                if (catalystB2MirroringConsecutiveCases < CATALYST_REQUIRED_CONSECUTIVE_CASES)
                {
                    catalystB2MirroringConsecutiveCases++;
                }
            }
            else
            {
                catalystB2MirroringConsecutiveCases = 0;
            }

            currentMirroringCount = catalystB2MirroringConsecutiveCases;
        }
        else
        {
            if (catalystMirroringThisCase)
            {
                if (catalystB1MirroringConsecutiveCases < CATALYST_REQUIRED_CONSECUTIVE_CASES)
                {
                    catalystB1MirroringConsecutiveCases++;
                }
            }
            else
            {
                catalystB1MirroringConsecutiveCases = 0;
            }

            currentMirroringCount = catalystB1MirroringConsecutiveCases;
        }


        bool catalystPersistentMirroring = currentMirroringCount >= CATALYST_REQUIRED_CONSECUTIVE_CASES;

        catalystResults.mirroringThisCase = catalystMirroringThisCase;

        catalystResults.consecutiveMirroringCases = currentMirroringCount;

        catalystResults.persistentMirroring = catalystPersistentMirroring;


        // Strong mirroring rule
        bool catalystStrongMirroringThisCase = catalystFeatures.maximumLaggedCorrelationValid && catalystFeatures.maximumLaggedCorrelation >= CATALYST_STRONG_CORRELATION_THRESHOLD;


        uint8_t currentStrongMirroringCount = 0;


        if (catalystFeatures.selectedBank == 2)
        {
            if (catalystStrongMirroringThisCase)
            {
                if (catalystB2StrongMirroringConsecutiveCases < CATALYST_STRONG_REQUIRED_CONSECUTIVE_CASES)
                {
                    catalystB2StrongMirroringConsecutiveCases++;
                }
            }
            else
            {
                catalystB2StrongMirroringConsecutiveCases = 0;
            }

            currentStrongMirroringCount = catalystB2StrongMirroringConsecutiveCases;
        }
        else
        {
            if (catalystStrongMirroringThisCase)
            {
                if (catalystB1StrongMirroringConsecutiveCases < CATALYST_STRONG_REQUIRED_CONSECUTIVE_CASES)
                {
                    catalystB1StrongMirroringConsecutiveCases++;
                }
            }
            else
            {
                catalystB1StrongMirroringConsecutiveCases = 0;
            }

            currentStrongMirroringCount = catalystB1StrongMirroringConsecutiveCases;
        }


        bool catalystStrongPersistentMirroring = currentStrongMirroringCount >= CATALYST_STRONG_REQUIRED_CONSECUTIVE_CASES;


        catalystResults.strongMirroringThisCase = catalystStrongMirroringThisCase;

        catalystResults.consecutiveStrongMirroringCases = currentStrongMirroringCount;

        catalystResults.strongPersistentMirroring = catalystStrongPersistentMirroring;

        // Diagnostic evidence requires persistent oxygen-sensor
        // behavior rather than a single-case correlation
        bool catalystDiagnosticEvidence = catalystPersistentMirroring || catalystStrongPersistentMirroring;


        // Customer-facing catalyst alerts require agreement
        // between the Isolation Forest and diagnostic logic
        bool catalystHybridAlert = catalystIfAlert && catalystDiagnosticEvidence;

        catalystResults.hybridAlert = catalystHybridAlert;


        bool catalystKeepEvidence = catalystHybridAlert;


        // Inspect Soon also requires both evidence sources.
        // Strong persistent mirroring can provide the diagnostic
        // severity without requiring the higher IF score.
        bool catalystInspectEvidence = catalystIfAlert && ((catalystScore >= CATALYST_INSPECT_SCORE_THRESHOLD && catalystPersistentMirroring) || catalystStrongPersistentMirroring);

        if (catalystIfAlert && catalystDiagnosticEvidence)
        {
            catalystResults.evidenceSource = "Both";
        }
        else if (catalystIfAlert)
        {
            catalystResults.evidenceSource = "Isolation Forest";
        }
        else if (catalystDiagnosticEvidence)
        {
            catalystResults.evidenceSource = "Diagnostic Logic";
        }
        else
        {
            catalystResults.evidenceSource = "None";
        }

        Serial.println();
        Serial.println("--- CATALYST MODEL RESULT ---");

        Serial.print("Anomaly score: ");
        Serial.println(catalystScore, 6);

        Serial.print("Threshold: ");
        Serial.println(CATALYST_ANOMALY_THRESHOLD, 6);

        Serial.print("IF Alert: ");

        if (catalystIfAlert)
        {
            Serial.println("YES");
        }
        else
        {
            Serial.println("NO");
        }


        Serial.print("Maximum lagged correlation: ");

        if (catalystFeatures.maximumLaggedCorrelationValid)
        {
            Serial.println(catalystFeatures.maximumLaggedCorrelation, 6);
        }
        else
        {
            Serial.println("N/A");
        }


        Serial.print("Mirroring this case: ");

        if (catalystMirroringThisCase)
        {
            Serial.println("YES");
        }
        else
        {
            Serial.println("NO");
        }


        Serial.print("Consecutive mirroring cases: ");
        Serial.println(currentMirroringCount);


        Serial.print("Persistent mirroring: ");

        if (catalystPersistentMirroring)
        {
            Serial.println("YES");
        }
        else
        {
            Serial.println("NO");
        }


        Serial.print("Strong mirroring this case: ");

        if (catalystStrongMirroringThisCase)
        {
            Serial.println("YES");
        }
        else
        {
            Serial.println("NO");
        }


        Serial.print("Consecutive strong mirroring cases: ");
        Serial.println(currentStrongMirroringCount);


        Serial.print("Hybrid catalyst alert: ");

        if (catalystHybridAlert)
        {
            Serial.println("YES");
        }
        else
        {
            Serial.println("NO");
        }


        Serial.print("Condition: ");

        if (catalystKeepEvidence)
        {
            catalystResults.condition = "Reduced-efficiency catalyst behavior";
        }
        else
        {
            catalystResults.condition ="Normal";
        }
        

        Serial.println(catalystResults.condition);


        Serial.print("Action Level: ");

        if (catalystInspectEvidence)
        {
            catalystResults.actionLevel = "Inspect Soon";
        }
        else if (catalystKeepEvidence)
        {
            catalystResults.actionLevel = "Keep an Eye Out";
        }
        else
        {
            catalystResults.actionLevel = "Normal Operation";
        }

        Serial.println(catalystResults.actionLevel);

        // 00 - Neither method detected abnormal behavior
        if (!catalystIfAlert && !catalystDiagnosticEvidence)
        {
            snprintf(
                catalystResults.description,
                sizeof(catalystResults.description),
                "Bank %u catalyst behavior remained consistent with normal operation. The oxygen sensor after the catalytic converter stayed sufficiently different from the sensor before it, which is expected when the catalyst is working normally.",
                catalystFeatures.selectedBank);
        }

        // 10 - Isolation Forest only
        else if (catalystIfAlert && !catalystDiagnosticEvidence)
        {
            snprintf(
                catalystResults.description,
                sizeof(catalystResults.description),
                "The predictive model noticed an exhaust-sensor pattern that differed somewhat from learned normal behavior. However, the sensor after the catalyst did not repeatedly follow the sensor before it, so the result remains Normal Operation.");
        }

        // 01 - Diagnostic logic only
        else if (!catalystIfAlert && catalystDiagnosticEvidence)
        {
            if (catalystStrongPersistentMirroring)
            {
                snprintf(
                    catalystResults.description,
                    sizeof(catalystResults.description),
                    "The oxygen sensor after the Bank %u catalyst closely followed the sensor before it for %u consecutive cases. This can be associated with reduced catalyst efficiency, but the predictive model did not detect an abnormal overall pattern, so the result remains Normal Operation.",
                    catalystFeatures.selectedBank,
                    currentStrongMirroringCount);
            }
            else
            {
                snprintf(
                    catalystResults.description,
                    sizeof(catalystResults.description),
                    "The oxygen sensor after the Bank %u catalyst followed the sensor before it more closely than expected for %u consecutive cases. The predictive model did not detect an abnormal overall pattern, so the result remains Normal Operation.",
                    catalystFeatures.selectedBank,
                    currentMirroringCount);
            }
        }

        // 11 - Both methods agree
        else
        {
            if (catalystStrongPersistentMirroring)
            {
                snprintf(
                    catalystResults.description,
                    sizeof(catalystResults.description),
                    "Both the predictive model and repeated oxygen-sensor behavior indicate that the sensor after the Bank %u catalyst is closely following the sensor before it. This continued for %u consecutive cases and provides stronger evidence of reduced catalyst efficiency.",
                    catalystFeatures.selectedBank,
                    currentStrongMirroringCount);
            }
            else
            {
                snprintf(
                    catalystResults.description,
                    sizeof(catalystResults.description),
                    "Both the predictive model and repeated oxygen-sensor behavior indicate that the sensor after the Bank %u catalyst is following the sensor before it more closely than expected. This continued for %u consecutive cases and can indicate reduced catalyst efficiency.",
                    catalystFeatures.selectedBank,
                    currentMirroringCount);
            }
        }

    }





    // Charging model features
    // --------------------------------------------------------

    Serial.println();
    Serial.println("--- CHARGING FEATURES ---");

    if (!chargingFeatures.ready)
    {
        Serial.println("Charging feature data is incomplete.");
    }
    else
    {
        chargingResults.available = true;

        for (size_t i = 0; i < 7; i++)
        {
            chargingResults.features[i] = chargingFeatures.values[i];
        }

        const char* chargingFeatureNames[7] =
        {
            "CONTROL_MODULE_VOLTAGE_V_MEAN",
            "CONTROL_MODULE_VOLTAGE_V_MIN",
            "CONTROL_MODULE_VOLTAGE_V_MAX",
            "BELOW_CHARGING_COUNT_LT_13_0",
            "ABOVE_CHARGING_COUNT_GT_14_8",
            "VOLTAGE_MAX_ABS_STEP",
            "VOLTAGE_STEP_STD"
        };


        for (size_t i = 0; i < 7; i++)
        {
            printFeature(chargingFeatureNames[i], chargingFeatures.values[i]);
        }

    }




    // Charging model result
    // --------------------------------------------------------

    if (chargingFeatures.ready)
    {
        float chargingScore = pistonIsolationForestScore(
            CHARGING_NODES,
            CHARGING_TREE_OFFSETS,
            CHARGING_TREE_COUNT,
            CHARGING_NORMALIZATION,
            CHARGING_MODEL_OFFSET,
            chargingFeatures.values,
            CHARGING_IMPUTER_MEDIANS);


        bool chargingAlert = pistonIsolationForestAlert(chargingScore, CHARGING_ANOMALY_THRESHOLD);

        chargingResults.anomalyScore = chargingScore;
        chargingResults.anomalyAlert = chargingAlert;

        // Build a second set of charging measurements for the
        // rule-based logic. These exclude engine-stop and restart
        // transients while leaving the Isolation Forest untouched.
        ChargingDiagnosticContext chargingContext = calculateChargingDiagnosticContext(samples);

        float meanVoltage = chargingContext.meanVoltage;

        float belowChargingCount = chargingContext.belowChargingCount;

        float aboveChargingCount = chargingContext.aboveChargingCount;

        float maximumVoltageStep = chargingContext.maximumAbsoluteStep;

        float voltageStepStandardDeviation = chargingContext.stepStandardDeviation;

        bool chargingLowMild = chargingContext.ready && belowChargingCount >= CHARGING_KEEP_LOW_COUNT;

        bool chargingHighMild = chargingContext.ready && aboveChargingCount >= CHARGING_KEEP_HIGH_COUNT;

        bool chargingStepMild = chargingContext.ready && maximumVoltageStep >= CHARGING_KEEP_MAX_STEP;

        bool chargingStepStdMild = chargingContext.ready && voltageStepStandardDeviation >= CHARGING_KEEP_STEP_STD;

        uint8_t chargingMildEvidenceCount = 0;

        if (chargingLowMild)
        {
            chargingMildEvidenceCount++;
        }

        if (chargingHighMild)
        {
            chargingMildEvidenceCount++;
        }

        if (chargingStepMild)
        {
            chargingMildEvidenceCount++;
        }

        if (chargingStepStdMild)
        {
            chargingMildEvidenceCount++;
        }


        bool chargingPhysicalMild = chargingMildEvidenceCount > 0;


        bool chargingKeepEvidence = chargingAlert && chargingPhysicalMild;


        // Strong physical evidence
        bool chargingStrongPhysical = chargingContext.ready &&
        (
            meanVoltage < CHARGING_INSPECT_LOW_MEAN ||
            meanVoltage > CHARGING_INSPECT_HIGH_MEAN ||
            belowChargingCount >= CHARGING_INSPECT_LOW_COUNT ||
            aboveChargingCount >= CHARGING_INSPECT_HIGH_COUNT ||
            maximumVoltageStep >= CHARGING_INSPECT_MAX_STEP ||
            voltageStepStandardDeviation >= CHARGING_INSPECT_STEP_STD
        );

        bool chargingMeanLowStrong = chargingContext.ready && meanVoltage < CHARGING_INSPECT_LOW_MEAN;

        bool chargingMeanHighStrong = chargingContext.ready && meanVoltage > CHARGING_INSPECT_HIGH_MEAN;

        bool chargingLowCountStrong = chargingContext.ready && belowChargingCount >= CHARGING_INSPECT_LOW_COUNT;

        bool chargingHighCountStrong = chargingContext.ready && aboveChargingCount >= CHARGING_INSPECT_HIGH_COUNT;

        bool chargingStepStrong = chargingContext.ready && maximumVoltageStep >= CHARGING_INSPECT_MAX_STEP;

        bool chargingStepStdStrong = chargingContext.ready && voltageStepStandardDeviation >= CHARGING_INSPECT_STEP_STD;
        
        
        // A strong charging case requires agreement between
        // the Isolation Forest and strong measured voltage behavior
        bool chargingStrongThisCase = chargingAlert && chargingStrongPhysical;

        chargingResults.strongChargingCase = chargingStrongThisCase;


        // Modern charging systems can intentionally change voltage
        // with battery state and electrical load, so Inspect Soon
        // requires strong agreement in consecutive cases
        if (chargingStrongThisCase)
        {
            if (chargingStrongConsecutiveCases < CHARGING_INSPECT_REQUIRED_CONSECUTIVE_CASES)
            {
                chargingStrongConsecutiveCases++;
            }
        }
        else chargingStrongConsecutiveCases = 0;


        bool chargingInspectEvidence = chargingStrongConsecutiveCases >= CHARGING_INSPECT_REQUIRED_CONSECUTIVE_CASES;

        chargingResults.consecutiveStrongCases = chargingStrongConsecutiveCases;


        // Evidence reporting should still record diagnostic evidence
        // even when it is not enough to raise the action level
        bool chargingDiagnosticEvidence = chargingPhysicalMild || chargingStrongPhysical;

        if (chargingAlert && chargingDiagnosticEvidence)
        {
            chargingResults.evidenceSource = "Both";
        }
        else if (chargingAlert)
        {
            chargingResults.evidenceSource = "Isolation Forest";
        }
        else if (chargingDiagnosticEvidence)
        {
            chargingResults.evidenceSource = "Diagnostic Logic";
        }
        else
        {
            chargingResults.evidenceSource = "None";
        }

        Serial.println();
        Serial.println("--- CHARGING MODEL RESULT ---");

        Serial.print("Anomaly score: ");
        Serial.println(chargingScore, 6);

        Serial.print("Threshold: ");
        Serial.println(CHARGING_ANOMALY_THRESHOLD, 6);

        Serial.print("Alert: ");

        if (chargingAlert)
        {
            Serial.println("YES");
        }
        else
        {
            Serial.println("NO");
        }


        Serial.print("Context-ready charging data: ");

if (chargingContext.ready) Serial.println("YES");
else Serial.println("NO");

Serial.print("Evaluated running samples: ");
Serial.println(chargingContext.evaluatedSampleCount);

Serial.print("Engine stop detected: ");

if (chargingContext.engineStopDetected) Serial.println("YES");
else Serial.println("NO");

Serial.print("Engine restart detected: ");

if (chargingContext.engineRestartDetected) Serial.println("YES");
else Serial.println("NO");

        if (chargingContext.ready)
        {
            Serial.print("Context mean voltage: ");
            Serial.println(meanVoltage, 6);

            Serial.print("Context below-13.0 count: ");
            Serial.println(belowChargingCount, 0);

            Serial.print("Context above-14.8 count: ");
            Serial.println(aboveChargingCount, 0);

            Serial.print("Context maximum voltage step: ");
            Serial.println(maximumVoltageStep, 6);

            Serial.print("Context voltage-step STD: ");
            Serial.println(voltageStepStandardDeviation, 6);
        }

        Serial.print("Strong charging case: ");

        if (chargingStrongThisCase)
        {
            Serial.println("YES");
        }
        else
        {
            Serial.println("NO");
        }


        Serial.print("Consecutive strong charging cases: ");
        Serial.println(chargingStrongConsecutiveCases);


        Serial.print("Condition: ");

        if (!chargingKeepEvidence)
        {
            chargingResults.condition = "Normal";
        }
        else if ( belowChargingCount > aboveChargingCount || meanVoltage < CHARGING_INSPECT_LOW_MEAN)
        {
            chargingResults.condition = "Low voltage behavior";
        }

        else if (aboveChargingCount > belowChargingCount || meanVoltage > CHARGING_INSPECT_HIGH_MEAN)
        {
            chargingResults.condition = "High voltage behavior";
        }

        else
        {
            chargingResults.condition = "Unstable voltage behavior";
        }

        Serial.println(chargingResults.condition);

        Serial.print("Action Level: ");

       if (chargingInspectEvidence)
        {
            chargingResults.actionLevel = "Inspect Soon";
        }
        else if (chargingKeepEvidence)
        {
            chargingResults.actionLevel = "Keep an Eye Out";
        }
        else
        {
            chargingResults.actionLevel = "Normal Operation";
        }

        Serial.println(chargingResults.actionLevel);

        chargingResults.description[0] = '\0';

        // 00 - Neither method detected abnormal behavior
        if (!chargingAlert && !chargingDiagnosticEvidence)
        {
            snprintf(
                chargingResults.description,
                sizeof(chargingResults.description),
                "Charging-system voltage remained stable and within the expected operating limits during this case.");
        }

        // 10 - Isolation Forest only
        else if (chargingAlert && !chargingDiagnosticEvidence)
        {
            if (!chargingContext.ready && (chargingContext.engineStopDetected || chargingContext.engineRestartDetected))
            {
                snprintf(
                    chargingResults.description,
                    sizeof(chargingResults.description),
                    "The predictive model noticed a charging-voltage pattern that differed from learned normal behavior. Engine stop or restart activity was also detected, so expected voltage changes around that event were excluded from the charging fault decision. There was not enough steady engine-running evidence to confirm a charging problem, so the result remains Normal Operation.");
            }
            else if (!chargingContext.ready)
            {
                snprintf(
                    chargingResults.description,
                    sizeof(chargingResults.description),
                    "The predictive model noticed a charging-voltage pattern that differed from learned normal behavior. There was not enough valid steady engine-running data to confirm the pattern with the charging diagnostic logic, so the result remains Normal Operation.");
            }
            else
            {
                snprintf(
                    chargingResults.description,
                    sizeof(chargingResults.description),
                    "The predictive model noticed a charging-voltage pattern that differed somewhat from learned normal behavior. Measured steady engine-running voltage did not cross the configured monitoring limits, so the result remains Normal Operation.");
            }
        }

        // Diagnostic evidence exists: 01 or 11
        else
        {
            if (!chargingAlert)
            {
                snprintf(
                    chargingResults.description,
                    sizeof(chargingResults.description),
                    "One or more charging measurements crossed a monitoring threshold during this case. The predictive model did not detect an abnormal overall charging pattern, so the result remains Normal Operation.");
            }
            else
            {
                snprintf(
                    chargingResults.description,
                    sizeof(chargingResults.description),
                    "Both the predictive model and measured charging behavior detected an unusual voltage pattern.");
            }

            uint8_t evidenceItemsAdded = 0;

            // Strong evidence is added first
            // --------------------------------------------------------

            if (chargingMeanLowStrong && evidenceItemsAdded < 3)
            {
                size_t used = strlen(chargingResults.description);

                const char* separator = "";

                if (used > 0) separator = " ";

                snprintf(
                    chargingResults.description + used,
                    sizeof(chargingResults.description) - used,
                    "%sAverage control-module voltage was %.2f V, below the %.2f V inspection threshold.",
                    separator,
                    meanVoltage,
                    CHARGING_INSPECT_LOW_MEAN);

                evidenceItemsAdded++;
            }


            if (chargingMeanHighStrong && evidenceItemsAdded < 3)
            {
                size_t used = strlen(chargingResults.description);

                const char* separator = "";

                if (used > 0) separator = " ";

                snprintf(
                    chargingResults.description + used,
                    sizeof(chargingResults.description) - used,
                    "%sAverage control-module voltage was %.2f V, above the %.2f V inspection threshold.",
                    separator,
                    meanVoltage,
                    CHARGING_INSPECT_HIGH_MEAN);

                evidenceItemsAdded++;
            }


            if (chargingLowCountStrong && evidenceItemsAdded < 3)
            {
                size_t used = strlen(chargingResults.description);

                const char* separator = "";

                if (used > 0) separator = " ";

                snprintf(
                    chargingResults.description + used,
                    sizeof(chargingResults.description) - used,
                    "%s%.0f of %u voltage readings were below %.1f V, meeting the %u-reading inspection threshold.",
                    separator,
                    belowChargingCount,
                    static_cast<unsigned int>(chargingContext.evaluatedSampleCount),
                    CHARGING_INSPECT_LOW_MEAN,
                    static_cast<unsigned int>(CHARGING_INSPECT_LOW_COUNT));

                evidenceItemsAdded++;
            }


            if (chargingHighCountStrong && evidenceItemsAdded < 3)
            {
                size_t used = strlen(chargingResults.description);

                const char* separator = "";

                if (used > 0) separator = " ";

                snprintf(
                    chargingResults.description + used,
                    sizeof(chargingResults.description) - used,
                    "%s%.0f of %u voltage readings were above %.1f V, meeting the %u-reading inspection threshold.",
                    separator,
                    aboveChargingCount,
                    static_cast<unsigned int>(chargingContext.evaluatedSampleCount),
                    CHARGING_INSPECT_HIGH_MEAN,
                    static_cast<unsigned int>(CHARGING_INSPECT_HIGH_COUNT));

                evidenceItemsAdded++;
            }


            if (chargingStepStrong && evidenceItemsAdded < 3)
            {
                size_t used = strlen(chargingResults.description);

                const char* separator = "";

                if (used > 0) separator = " ";

                snprintf(
                    chargingResults.description + used,
                    sizeof(chargingResults.description) - used,
                    "%sVoltage changed by as much as %.2f V between consecutive readings, above the %.2f V inspection threshold.",
                    separator,
                    maximumVoltageStep,
                    CHARGING_INSPECT_MAX_STEP);

                evidenceItemsAdded++;
            }


            if (chargingStepStdStrong && evidenceItemsAdded < 3)
            {
                size_t used = strlen(chargingResults.description);

                const char* separator = "";

                if (used > 0) separator = " ";

                snprintf(
                    chargingResults.description + used,
                    sizeof(chargingResults.description) - used,
                    "%sVariation between consecutive voltage changes was %.2f V, above the %.2f V inspection threshold.",
                    separator,
                    voltageStepStandardDeviation,
                    CHARGING_INSPECT_STEP_STD);

                evidenceItemsAdded++;
            }


            // Mild evidence is added only if there is room
            // --------------------------------------------------------

            if (chargingLowMild && !chargingLowCountStrong && evidenceItemsAdded < 3)
            {
                size_t used = strlen(chargingResults.description);

                const char* separator = "";

                if (used > 0) separator = " ";

                snprintf(
                    chargingResults.description + used,
                    sizeof(chargingResults.description) - used,
                    "%s%.0f of %u voltage readings were below %.1f V, meeting the %u-reading monitoring threshold.",
                    separator,
                    belowChargingCount,
                    static_cast<unsigned int>(chargingContext.evaluatedSampleCount),
                    CHARGING_INSPECT_LOW_MEAN,
                    static_cast<unsigned int>(CHARGING_KEEP_LOW_COUNT));

                evidenceItemsAdded++;
            }


            if (chargingHighMild && !chargingHighCountStrong && evidenceItemsAdded < 3)
            {
                size_t used = strlen(chargingResults.description);

                const char* separator = "";

                if (used > 0) separator = " ";

                snprintf(
                    chargingResults.description + used,
                    sizeof(chargingResults.description) - used,
                    "%s%.0f of %u voltage readings were above %.1f V, meeting the %u-reading monitoring threshold.",
                    separator,
                    aboveChargingCount,
                    static_cast<unsigned int>(chargingContext.evaluatedSampleCount),
                    CHARGING_INSPECT_HIGH_MEAN,
                    static_cast<unsigned int>(CHARGING_KEEP_HIGH_COUNT));

                evidenceItemsAdded++;
            }


            if (chargingStepMild && !chargingStepStrong && evidenceItemsAdded < 3)
            {
                size_t used = strlen(chargingResults.description);

                const char* separator = "";

                if (used > 0) separator = " ";

                snprintf(
                    chargingResults.description + used,
                    sizeof(chargingResults.description) - used,
                    "%sVoltage changed by as much as %.2f V between consecutive readings, above the %.2f V monitoring threshold.",
                    separator,
                    maximumVoltageStep,
                    CHARGING_KEEP_MAX_STEP);

                evidenceItemsAdded++;
            }


            if (chargingStepStdMild && !chargingStepStdStrong && evidenceItemsAdded < 3)
            {
                size_t used = strlen(chargingResults.description);

                const char* separator = "";

                if (used > 0) separator = " ";

                snprintf(
                    chargingResults.description + used,
                    sizeof(chargingResults.description) - used,
                    "%sVariation between consecutive voltage changes was %.2f V, above the %.2f V monitoring threshold.",
                    separator,
                    voltageStepStandardDeviation,
                    CHARGING_KEEP_STEP_STD);

                evidenceItemsAdded++;
            }
        }
    }

    // Build the complete case that will eventually be converted to JSON
    static CompletedCaseToTransmit completedCase;
    completedCase = {};

    currentCaseId++;
    completedCase.caseId = currentCaseId;


    // Copy all 20 raw vehicle samples into the completed case
    for (size_t i = 0; i < CASE_SIZE; i++)
    {
        completedCase.samples[i] = samples[i];
    }


    // Copy the final subsystem results
    completedCase.fuel = fuelResults;
    completedCase.catalyst = catalystResults;
    completedCase.charging = chargingResults;

    JsonDocument completedCaseJson;

    buildCompletedCaseJson(completedCase, completedCaseJson);

    size_t completedCaseJsonLength = measureJson(completedCaseJson);

    String completedCaseJsonPayload;

    completedCaseJsonPayload.reserve(completedCaseJsonLength + 1);

    serializeJson(completedCaseJson, completedCaseJsonPayload);

    Serial.println();
    Serial.println("--- COMPLETE CASE JSON ---");

    Serial.println(completedCaseJsonPayload);

    sendJsonPayloadOverBle(completedCaseJsonPayload);

    Serial.println();
    Serial.println("--- COMPLETED CASE STRUCT CHECK ---");

    Serial.print("Case ID: ");
    Serial.println(completedCase.caseId);

    Serial.print("First sample timestamp: ");
    Serial.println(completedCase.samples[0].timestampMs);

    Serial.print("Last sample timestamp: ");
    Serial.println(completedCase.samples[CASE_SIZE - 1].timestampMs);

    Serial.print("Fuel available: ");
    if (completedCase.fuel.available) Serial.println("YES");
    else Serial.println("NO");

    Serial.print("Fuel action level: ");
    Serial.println(completedCase.fuel.actionLevel);

    Serial.print("Catalyst available: ");
    if (completedCase.catalyst.available) Serial.println("YES");
    else Serial.println("NO");

    Serial.print("Catalyst action level: ");
    Serial.println(completedCase.catalyst.actionLevel);

    Serial.print("Charging available: ");
    if (completedCase.charging.available) Serial.println("YES");
    else Serial.println("NO");

    Serial.print("Charging action level: ");
    Serial.println(completedCase.charging.actionLevel);


    Serial.println();
    Serial.println("Starting a new non-overlapping case.");
    Serial.println("=================================");
    Serial.println();
}



// TWAI controller initialization
// ------------------------------------------------------------

// Starts TWAI at 500 kbit/s
bool initializeCan()
{
    twai_general_config_t generalConfig = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);

    // Number of outgoing CAN messages that can wait in the transmit queue
    generalConfig.tx_queue_len = 10;

    // Number of incoming CAN messages that can wait in the receive queue
    generalConfig.rx_queue_len = 40;

    // Configure the CAN bus for 500 kbit/s
    twai_timing_config_t timingConfig = TWAI_TIMING_CONFIG_500KBITS();

    // Initially accept every CAN message
    // The request function later checks the identifier and response contents
    twai_filter_config_t filterConfig = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // Install the TWAI driver
    if (twai_driver_install(&generalConfig, &timingConfig, &filterConfig) != ESP_OK)
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


// BLE initialization
// ------------------------------------------------------------

void initializeBle()
{
    BLEDevice::init(BLE_DEVICE_NAME);

    bleServer = BLEDevice::createServer();

    bleServer->setCallbacks(new PistonBleServerCallbacks());


    BLEService* pistonService = bleServer->createService(BLE_SERVICE_UUID);


    bleTxCharacteristic = pistonService->createCharacteristic(BLE_TX_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_NOTIFY);


    bleTxCharacteristic->addDescriptor(new BLE2902());


    bleRxCharacteristic = pistonService->createCharacteristic(BLE_RX_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_WRITE);


    bleRxCharacteristic->setCallbacks(new PistonBleCommandCallbacks());


    pistonService->start();


    BLEAdvertising* advertising = BLEDevice::getAdvertising();

    advertising->addServiceUUID(BLE_SERVICE_UUID);

    advertising->start();


    Serial.println("BLE initialized.");
    Serial.println("Advertising as P.I.S.T.O.N.");
}


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

    initializeBle(); 

    // CSV column titles for the serial monitor output
    Serial.println(
        "index,time_ms,engine_load,"
        "stft_b1,ltft_b1,stft_b2,ltft_b2,rpm,"
        "o2_b1s1_voltage,o2_b1s2_voltage,o2_b1s1_equiv,"
        "o2_b2s1_voltage,o2_b2s2_voltage,o2_b2s1_equiv,"
        "control_module_voltage");

    // Sampling starts paused.
    // Timing is reset when start_logging is received.
    previousSampleTime = millis();

    Serial.println("Logging state: PAUSED");
}



// main loop
// ------------------------------------------------------------

void loop()
{

    const int loggingCommand = pendingLoggingCommand.exchange(LOGGING_COMMAND_NONE);

    if (loggingCommand == LOGGING_COMMAND_START)
    {
        // Determine the vehicle's Bank 2 configuration before
        // beginning the first sample. If the check is inconclusive,
        // it will be attempted again the next time logging starts.
        if (!bank2AvailabilityKnown) detectBank2Availability();

        // A new logging run starts a new charging-context timeline.
        // Do not carry a possible engine-stop/restart state across
        // a pause or disconnected period.
        chargingPreviousEngineStopped = false;
        chargingRestartSuppressionSamplesRemaining = 0;

        loggingEnabled = true;

        // Always begin with a fresh case.
        sampleIndex = 0;

        previousSampleTime = millis() - SAMPLE_PERIOD_MS;

        Serial.println();
        Serial.println("P.I.S.T.O.N. logging STARTED.");

        sendJsonPayloadOverBle("{\"type\":\"logging_status\"," "\"enabled\":true}");
    }
    else if (loggingCommand == LOGGING_COMMAND_PAUSE)
    {
        loggingEnabled = false;

        // Discard any incomplete case.
        sampleIndex = 0;

        Serial.println();
        Serial.println("P.I.S.T.O.N. logging PAUSED.");

        if (bleDeviceConnected)
        {
            sendJsonPayloadOverBle("{\"type\":\"logging_status\"," "\"enabled\":false}");
        }

    }
    else if (loggingCommand == LOGGING_COMMAND_STATUS)
    {
        if (loggingEnabled)
        {
            sendJsonPayloadOverBle("{\"type\":\"logging_status\"," "\"enabled\":true}");
        }
        else
        {
            sendJsonPayloadOverBle("{\"type\":\"logging_status\"," "\"enabled\":false}");
        }

        Serial.print("Logging status sent: ");

        if (loggingEnabled) Serial.println("ENABLED");
        else Serial.println("PAUSED");
    }

    const bool dtcScanRequested = pendingDtcRequest.exchange(false);

    if (dtcScanRequested)
    {
        if (loggingEnabled)
        {
            Serial.println();
            Serial.println("DTC scan rejected: logging is active.");

            sendJsonPayloadOverBle("{\"type\":\"dtc_result\"," "\"success\":false," "\"error\":\"Pause logging before scanning DTCs\"}");
        }
        else
        {
            performDtcScanAndSend();
        }
    }

    const bool vinReadRequested = pendingVinRequest.exchange(false);

    if (vinReadRequested)
    {
        if (loggingEnabled)
        {
            Serial.println();
            Serial.println("VIN read rejected: logging is active.");

            sendJsonPayloadOverBle("{\"type\":\"vehicle_info\"," "\"success\":false," "\"vin\":null," "\"error\":\"Pause logging before reading vehicle information\"}");
        }
        else performVehicleInfoReadAndSend();
    }

    // P.I.S.T.O.N. powers on paused.
    if (!loggingEnabled)
    {
        delay(10);
        return;
    }


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
    VehicleSample& currentSample = caseSamples[sampleIndex];

    // Request all required PIDs and store one complete sample
    collectOneSample(currentSample);

    // A pause command may have arrived while this
    // sample was being collected.
    if (pendingLoggingCommand.load() == LOGGING_COMMAND_PAUSE)
    {
        sampleIndex = 0;
        return;
    }

    // Print the completed sample to the serial monitor
    printSample(currentSample, sampleIndex);

    // Move to the next position in the case array/
    sampleIndex++;

    // Process the case after all twenty samples have been collected
    if (sampleIndex >= CASE_SIZE)
    {
        processCompletedCase(caseSamples);

        // Reset to the beginning of the array for a new non-overlapping twenty-sample case
        sampleIndex = 0;
    }
}
