#include <Arduino.h>
#include <driver/twai.h>
#include <math.h>


 
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


 
// Sampling configuration
// ------------------------------------------------------------

// Twenty samples are collected for each case, matching the ML training data
constexpr size_t CASE_SIZE = 20;

// Attempt to start one complete sample collection every second
constexpr uint32_t SAMPLE_PERIOD_MS = 1000;

// Maximum time to wait for the vehicle to respond to one PID request
constexpr uint32_t PID_TIMEOUT_MS = 80;

// Parameter valid when at least 16 of 20 readings valid
constexpr size_t MIN_VALID_SAMPLES = 16;

// Check O2 correlation from lag -3 to +3
constexpr int MAX_O2_CORRELATION_LAG = 3;

// At least 8 valid O2 pairs needed for correlation
constexpr size_t MIN_O2_CORRELATION_PAIRS = 8;

// Prevents division by zero in catalyst calculations
constexpr float CATALYST_EPSILON = 0.000001f;

 
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

// Statistics calculated from one parameter
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


// Stores the 20 fuel model inputs
struct FuelCaseFeatures
{
    bool ready;

    // Zero means no valid bank was available
    // One means Bank 1 was selected
    // Two means Bank 2 was selected
    uint8_t selectedBank;

    float values[20];
};


// Stores the 5 catalyst model inputs and separate correlation value
struct CatalystCaseFeatures
{
    bool ready;
    float values[5];

    float maximumLaggedCorrelation;
    bool maximumLaggedCorrelationValid;
};


// Stores the 11 battery model inputs
struct BatteryCaseFeatures
{
    bool ready;
    float values[11];
};


// Stores catalyst calculations for one bank
struct CatalystBankFeatures
{
    bool available;

    float standardDeviationSimilarityGap;
    float rangeSimilarityGap;
    float downstreamRelativeStandardDeviation;
    float downstreamRelativeRange;
    float downstreamSwitchRange;
};

// Stores one non-overlapping 20-sample case
VehicleSample caseSamples[CASE_SIZE];

// Next position in the 20-sample array
size_t sampleIndex = 0;

// Time the last sample collection started
uint32_t previousSampleTime = 0;

// ECU response ID, starts at 0 until one responds
uint32_t selectedEcuResponseId = 0;


 
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
    // Four-cylinder engines normally do not have Bank 2, so this PID may be unsupported on many vehicles
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
    if (requestMode01Pid(0x18, data, 2))
    {
        sample.o2B2S1Voltage = decodeNarrowbandO2Voltage(data[0]);

        sample.o2B2S1VoltageValid = true;
    }


    // PID 19: O2 Sensor 6
    // In the normal two-bank layout this is Bank 2 Sensor 2, which is the downstream O2 sensor for Bank 2
    if (requestMode01Pid(0x19, data, 2))
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
    if (
        !sample.o2B1S2VoltageValid &&
        requestMode01Pid(0x25, data, 4))
    {
        sample.o2B1S2Voltage = decodeWidebandVoltage(data[2], data[3]);

        sample.o2B1S2VoltageValid = true;
    }


    // PID 28: Wideband O2 Sensor 5
    // Normally represents Bank 2 Sensor 1 in a two-bank layout
    if (requestMode01Pid(0x28, data, 4))
    {
        sample.o2B2S1EquivalenceRatio =
            decodeWidebandEquivalenceRatio(data[0], data[1]);

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
    if (
        !sample.o2B2S2VoltageValid &&
        requestMode01Pid(0x29, data, 4))
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
    if (valid) Serial.print(value, 2);
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

// Creates the 20 fuel features
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

    ValueStatistics totalTrimB1 = calculateStatistics(totalTrimB1Values,totalTrimB1Valid);

    ValueStatistics absoluteTotalTrimB1 = calculateStatistics(absoluteTotalTrimB1Values, absoluteTotalTrimB1Valid);


    ValueStatistics stftB2 = calculateStatistics(stftB2Values, stftB2Valid);

    ValueStatistics ltftB2 = calculateStatistics(ltftB2Values, ltftB2Valid);

    ValueStatistics totalTrimB2 = calculateStatistics(totalTrimB2Values, totalTrimB2Valid);

    ValueStatistics absoluteTotalTrimB2 = calculateStatistics(absoluteTotalTrimB2Values,absoluteTotalTrimB2Valid);


    bool bank1Reliable = absoluteTotalTrimB1.validCount >= MIN_VALID_SAMPLES;

    bool bank2Reliable = absoluteTotalTrimB2.validCount >= MIN_VALID_SAMPLES;


    // Bank 2 is selected only when it is available and its average absolute total trim is worse than Bank 1
    bool useBank2 = bank2Reliable && ( !bank1Reliable || absoluteTotalTrimB2.mean > absoluteTotalTrimB1.mean );


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


    // Fuel feature order used during training

    result.values[0] = rpm.mean;
    result.values[1] = rpm.standardDeviation;

    result.values[2] = load.mean;
    result.values[3] = load.standardDeviation;

    result.values[4] = selectedStft.mean;
    result.values[5] = selectedStft.standardDeviation;
    result.values[6] = selectedStft.minimum;
    result.values[7] = selectedStft.maximum;

    result.values[8] = selectedLtft.mean;
    result.values[9] = selectedLtft.standardDeviation;
    result.values[10] = selectedLtft.minimum;
    result.values[11] = selectedLtft.maximum;

    result.values[12] = selectedTotalTrim.mean;
    result.values[13] = selectedTotalTrim.standardDeviation;
    result.values[14] = selectedTotalTrim.minimum;
    result.values[15] = selectedTotalTrim.maximum;

    result.values[16] = selectedAbsoluteTotalTrim.mean;
    result.values[17] = selectedAbsoluteTotalTrim.standardDeviation;
    result.values[18] = selectedAbsoluteTotalTrim.minimum;
    result.values[19] = selectedAbsoluteTotalTrim.maximum;


    result.ready =
        rpm.validCount >= MIN_VALID_SAMPLES &&
        load.validCount >= MIN_VALID_SAMPLES &&
        selectedStft.validCount >= MIN_VALID_SAMPLES &&
        selectedLtft.validCount >= MIN_VALID_SAMPLES &&
        selectedTotalTrim.validCount >= MIN_VALID_SAMPLES;

    return result;
}


 
// Catalyst bank feature calculation
// ------------------------------------------------------------

// Calculates catalyst features for one bank
CatalystBankFeatures calculateCatalystBankFeatures(
    const ValueStatistics& upstreamVoltage,
    const ValueStatistics& upstreamEquivalenceRatio,
    const ValueStatistics& downstreamVoltage)
{
    CatalystBankFeatures result = {};

    result.available = false;

    bool voltageReliable = upstreamVoltage.validCount >= MIN_VALID_SAMPLES;

    bool equivalenceRatioReliable = upstreamEquivalenceRatio.validCount >= MIN_VALID_SAMPLES;

    bool downstreamReliable = downstreamVoltage.validCount >= MIN_VALID_SAMPLES;


    // Upstream voltage is preferred
    // Equivalence ratio is used when upstream voltage is unavailable
    if ((!voltageReliable && !equivalenceRatioReliable) || !downstreamReliable) return result;

    ValueStatistics upstream;

    if (voltageReliable)
    {
        upstream = upstreamVoltage;
    }
    else
    {
        upstream = upstreamEquivalenceRatio;
    }


    float upstreamRelativeStandardDeviation = upstream.standardDeviation / (fabs(upstream.mean) + CATALYST_EPSILON);

    float upstreamRelativeRange = upstream.range / (fabs(upstream.mean) + CATALYST_EPSILON);

    float downstreamRelativeStandardDeviation = downstreamVoltage.standardDeviation / (fabs(downstreamVoltage.mean) + CATALYST_EPSILON);

    float downstreamRelativeRange = downstreamVoltage.range / (fabs(downstreamVoltage.mean) + CATALYST_EPSILON);


    result.standardDeviationSimilarityGap = fabs(log((downstreamRelativeStandardDeviation + CATALYST_EPSILON) / (upstreamRelativeStandardDeviation + CATALYST_EPSILON)));

    result.rangeSimilarityGap = fabs(log((downstreamRelativeRange + CATALYST_EPSILON) / (upstreamRelativeRange + CATALYST_EPSILON)));

    result.downstreamRelativeStandardDeviation = downstreamRelativeStandardDeviation;

    result.downstreamRelativeRange = downstreamRelativeRange;

    // Downstream switch range = max - min
    result.downstreamSwitchRange = downstreamVoltage.range;

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


// Checks lag -3 through +3 and keeps the highest correlation
bool calculateMaximumLaggedCorrelation(const float upstreamValues[CASE_SIZE], const bool upstreamValid[CASE_SIZE], const float downstreamValues[CASE_SIZE], const bool downstreamValid[CASE_SIZE], float& maximumCorrelationOutput)
{
    bool correlationFound = false;
    float maximumCorrelation = -1.0f;

    for (int lag = -MAX_O2_CORRELATION_LAG; lag <= MAX_O2_CORRELATION_LAG; lag++)
    {
        float currentCorrelation = NAN;

        if (!calculateCorrelationAtLag(upstreamValues, upstreamValid, downstreamValues, downstreamValid, lag, currentCorrelation))
        {
            continue;
        }

        if (!correlationFound || currentCorrelation > maximumCorrelation)
        {
            maximumCorrelation = currentCorrelation;
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

// Creates the 5 catalyst model features and O2 correlation value
CatalystCaseFeatures calculateCatalystFeatures(const VehicleSample samples[CASE_SIZE])
{
    CatalystCaseFeatures result = {};
    result.ready = false;
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


    CatalystBankFeatures bank1 = calculateCatalystBankFeatures(upstreamB1Voltage, upstreamB1Equivalence, downstreamB1Voltage);

    CatalystBankFeatures bank2 = calculateCatalystBankFeatures(upstreamB2Voltage, upstreamB2Equivalence, downstreamB2Voltage);


    if (!bank1.available && !bank2.available)
    {
        return result;
    }


    if (bank1.available && bank2.available)
    {
        result.values[0] = min(bank1.standardDeviationSimilarityGap, bank2.standardDeviationSimilarityGap);

        result.values[1] = min(bank1.rangeSimilarityGap, bank2.rangeSimilarityGap);

        result.values[2] = max(bank1.downstreamRelativeStandardDeviation, bank2.downstreamRelativeStandardDeviation);

        result.values[3] = max(bank1.downstreamRelativeRange, bank2.downstreamRelativeRange);

        result.values[4] = max(bank1.downstreamSwitchRange, bank2.downstreamSwitchRange);
    }
    else
    {
        CatalystBankFeatures selectedBank;

        if (bank1.available)
        {
            selectedBank = bank1;
        }
        else
        {
            selectedBank = bank2;
        }

        result.values[0] = selectedBank.standardDeviationSimilarityGap;

        result.values[1] = selectedBank.rangeSimilarityGap;

        result.values[2] = selectedBank.downstreamRelativeStandardDeviation;

        result.values[3] = selectedBank.downstreamRelativeRange;

        result.values[4] = selectedBank.downstreamSwitchRange;
    }


    // Calculate B1 correlation using voltage first, then equivalence ratio
    float bank1Correlation = NAN;
    bool bank1CorrelationValid = false;

    if (upstreamB1Voltage.validCount >= MIN_O2_CORRELATION_PAIRS)
    {
        bank1CorrelationValid = calculateMaximumLaggedCorrelation(upstreamB1VoltageValues, upstreamB1VoltageValid, downstreamB1VoltageValues, downstreamB1VoltageValid, bank1Correlation);
    }
    else if (upstreamB1Equivalence.validCount >= MIN_O2_CORRELATION_PAIRS)
    {
        bank1CorrelationValid = calculateMaximumLaggedCorrelation(upstreamB1EquivalenceValues, upstreamB1EquivalenceValid, downstreamB1VoltageValues, downstreamB1VoltageValid, bank1Correlation);
    }


    // Do the same for B2 if the vehicle has it
    float bank2Correlation = NAN;
    bool bank2CorrelationValid = false;

    if (upstreamB2Voltage.validCount >= MIN_O2_CORRELATION_PAIRS)
    {
        bank2CorrelationValid = calculateMaximumLaggedCorrelation(upstreamB2VoltageValues, upstreamB2VoltageValid, downstreamB2VoltageValues, downstreamB2VoltageValid, bank2Correlation);
    }
    else if (upstreamB2Equivalence.validCount >= MIN_O2_CORRELATION_PAIRS)
    {
        bank2CorrelationValid = calculateMaximumLaggedCorrelation(upstreamB2EquivalenceValues, upstreamB2EquivalenceValid, downstreamB2VoltageValues, downstreamB2VoltageValid, bank2Correlation);
    }


    // Keep whichever bank had the higher valid correlation
    if (bank1CorrelationValid && bank2CorrelationValid)
    {
        result.maximumLaggedCorrelation = max(bank1Correlation, bank2Correlation);

        result.maximumLaggedCorrelationValid = true;
    }
    else if (bank1CorrelationValid)
    {
        result.maximumLaggedCorrelation = bank1Correlation;

        result.maximumLaggedCorrelationValid = true;
    }
    else if (bank2CorrelationValid)
    {
        result.maximumLaggedCorrelation = bank2Correlation;

        result.maximumLaggedCorrelationValid = true;
    }


    result.ready = true;

    return result;
}



// Battery feature calculation
// ------------------------------------------------------------

// Creates the 11 battery features
BatteryCaseFeatures calculateBatteryFeatures(const VehicleSample samples[CASE_SIZE])
{
    BatteryCaseFeatures result = {};
    result.ready = false;

    float voltageValues[CASE_SIZE];
    bool voltageValid[CASE_SIZE];

    int lowVoltageCount = 0;
    int highVoltageCount = 0;
    int belowChargingCount = 0;
    int aboveChargingCount = 0;


    for (size_t i = 0; i < CASE_SIZE; i++)
    {
        voltageValues[i] = samples[i].controlModuleVoltage;

        voltageValid[i] = samples[i].controlModuleVoltageValid;

        if (!voltageValid[i])
        {
            continue;
        }

        float voltage = samples[i].controlModuleVoltage;

        if (voltage < 12.8f)
        {
            lowVoltageCount++;
        }

        if (voltage > 15.0f)
        {
            highVoltageCount++;
        }

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


    // Battery feature order used during training

    result.values[0] = voltage.mean;
    result.values[1] = voltage.standardDeviation;
    result.values[2] = voltage.minimum;
    result.values[3] = voltage.maximum;

    result.values[4] = voltage.range;

    // VOLTAGE_RANGE and CONTROL_MODULE_VOLTAGE_V_RANGE are duplicate columns in the training feature table
    result.values[5] = voltage.range;

    result.values[6] = fabs(voltage.mean - 14.0f);

    result.values[7] = static_cast<float>(lowVoltageCount);

    result.values[8] = static_cast<float>(highVoltageCount);

    result.values[9] = static_cast<float>(belowChargingCount);

    result.values[10] = static_cast<float>(aboveChargingCount);

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



// Completed case processing
// ------------------------------------------------------------

// Calculates and prints all features after 20 samples
void processCompletedCase(const VehicleSample samples[CASE_SIZE])
{
    Serial.println();
    Serial.println("=== 20-SAMPLE CASE COMPLETE ===");


    FuelCaseFeatures fuelFeatures = calculateFuelFeatures(samples);

    CatalystCaseFeatures catalystFeatures = calculateCatalystFeatures(samples);

    BatteryCaseFeatures batteryFeatures = calculateBatteryFeatures(samples);


    // --------------------------------------------------------
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
        Serial.print("Selected fuel bank: B");
        Serial.println(fuelFeatures.selectedBank);

        const char* fuelFeatureNames[20] =
        {
            "ENGINE_RPM_MEAN",
            "ENGINE_RPM_STD",
            "ENGINE_LOAD_PCT_MEAN",
            "ENGINE_LOAD_PCT_STD",

            "SELECTED_BANK_STFT_PCT_MEAN",
            "SELECTED_BANK_STFT_PCT_STD",
            "SELECTED_BANK_STFT_PCT_MIN",
            "SELECTED_BANK_STFT_PCT_MAX",

            "SELECTED_BANK_LTFT_PCT_MEAN",
            "SELECTED_BANK_LTFT_PCT_STD",
            "SELECTED_BANK_LTFT_PCT_MIN",
            "SELECTED_BANK_LTFT_PCT_MAX",

            "SELECTED_BANK_TOTAL_TRIM_PCT_MEAN",
            "SELECTED_BANK_TOTAL_TRIM_PCT_STD",
            "SELECTED_BANK_TOTAL_TRIM_PCT_MIN",
            "SELECTED_BANK_TOTAL_TRIM_PCT_MAX",

            "SELECTED_BANK_ABS_TOTAL_TRIM_PCT_MEAN",
            "SELECTED_BANK_ABS_TOTAL_TRIM_PCT_STD",
            "SELECTED_BANK_ABS_TOTAL_TRIM_PCT_MIN",
            "SELECTED_BANK_ABS_TOTAL_TRIM_PCT_MAX"
        };

        for (size_t i = 0; i < 20; i++)
        {
            printFeature(fuelFeatureNames[i], fuelFeatures.values[i]);
        }
    }


    // --------------------------------------------------------
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
        const char* catalystFeatureNames[5] =
        {
            "SMALLEST_O2_STD_SIMILARITY_GAP",
            "SMALLEST_O2_RANGE_SIMILARITY_GAP",
            "LARGEST_DOWNSTREAM_O2_RELATIVE_STD",
            "LARGEST_DOWNSTREAM_O2_RELATIVE_RANGE",
            "LARGEST_DOWNSTREAM_O2_SWITCH_RANGE"
        };

        for (size_t i = 0; i < 5; i++)
        {
            printFeature(catalystFeatureNames[i], catalystFeatures.values[i]);
        }

        if (catalystFeatures.maximumLaggedCorrelationValid)
        {
            printFeature("MAX_UPSTREAM_DOWNSTREAM_O2_CORRELATION", catalystFeatures.maximumLaggedCorrelation);
        }
        else
        {
            Serial.println("MAX_UPSTREAM_DOWNSTREAM_O2_CORRELATION: NA");
        }
    }


    // --------------------------------------------------------
    // Battery model features
    // --------------------------------------------------------

    Serial.println();
    Serial.println("--- BATTERY FEATURES ---");

    if (!batteryFeatures.ready)
    {
        Serial.println("Battery feature data is incomplete.");
    }
    else
    {
        const char* batteryFeatureNames[11] =
        {
            "CONTROL_MODULE_VOLTAGE_V_MEAN",
            "CONTROL_MODULE_VOLTAGE_V_STD",
            "CONTROL_MODULE_VOLTAGE_V_MIN",
            "CONTROL_MODULE_VOLTAGE_V_MAX",
            "CONTROL_MODULE_VOLTAGE_V_RANGE",
            "VOLTAGE_RANGE",
            "VOLTAGE_DEVIATION_FROM_14",
            "LOW_VOLTAGE_COUNT_LT_12_8",
            "HIGH_VOLTAGE_COUNT_GT_15_0",
            "BELOW_CHARGING_COUNT_LT_13_0",
            "ABOVE_CHARGING_COUNT_GT_14_8"
        };

        for (size_t i = 0; i < 11; i++)
        {
            printFeature(batteryFeatureNames[i], batteryFeatures.values[i]);
        }
    }


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



// setup
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



// main loop
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
    VehicleSample& currentSample = caseSamples[sampleIndex];

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

        // Reset to the beginning of the array for a new non-overlapping twenty-sample case
        sampleIndex = 0;
    }
}
