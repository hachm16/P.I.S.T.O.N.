#include "PistonBackend.h"

#include <QCoreApplication>
#include <QDebug>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QPermissions>
#include <QUuid>
#include <QJsonArray>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QStandardPaths>
#include <initializer_list>
#include <QSaveFile>
#include <QHash>

#ifdef Q_OS_ANDROID
#include <QJniObject>
#endif

const QString PistonBackend::kDeviceName =
    QStringLiteral("P.I.S.T.O.N.");

const QBluetoothUuid PistonBackend::kServiceUuid(
    QUuid(QStringLiteral("{6c31c5f0-7d91-4d52-9f7b-23d2c7059a10}")));

const QBluetoothUuid PistonBackend::kTxCharacteristicUuid(
    QUuid(QStringLiteral("{6c31c5f1-7d91-4d52-9f7b-23d2c7059a10}")));

const QBluetoothUuid PistonBackend::kRxCharacteristicUuid(
    QUuid(QStringLiteral("{6c31c5f2-7d91-4d52-9f7b-23d2c7059a10}")));

namespace
{

// ------------------------------------------------------------
//DTC descriptions
// ------------------------------------------------------------

const QHash<QString, QString> DTC_DESCRIPTIONS =
    {
        {"P0010", "Camshaft timing actuator circuit problem for Bank 1. The engine computer detected an electrical fault in the camshaft timing control circuit."},
        {"P0011", "Bank 1 intake camshaft timing is more advanced than expected. This can be related to variable valve timing operation, oil flow, or camshaft timing control."},
        {"P0012", "Bank 1 intake camshaft timing is more delayed than expected. This can be related to variable valve timing operation, oil flow, or camshaft timing control."},
        {"P0013", "Camshaft timing actuator circuit problem for Bank 1 exhaust camshaft. The engine computer detected an electrical fault in the timing control circuit."},
        {"P0014", "Bank 1 exhaust camshaft timing is more advanced than expected. This can be related to variable valve timing operation, oil flow, or camshaft timing control."},
        {"P0016", "The crankshaft and Bank 1 intake camshaft positions are not matching as expected. This can indicate a timing, sensor, or variable valve timing problem."},
        {"P0017", "The crankshaft and Bank 1 exhaust camshaft positions are not matching as expected. This can indicate a timing, sensor, or variable valve timing problem."},

        {"P0100", "Mass airflow sensor circuit problem. The engine computer detected an electrical issue with the sensor used to measure incoming air."},
        {"P0101", "Mass airflow sensor readings are outside the expected operating range. Airflow measurements may not agree with other engine operating conditions."},
        {"P0102", "Mass airflow sensor signal is lower than expected."},
        {"P0103", "Mass airflow sensor signal is higher than expected."},

        {"P0105", "Manifold pressure sensor circuit problem. The engine computer detected an electrical issue with the intake manifold pressure signal."},
        {"P0106", "Manifold pressure sensor readings are outside the expected operating range."},
        {"P0107", "Manifold pressure sensor signal is lower than expected."},
        {"P0108", "Manifold pressure sensor signal is higher than expected."},

        {"P0112", "Intake air temperature sensor signal is lower than expected, usually indicating an unusually high reported air temperature or an electrical circuit issue."},
        {"P0113", "Intake air temperature sensor signal is higher than expected, usually indicating an unusually low reported air temperature or an electrical circuit issue."},
        {"P0117", "Engine coolant temperature sensor signal is lower than expected."},
        {"P0118", "Engine coolant temperature sensor signal is higher than expected."},

        {"P0121", "Throttle position sensor reading is outside the expected operating range."},
        {"P0122", "Throttle position sensor signal is lower than expected."},
        {"P0123", "Throttle position sensor signal is higher than expected."},

        {"P0130", "Oxygen sensor circuit problem for Bank 1, Sensor 1. This is the sensor before the catalytic converter."},
        {"P0131", "Bank 1, Sensor 1 oxygen sensor voltage is lower than expected."},
        {"P0132", "Bank 1, Sensor 1 oxygen sensor voltage is higher than expected."},
        {"P0133", "Bank 1, Sensor 1 oxygen sensor is responding more slowly than expected."},
        {"P0134", "Little or no activity was detected from the Bank 1, Sensor 1 oxygen sensor."},
        {"P0135", "Heater circuit problem for the Bank 1, Sensor 1 oxygen sensor."},

        {"P0136", "Oxygen sensor circuit problem for Bank 1, Sensor 2. This is normally the sensor after the catalytic converter."},
        {"P0137", "Bank 1, Sensor 2 oxygen sensor voltage is lower than expected."},
        {"P0138", "Bank 1, Sensor 2 oxygen sensor voltage is higher than expected."},
        {"P0139", "Bank 1, Sensor 2 oxygen sensor is responding more slowly than expected."},
        {"P0140", "Little or no activity was detected from the Bank 1, Sensor 2 oxygen sensor."},
        {"P0141", "Heater circuit problem for the Bank 1, Sensor 2 oxygen sensor. This is normally the sensor after the catalytic converter."},

        {"P0150", "Oxygen sensor circuit problem for Bank 2, Sensor 1. This is the sensor before the catalytic converter."},
        {"P0151", "Bank 2, Sensor 1 oxygen sensor voltage is lower than expected."},
        {"P0152", "Bank 2, Sensor 1 oxygen sensor voltage is higher than expected."},
        {"P0153", "Bank 2, Sensor 1 oxygen sensor is responding more slowly than expected."},
        {"P0154", "Little or no activity was detected from the Bank 2, Sensor 1 oxygen sensor."},
        {"P0155", "Heater circuit problem for the Bank 2, Sensor 1 oxygen sensor."},

        {"P0156", "Oxygen sensor circuit problem for Bank 2, Sensor 2. This is normally the sensor after the catalytic converter."},
        {"P0157", "Bank 2, Sensor 2 oxygen sensor voltage is lower than expected."},
        {"P0158", "Bank 2, Sensor 2 oxygen sensor voltage is higher than expected."},
        {"P0159", "Bank 2, Sensor 2 oxygen sensor is responding more slowly than expected."},
        {"P0160", "Little or no activity was detected from the Bank 2, Sensor 2 oxygen sensor."},
        {"P0161", "Heater circuit problem for the Bank 2, Sensor 2 oxygen sensor."},

        {"P0171", "The engine is correcting for a lean air-fuel condition on Bank 1. The engine computer is adding more fuel than expected."},
        {"P0172", "The engine is correcting for a rich air-fuel condition on Bank 1. The engine computer is removing more fuel than expected."},
        {"P0174", "The engine is correcting for a lean air-fuel condition on Bank 2. The engine computer is adding more fuel than expected."},
        {"P0175", "The engine is correcting for a rich air-fuel condition on Bank 2. The engine computer is removing more fuel than expected."},

        {"P0300", "Random or multiple-cylinder misfires were detected. Combustion was inconsistent across one or more cylinders."},
        {"P0301", "A misfire was detected on cylinder 1."},
        {"P0302", "A misfire was detected on cylinder 2."},
        {"P0303", "A misfire was detected on cylinder 3."},
        {"P0304", "A misfire was detected on cylinder 4."},
        {"P0305", "A misfire was detected on cylinder 5."},
        {"P0306", "A misfire was detected on cylinder 6."},
        {"P0307", "A misfire was detected on cylinder 7."},
        {"P0308", "A misfire was detected on cylinder 8."},

        {"P0325", "Knock sensor circuit problem for Bank 1. The engine computer detected an electrical or signal issue with the knock sensor system."},
        {"P0335", "Crankshaft position sensor circuit problem. The engine computer is not receiving the expected crankshaft position signal."},
        {"P0340", "Camshaft position sensor circuit problem for Bank 1. The engine computer is not receiving the expected camshaft position signal."},

        {"P0401", "Exhaust gas recirculation flow is lower than expected."},
        {"P0402", "Exhaust gas recirculation flow is higher than expected."},

        {"P0420", "Catalytic converter efficiency is below the expected level on Bank 1. The exhaust sensor readings suggest the catalyst may not be reducing emissions as effectively as expected."},
        {"P0430", "Catalytic converter efficiency is below the expected level on Bank 2. The exhaust sensor readings suggest the catalyst may not be reducing emissions as effectively as expected."},

        {"P0440", "The evaporative emissions system detected a general fault. This system prevents fuel vapors from escaping into the atmosphere."},
        {"P0442", "A small leak was detected in the evaporative emissions system. This can be caused by a loose seal, hose leak, valve issue, or another small vapor leak."},
        {"P0446", "The evaporative emissions vent control system is not operating as expected."},
        {"P0455", "A large leak was detected in the evaporative emissions system, or the system may not be sealing properly."},
        {"P0456", "A very small leak was detected in the evaporative emissions system."},

        {"P0500", "Vehicle speed sensor signal is missing or outside the expected range."},

        {"P0560", "The vehicle's system voltage is not behaving as expected."},
        {"P0562", "Vehicle system voltage is lower than expected. This may be related to the battery, charging system, wiring, or electrical load."},
        {"P0563", "Vehicle system voltage is higher than expected. This may be related to charging-system regulation or an electrical circuit issue."},

        {"P0606", "The engine or powertrain control module detected an internal processor fault."},
        {"P0700", "The transmission control system has requested a warning because another transmission-related fault is stored."},

        {"U0100", "Communication with the engine or powertrain control module was lost."},
        {"U0101", "Communication with the transmission control module was lost."},
        {"U0121", "Communication with the anti-lock brake or stability-control module was lost."},
        {"U0140", "Communication with the body control module was lost."}
};





// ------------------------------------------------------------
// VIN model-year decoding
// ------------------------------------------------------------
//
// P.I.S.T.O.N. intentionally supports model years 2000-2039.
//
// VIN position 10 contains the model-year code.
// VIN position 7 is used to distinguish the repeating
// 30-year VIN year cycle where applicable.

int decodeVinModelYear(const QString& vin)
{
    if (vin.length() != 17)
    {
        return -1;
    }


    const QChar position7 =
        vin.at(6).toUpper();

    const QChar yearCode =
        vin.at(9).toUpper();


    // --------------------------------------------------------
    // 2001-2009 / 2031-2039
    // --------------------------------------------------------

    if (yearCode.isDigit())
    {
        const int digit =
            yearCode.digitValue();

        if (digit < 1 ||
            digit > 9)
        {
            return -1;
        }


        if (position7.isDigit())
        {
            return 2000 + digit;
        }


        if (position7.isLetter())
        {
            return 2030 + digit;
        }


        return -1;
    }


    // --------------------------------------------------------
    // 2010-2030
    // --------------------------------------------------------

    static const QString yearCodes =
        QStringLiteral(
            "ABCDEFGHJKLMNPRSTVWXY");


    const int yearCodeIndex =
        yearCodes.indexOf(yearCode);


    if (yearCodeIndex < 0)
    {
        return -1;
    }


    if (position7.isLetter())
    {
        return 2010 + yearCodeIndex;
    }


    // Y is also the 2000 model-year code.
    if (position7.isDigit() &&
        yearCode == QLatin1Char('Y'))
    {
        return 2000;
    }


    return -1;
}


// ------------------------------------------------------------
// Offline make lookup
// ------------------------------------------------------------

bool wmiIsAny(
    const QString& wmi,
    std::initializer_list<const char*> values)
{
    for (const char* value : values)
    {
        if (wmi ==
            QString::fromLatin1(value))
        {
            return true;
        }
    }

    return false;
}


QString lookupMakeFromVin(
    const QString& vin,
    int modelYear)
{
    if (vin.length() != 17)
    {
        return QString();
    }


    const QString wmi =
        vin.left(3).toUpper();


    // --------------------------------------------------------
    // Stellantis / Chrysler-family special cases
    //
    // Some WMIs are shared across multiple consumer makes.
    // Handle only cases we can reasonably distinguish.
    // --------------------------------------------------------

    if (wmi == QStringLiteral("1C6"))
    {
        const QChar fourthCharacter =
            vin.at(3).toUpper();

        // Modern Jeep Gladiator VINs use 1C6 with
        // H/J vehicle descriptor prefixes.
        if (fourthCharacter == QLatin1Char('H') ||
            fourthCharacter == QLatin1Char('J'))
        {
            return QStringLiteral("Jeep");
        }

        if (modelYear >= 2010)
        {
            return QStringLiteral("Ram");
        }

        return QStringLiteral("Dodge");
    }


    if (wmi == QStringLiteral("3C6") ||
        wmi == QStringLiteral("3C7") ||
        wmi == QStringLiteral("1D7") ||
        wmi == QStringLiteral("3D7"))
    {
        if (modelYear >= 2010)
        {
            return QStringLiteral("Ram");
        }

        return QStringLiteral("Dodge");
    }


    if (wmi == QStringLiteral("1C4") &&
        modelYear >= 2011)
    {
        const QString positions5And6 =
            vin.mid(4, 2);

        // Dodge Durango:
        // DH = rear-wheel drive
        // DJ = all-wheel drive
        if (positions5And6 ==
                QStringLiteral("DH") ||
            positions5And6 ==
                QStringLiteral("DJ"))
        {
            return QStringLiteral("Dodge");
        }

        return QStringLiteral("Jeep");
    }


    if (wmi == QStringLiteral("2C4"))
    {
        const QString prefix5 =
            vin.left(5).toUpper();

        if (prefix5 ==
            QStringLiteral("2C4RC"))
        {
            return QStringLiteral("Chrysler");
        }

        if (prefix5 ==
            QStringLiteral("2C4RD"))
        {
            return QStringLiteral("Dodge");
        }

        // Ambiguous Chrysler/Dodge WMI.
        return QString();
    }


    if (wmi == QStringLiteral("3C3"))
    {
        if (modelYear >= 2012)
        {
            return QStringLiteral("Fiat");
        }

        return QStringLiteral("Chrysler");
    }


    if (wmi == QStringLiteral("ZFB"))
    {
        const QString positions5To7 =
            vin.mid(4, 3);

        if (positions5To7 ==
                QStringLiteral("RFA") ||
            positions5To7 ==
                QStringLiteral("RFB") ||
            positions5To7 ==
                QStringLiteral("RFC") ||
            positions5To7 ==
                QStringLiteral("RFD"))
        {
            return QStringLiteral("Ram");
        }

        if (positions5To7.startsWith(
                QStringLiteral("FA")) ||
            positions5To7.startsWith(
                QStringLiteral("FX")) ||
            positions5To7.startsWith(
                QStringLiteral("FY")))
        {
            return QStringLiteral("Fiat");
        }

        return QString();
    }


    // 5TD was overwhelmingly Toyota in the portion
    // of our supported range before newer Lexus usage.
    if (wmi == QStringLiteral("5TD"))
    {
        if (modelYear >= 2000 &&
            modelYear <= 2023)
        {
            return QStringLiteral("Toyota");
        }

        return QString();
    }


    // 5NM is common Hyundai territory, but newer
    // Genesis production can also use it.
    if (wmi == QStringLiteral("5NM"))
    {
        // Alabama-built Genesis GV70
        if (vin.startsWith(QStringLiteral("5NMM")))
        {
            return QStringLiteral("Genesis");
        }

        return QStringLiteral("Hyundai");
    }


    // --------------------------------------------------------
    // Toyota
    // --------------------------------------------------------

    if (wmiIsAny(
            wmi,
            {
                "1NX",
                "2T1",
                "2T3",
                "3TM",
                "3TY",
                "4T1",
                "4T3",
                "5TB",
                "5TF",
                "5YF",
                "JTD",
                "JTE",
                "7MU",
                "JTK",
                "JTL",
                "JTM"
            }))
    {
        return QStringLiteral("Toyota");
    }


    // --------------------------------------------------------
    // Volkswagen
    // --------------------------------------------------------

    if (wmiIsAny(
            wmi,
            {
                "1V2",
                "1VW",
                "3VV",
                "3VW",
                "9BW",
                "WV1",
                "WV2",
                "WVG",
                "WVW"
            }))
    {
        return QStringLiteral("Volkswagen");
    }


    // --------------------------------------------------------
    // Ford
    // --------------------------------------------------------

    if (wmiIsAny(
            wmi,
            {
                "1F1",
                "1FA",
                "1FB",
                "1FC",
                "1FD",
                "1FM",
                "1FT",
                "1ZV",
                "2FA",
                "2FD",
                "2FM",
                "2FT",
                "3FA",
                "3FD",
                "3FT",
                "MAJ",
                "NM0",
                "3FM",
                "WF0"
            }))
    {
        return QStringLiteral("Ford");
    }


    // --------------------------------------------------------
    // Honda
    // --------------------------------------------------------

    if (wmiIsAny(
            wmi,
            {
                "19X",
                "1HG",
                "2HG",
                "2HK",
                "3CZ",
                "3HG",
                "5FN",
                "5FP",
                "5J6",
                "5FY",
                "7FA",
                "JHL",
                "JHM",
                "SHH",
                "JH1",
                "5J7",
                "5KB",
                "2HJ",
                "SHS"
            }))
    {
        return QStringLiteral("Honda");
    }


    // --------------------------------------------------------
    // Hyundai
    // --------------------------------------------------------

    if (wmiIsAny(
            wmi,
            {
                "2HM",
                "5NP",
                "5NT",
                "KM8",
                "KMF",
                "KMH"
            }))
    {
        return QStringLiteral("Hyundai");
    }


    // --------------------------------------------------------
    // Chevrolet
    // --------------------------------------------------------

    if (wmiIsAny(
            wmi,
            {
                "1G1",
                "1GB",
                "1GC",
                "1GG",
                "1GN",
                "2G1",
                "2GN",
                "3G1",
                "3GC",
                "3GN",
                "KL1",
                "KL7",
                "6G3",
                "KL8"
            }))
    {
        return QStringLiteral("Chevrolet");
    }


    // --------------------------------------------------------
    // Nissan
    // --------------------------------------------------------

    if (wmiIsAny(
            wmi,
            {
                "1N4",
                "1N6",
                "3N1",
                "3N6",
                "5BB",
                "JN6"
            }))
    {
        return QStringLiteral("Nissan");
    }


    // --------------------------------------------------------
    // Kia
    // --------------------------------------------------------

    // Kia special case (5xy isnt always Kia
    // (check for 5xyz for unique Hyundai models)
    if (wmi == QStringLiteral("5XY") &&
        vin.startsWith(QStringLiteral("5XYZ")))
    {
        return QStringLiteral("Hyundai");
    }

    // Kia list
    if (wmiIsAny(
            wmi,
            {
                "3KP",
                "5XX",
                "5XY",
                "KNA",
                "KNC",
                "KND",
                "KNM"
            }))
    {
        return QStringLiteral("Kia");
    }


    // --------------------------------------------------------
    // Suzuki
    // --------------------------------------------------------

    if (wmiIsAny(
            wmi,
            {
                "2S3",
                "JS2",
                "JS3",
                "KL5"
            }))
    {
        return QStringLiteral("Suzuki");
    }


    // --------------------------------------------------------
    // Mercedes-Benz
    // --------------------------------------------------------

    if (wmiIsAny(
            wmi,
            {
                "4JC",
                "4JG",
                "55S",
                "W1K",
                "W1N",
                "W1V",
                "WDB",
                "WDC",
                "WDD",
                "WDF"
            }))
    {
        return QStringLiteral("Mercedes-Benz");
    }


    // --------------------------------------------------------
    // BMW
    // --------------------------------------------------------

    if (wmiIsAny(
            wmi,
            {
                "3MW",
                "4US",
                "5UX",
                "5YM",
                "WBA",
                "WBS",
                "WBX",
                "3MF",
                "5UM",
                "WBY"
            }))
    {
        return QStringLiteral("BMW");
    }


    // --------------------------------------------------------
    // Fiat
    // --------------------------------------------------------

    if (wmiIsAny(
            wmi,
            {
                "ZFA",
                "ZFC"
            }))
    {
        return QStringLiteral("Fiat");
    }


    // --------------------------------------------------------
    // Mazda
    // --------------------------------------------------------

    if (wmiIsAny(
            wmi,
            {
                "1YV",
                "3MV",
                "3MZ",
                "4F2",
                "4F4",
                "7MM",
                "JM1",
                "JM3",
                "JM7"
            }))
    {
        return QStringLiteral("Mazda");
    }


    // --------------------------------------------------------
    // Audi
    // --------------------------------------------------------

    if (wmiIsAny(
            wmi,
            {
                "93U",
                "TRU",
                "WA1",
                "WAD",
                "WAU",
                "WUA"
            }))
    {
        return QStringLiteral("Audi");
    }



    // --------------------------------------------------------
    // Jeep
    // --------------------------------------------------------
    if (wmiIsAny(
            wmi,
            {
                "1J4",
                "1J7",
                "1J8",
                "3C4"
            }))
    {
        return QStringLiteral("Jeep");
    }


    // (Jeep special case)
    if (wmi == QStringLiteral("ZAC"))
    {
        if (vin.startsWith(
                QStringLiteral("ZACN")))
        {
            return QStringLiteral("Dodge");
        }

        if (vin.startsWith(
                QStringLiteral("ZACC")))
        {
            return QStringLiteral("Jeep");
        }

        return QString();
    }


    // --------------------------------------------------------
    // GMC
    // --------------------------------------------------------

    if (wmiIsAny(
            wmi,
            {
                "1GD",
                "1GK",
                "1GT",
                "2GK",
                "3GT",
                "3GK"
            }))
    {
        return QStringLiteral("GMC");
    }


    // --------------------------------------------------------
    // Buick
    // --------------------------------------------------------

    if (wmiIsAny(
            wmi,
            {
                "1G4",
                "2G4",
                "3G5",
                "5GA",
                "KL4",
                "LRB"
            }))
    {
        return QStringLiteral("Buick");
    }


    // --------------------------------------------------------
    // Subaru
    // --------------------------------------------------------

    if (wmiIsAny(
            wmi,
            {
                "4S3",
                "4S4",
                "4S6",
                "JF1",
                "JF2",
                "JF3"
            }))
    {
        return QStringLiteral("Subaru");
    }


    // --------------------------------------------------------
    // Lexus
    // --------------------------------------------------------

    if (wmiIsAny(
            wmi,
            {
                "2T2",
                "58A",
                "58B",
                "JT8",
                "JTH",
                "JTJ"
            }))
    {
        return QStringLiteral("Lexus");
    }


    // --------------------------------------------------------
    // Volvo
    // --------------------------------------------------------

    if (wmiIsAny(
            wmi,
            {
                "7JR",
                "LVY",
                "YV1",
                "YV4"
            }))
    {
        return QStringLiteral("Volvo");
    }


    // --------------------------------------------------------
    // Dodge
    // --------------------------------------------------------

    if (wmiIsAny(
            wmi,
            {
                "1B3",
                "1D4",
                "1D8",
                "2B3",
                "3D4",
                "2B4"
            }))
    {
        return QStringLiteral("Dodge");
    }


    // --------------------------------------------------------
    // Mitsubishi
    // --------------------------------------------------------

    if (wmiIsAny(
            wmi,
            {
                "4A3",
                "4A4",
                "JA3",
                "JA4",
                "ML0",
                "ML3"
            }))
    {
        return QStringLiteral("Mitsubishi");
    }


    // --------------------------------------------------------
    // Cadillac
    // --------------------------------------------------------

    if (wmiIsAny(
            wmi,
            {
                "1G6",
                "1GE",
                "1GY",
                "2G6",
                "3GY",
                "3G0"
            }))
    {
        return QStringLiteral("Cadillac");
    }


    // --------------------------------------------------------
    // Lincoln
    // --------------------------------------------------------

    if (wmiIsAny(
            wmi,
            {
                "1LN",
                "2LM",
                "5L1",
                "5LJ",
                "5LM",
                "5LT",
                "2LN",
                "3LN"
            }))
    {
        return QStringLiteral("Lincoln");
    }


    // --------------------------------------------------------
    // Acura
    // --------------------------------------------------------

    if (wmiIsAny(
            wmi,
            {
                "19U",
                "19V",
                "2HN",
                "5FR",
                "5J8",
                "2HH",
                "2HU",
                "5J0",
                "5KC",
                "5FS",
                "JH4"
            }))
    {
        return QStringLiteral("Acura");
    }


    // --------------------------------------------------------
    // Land Rover
    // --------------------------------------------------------

    if (wmi ==
        QStringLiteral("SAL"))
    {
        return QStringLiteral("Land Rover");
    }


    // --------------------------------------------------------
    // Mini
    // --------------------------------------------------------

    if (wmiIsAny(
            wmi,
            {
                "WMW",
                "WMZ"
            }))
    {
        return QStringLiteral("Mini");
    }


    // --------------------------------------------------------
    // Porsche
    // --------------------------------------------------------

    if (wmiIsAny(
            wmi,
            {
                "WP0",
                "WP1"
            }))
    {
        return QStringLiteral("Porsche");
    }


    // --------------------------------------------------------
    // Genesis
    // --------------------------------------------------------

    if (wmiIsAny(
            wmi,
            {
                "KMT",
                "KMU"
            }))
    {
        return QStringLiteral("Genesis");
    }


    // --------------------------------------------------------
    // Chrysler
    // --------------------------------------------------------

    if (wmiIsAny(
            wmi,
            {
                "1C3",
                "2A4",
                "2C3"
            }))
    {
        return QStringLiteral("Chrysler");
    }


    // --------------------------------------------------------
    // Alfa Romeo
    // --------------------------------------------------------

    if (wmiIsAny(
            wmi,
            {
                "ZAB",
                "ZAR",
                "ZAS"
            }))
    {
        return QStringLiteral("Alfa Romeo");
    }


    // --------------------------------------------------------
    // Jaguar
    // --------------------------------------------------------

    if (wmiIsAny(
            wmi,
            {
                "SAD",
                "SAJ"
            }))
    {
        return QStringLiteral("Jaguar");
    }


    // --------------------------------------------------------
    // Smart
    // --------------------------------------------------------

    if (wmi ==
        QStringLiteral("WME"))
    {
        return QStringLiteral("Smart");
    }


    // --------------------------------------------------------
    // Infiniti
    // --------------------------------------------------------

    if (wmiIsAny(
            wmi,
            {
                "5N3",
                "JNK",
                "JNR",
                "SJK"
            }))
    {
        return QStringLiteral("Infiniti");
    }


    // --------------------------------------------------------
    // Maserati
    // --------------------------------------------------------

    if (wmiIsAny(
            wmi,
            {
                "ZAM",
                "ZN6"
            }))
    {
        return QStringLiteral("Maserati");
    }


    // --------------------------------------------------------
    // Pontiac
    // --------------------------------------------------------

    if (wmiIsAny(
            wmi,
            {
                "1G2",
                "2G2",
                "5Y2",
                "6G2"
            }))
    {
        return QStringLiteral("Pontiac");
    }


    // --------------------------------------------------------
    // Saturn
    // --------------------------------------------------------

    if (wmiIsAny(
            wmi,
            {
                "1G8",
                "3GS",
                "5GZ",
                "W08"
            }))
    {
        return QStringLiteral("Saturn");
    }


    // --------------------------------------------------------
    // Mercury
    // --------------------------------------------------------

    if (wmiIsAny(
            wmi,
            {
                "1ME",
                "1MH",
                "2ME",
                "2MH",
                "4M2"
            }))
    {
        return QStringLiteral("Mercury");
    }


    // Known ambiguous or unsupported WMI.
    //
    // Examples we deliberately do NOT guess on include
    // JN1 / JN8 / 5N1 (Nissan or Infiniti) and
    // JTN (Toyota or Lexus).
    return QString();
}

}

PistonBackend::PistonBackend(QObject* parent)
    : QObject(parent)
{
    m_liveDataModel =
        new LiveDataModel(this);

    m_fuelHistoryModel = new DiagnosticHistoryModel(this);

    m_catalystHistoryModel = new DiagnosticHistoryModel(this);

    m_chargingHistoryModel = new DiagnosticHistoryModel(this);

    loadSessionFromDisk();

    m_discoveryAgent = new QBluetoothDeviceDiscoveryAgent(this);

    m_discoveryAgent->setLowEnergyDiscoveryTimeout(10000);

    connect(
        m_discoveryAgent,
        &QBluetoothDeviceDiscoveryAgent::deviceDiscovered,
        this,
        &PistonBackend::onDeviceDiscovered);

    connect(
        m_discoveryAgent,
        &QBluetoothDeviceDiscoveryAgent::finished,
        this,
        &PistonBackend::onScanFinished);

    connect(
        m_discoveryAgent,
        &QBluetoothDeviceDiscoveryAgent::errorOccurred,
        this,
        &PistonBackend::onScanError);
}

PistonBackend::~PistonBackend()
{
    if (m_discoveryAgent != nullptr &&
        m_discoveryAgent->isActive())
    {
        m_discoveryAgent->stop();
    }

    if (m_controller != nullptr)
    {
        m_controller->disconnectFromDevice();
    }
}

bool PistonBackend::connected() const
{
    return m_connected;
}

QString PistonBackend::connectionState() const
{
    return m_connectionState;
}

QString PistonBackend::connectedDeviceName() const
{
    return m_connectedDeviceName;
}

int PistonBackend::caseCount() const
{
    return m_caseCount;
}

int PistonBackend::sampleCount() const
{
    return m_sampleCount;
}

bool PistonBackend::vehicleInfoAvailable() const
{
    return m_vehicleInfoAvailable;
}

QString PistonBackend::vehicleVin() const
{
    return m_vehicleVin;
}

QString PistonBackend::vehicleDisplayName() const
{
    return m_vehicleDisplayName;
}

QString PistonBackend::vehicleDetails() const
{
    return m_vehicleDetails;
}

QAbstractItemModel* PistonBackend::liveDataModel() const
{
    return m_liveDataModel;
}

QAbstractItemModel* PistonBackend::fuelHistoryModel() const
{
    return m_fuelHistoryModel;
}

QAbstractItemModel* PistonBackend::catalystHistoryModel() const
{
    return m_catalystHistoryModel;
}

QAbstractItemModel* PistonBackend::chargingHistoryModel() const
{
    return m_chargingHistoryModel;
}

QString PistonBackend::latestFuelCondition() const
{
    return m_latestFuelCondition;
}

QString PistonBackend::latestFuelActionLevel() const
{
    return m_latestFuelActionLevel;
}

QString PistonBackend::latestCatalystCondition() const
{
    return m_latestCatalystCondition;
}

QString PistonBackend::latestCatalystActionLevel() const
{
    return m_latestCatalystActionLevel;
}

QString PistonBackend::latestChargingCondition() const
{
    return m_latestChargingCondition;
}

QString PistonBackend::latestChargingActionLevel() const
{
    return m_latestChargingActionLevel;
}

bool PistonBackend::loggingEnabled() const
{
    return m_loggingEnabled;
}


bool PistonBackend::loggingStateKnown() const
{
    return m_loggingStateKnown;
}

QStringList PistonBackend::confirmedDtcs() const
{
    return m_confirmedDtcs;
}

QStringList PistonBackend::pendingDtcs() const
{
    return m_pendingDtcs;
}

QStringList PistonBackend::permanentDtcs() const
{
    return m_permanentDtcs;
}

bool PistonBackend::confirmedDtcResponseReceived() const
{
    return m_confirmedDtcResponseReceived;
}

bool PistonBackend::pendingDtcResponseReceived() const
{
    return m_pendingDtcResponseReceived;
}

bool PistonBackend::permanentDtcResponseReceived() const
{
    return m_permanentDtcResponseReceived;
}

bool PistonBackend::dtcScanInProgress() const
{
    return m_dtcScanInProgress;
}

bool PistonBackend::dtcResultAvailable() const
{
    return m_dtcResultAvailable;
}

QString PistonBackend::dtcError() const
{
    return m_dtcError;
}

QString PistonBackend::dtcDescription(const QString& code) const
{
    const QString normalizedCode = code.trimmed().toUpper();
    const auto description = DTC_DESCRIPTIONS.constFind(normalizedCode);

    if (description != DTC_DESCRIPTIONS.constEnd())
    {
        return description.value();
    }

    return QStringLiteral(
        "A detailed description for this trouble code is not stored in the offline P.I.S.T.O.N. database. "
        "Refer to the vehicle manufacturer's service information for the exact definition.");
}

void PistonBackend::connectToPiston()
{
    if (m_connected)
    {
        return;
    }

    if (m_connectionState == QStringLiteral("Scanning") ||
        m_connectionState == QStringLiteral("Connecting") ||
        m_connectionState == QStringLiteral("Discovering services") ||
        m_connectionState == QStringLiteral("Discovering characteristics") ||
        m_connectionState == QStringLiteral("Subscribing"))
    {
        return;
    }

    requestBluetoothAccessAndStartScan();
}

void PistonBackend::disconnectFromPiston()
{
    if (m_discoveryAgent != nullptr &&
        m_discoveryAgent->isActive())
    {
        m_discoveryAgent->stop();
    }

    if (m_controller != nullptr &&
        m_controller->state() !=
            QLowEnergyController::UnconnectedState)
    {
        setConnectionState(QStringLiteral("Disconnecting"));
        m_controller->disconnectFromDevice();
        return;
    }

    setConnected(false);
    setConnectionState(QStringLiteral("Disconnected"));
}

QString PistonBackend::sessionFilePath() const
{
    const QString appDataPath =
        QStandardPaths::writableLocation(
            QStandardPaths::AppDataLocation);

    if (appDataPath.isEmpty())
    {
        return QString();
    }

    return QDir(appDataPath).filePath(
        QStringLiteral("piston_session.json"));
}


void PistonBackend::saveSessionToDisk() const
{
    const bool hasSessionData =
        !m_persistedCases.isEmpty() ||
        m_dtcResultAvailable ||
        m_vehicleInfoAvailable;

    if (!hasSessionData)
    {
        return;
    }


    const QString appDataPath =
        QStandardPaths::writableLocation(
            QStandardPaths::AppDataLocation);

    if (appDataPath.isEmpty())
    {
        qWarning()
        << "P.I.S.T.O.N. app data path unavailable.";

        return;
    }


    QDir directory(appDataPath);

    if (!directory.exists() &&
        !directory.mkpath(QStringLiteral(".")))
    {
        qWarning()
        << "Could not create P.I.S.T.O.N. app data directory.";

        return;
    }


    QJsonObject root;

    root["version"] = 1;

    root["cases"] =
        m_persistedCases;


    // --------------------------------------------------------
    // DTC state
    // --------------------------------------------------------

    QJsonObject dtc;

    dtc["result_available"] =
        m_dtcResultAvailable;

    dtc["confirmed_response"] =
        m_confirmedDtcResponseReceived;

    dtc["pending_response"] =
        m_pendingDtcResponseReceived;

    dtc["permanent_response"] =
        m_permanentDtcResponseReceived;

    dtc["confirmed"] =
        QJsonArray::fromStringList(
            m_confirmedDtcs);

    dtc["pending"] =
        QJsonArray::fromStringList(
            m_pendingDtcs);

    dtc["permanent"] =
        QJsonArray::fromStringList(
            m_permanentDtcs);

    dtc["error"] =
        m_dtcError;

    root["dtc"] =
        dtc;


    // --------------------------------------------------------
    // Vehicle information
    // --------------------------------------------------------

    QJsonObject vehicle;

    vehicle["available"] =
        m_vehicleInfoAvailable;

    vehicle["vin"] =
        m_vehicleVin;

    vehicle["display_name"] =
        m_vehicleDisplayName;

    vehicle["details"] =
        m_vehicleDetails;

    root["vehicle"] =
        vehicle;


    // --------------------------------------------------------
    // Atomic write
    // --------------------------------------------------------

    const QString filePath =
        directory.filePath(
            QStringLiteral(
                "piston_session.json"));


    QSaveFile file(filePath);

    if (!file.open(
            QIODevice::WriteOnly |
            QIODevice::Truncate))
    {
        qWarning()
        << "Could not open persistent session file:"
        << file.errorString();

        return;
    }


    const QByteArray data =
        QJsonDocument(root)
            .toJson(
                QJsonDocument::Compact);


    if (file.write(data) !=
        data.size())
    {
        qWarning()
        << "Could not write complete persistent session.";

        file.cancelWriting();

        return;
    }


    if (!file.commit())
    {
        qWarning()
        << "Could not commit persistent session:"
        << file.errorString();

        return;
    }


    qDebug()
        << "P.I.S.T.O.N. session saved."
        << "Cases:"
        << m_persistedCases.size();
}


void PistonBackend::restoreCaseFromJson(
    const QJsonObject& caseObject,
    int sessionCaseNumber)
{
    if (caseObject.value("type").toString() !=
        QStringLiteral("case_complete"))
    {
        return;
    }


    if (caseObject.value(
                      "protocol_version").toInt() != 1)
    {
        return;
    }


    // --------------------------------------------------------
    // Raw samples
    // --------------------------------------------------------

    const QJsonValue bank2KnownValue =
        caseObject.value("bank2_availability_known");

    const QJsonValue hasBank2Value =
        caseObject.value("has_bank_2");

    if (bank2KnownValue.isBool() && hasBank2Value.isBool())
    {
        m_liveDataModel->setBank2Availability(
            bank2KnownValue.toBool(),
            hasBank2Value.toBool());
    }

    const QJsonArray samples =
        caseObject.value(
                      "samples").toArray();

    m_liveDataModel->appendSamples(
        sessionCaseNumber,
        samples);


    // --------------------------------------------------------
    // Diagnostic outputs
    // --------------------------------------------------------

    const QJsonObject outputs =
        caseObject.value(
                      "outputs").toObject();


    // Fuel
    const QJsonObject fuel =
        outputs.value(
                   "fuel").toObject();

    m_latestFuelCondition =
        fuel.value("condition")
            .toString(
                QStringLiteral(
                    "Unavailable"));

    m_latestFuelActionLevel =
        fuel.value("action_level")
            .toString(
                QStringLiteral(
                    "Unavailable"));

    m_fuelHistoryModel->prependResult(
        sessionCaseNumber,
        m_latestFuelCondition,
        m_latestFuelActionLevel,
        fuel.value("evidence_source")
            .toString(
                QStringLiteral("None")),
        fuel.value("description")
            .toString());


    // Catalyst
    const QJsonObject catalyst =
        outputs.value(
                   "catalyst").toObject();

    m_latestCatalystCondition =
        catalyst.value("condition")
            .toString(
                QStringLiteral(
                    "Unavailable"));

    m_latestCatalystActionLevel =
        catalyst.value("action_level")
            .toString(
                QStringLiteral(
                    "Unavailable"));

    m_catalystHistoryModel->prependResult(
        sessionCaseNumber,
        m_latestCatalystCondition,
        m_latestCatalystActionLevel,
        catalyst.value("evidence_source")
            .toString(
                QStringLiteral("None")),
        catalyst.value("description")
            .toString());


    // Charging
    const QJsonObject charging =
        outputs.value(
                   "charging").toObject();

    m_latestChargingCondition =
        charging.value("condition")
            .toString(
                QStringLiteral(
                    "Unavailable"));

    m_latestChargingActionLevel =
        charging.value("action_level")
            .toString(
                QStringLiteral(
                    "Unavailable"));

    m_chargingHistoryModel->prependResult(
        sessionCaseNumber,
        m_latestChargingCondition,
        m_latestChargingActionLevel,
        charging.value("evidence_source")
            .toString(
                QStringLiteral("None")),
        charging.value("description")
            .toString());


    ++m_caseCount;

    m_sampleCount +=
        samples.size();
}


void PistonBackend::loadSessionFromDisk()
{
    const QString filePath =
        sessionFilePath();

    if (filePath.isEmpty() ||
        !QFile::exists(filePath))
    {
        return;
    }


    QFile file(filePath);

    if (!file.open(
            QIODevice::ReadOnly))
    {
        qWarning()
        << "Could not open saved P.I.S.T.O.N. session.";

        return;
    }


    QJsonParseError parseError;

    const QJsonDocument document =
        QJsonDocument::fromJson(
            file.readAll(),
            &parseError);

    file.close();


    if (parseError.error !=
            QJsonParseError::NoError ||
        !document.isObject())
    {
        qWarning()
        << "Saved P.I.S.T.O.N. session is invalid:"
        << parseError.errorString();

        return;
    }


    const QJsonObject root =
        document.object();

    if (root.value("version").toInt() != 1)
    {
        qWarning()
        << "Unsupported saved session version.";

        return;
    }


    // --------------------------------------------------------
    // Restore completed cases
    // --------------------------------------------------------

    const QJsonArray cases =
        root.value(
                "cases").toArray();

    m_persistedCases =
        QJsonArray();


    for (const QJsonValue& caseValue : cases)
    {
        if (!caseValue.isObject())
        {
            continue;
        }


        const QJsonObject caseObject =
            caseValue.toObject();


        const int previousCaseCount =
            m_caseCount;


        restoreCaseFromJson(
            caseObject,
            m_caseCount + 1);


        if (m_caseCount >
            previousCaseCount)
        {
            m_persistedCases.append(
                caseObject);
        }
    }


    // --------------------------------------------------------
    // Restore DTC state
    // --------------------------------------------------------

    const QJsonObject dtc =
        root.value(
                "dtc").toObject();

    m_dtcResultAvailable =
        dtc.value(
               "result_available")
            .toBool(false);

    m_dtcScanInProgress =
        false;

    m_confirmedDtcResponseReceived =
        dtc.value(
               "confirmed_response")
            .toBool(false);

    m_pendingDtcResponseReceived =
        dtc.value(
               "pending_response")
            .toBool(false);

    m_permanentDtcResponseReceived =
        dtc.value(
               "permanent_response")
            .toBool(false);


    m_confirmedDtcs.clear();
    m_pendingDtcs.clear();
    m_permanentDtcs.clear();


    for (const QJsonValue& value :
         dtc.value("confirmed").toArray())
    {
        if (value.isString())
        {
            m_confirmedDtcs.append(
                value.toString());
        }
    }


    for (const QJsonValue& value :
         dtc.value("pending").toArray())
    {
        if (value.isString())
        {
            m_pendingDtcs.append(
                value.toString());
        }
    }


    for (const QJsonValue& value :
         dtc.value("permanent").toArray())
    {
        if (value.isString())
        {
            m_permanentDtcs.append(
                value.toString());
        }
    }


    m_dtcError =
        dtc.value(
               "error").toString();


    // --------------------------------------------------------
    // Restore vehicle identity
    // --------------------------------------------------------

    const QJsonObject vehicle =
        root.value(
                "vehicle").toObject();

    m_vehicleInfoAvailable =
        vehicle.value(
                   "available")
            .toBool(false);

    m_vehicleVin =
        vehicle.value(
                   "vin").toString();

    m_vehicleDisplayName =
        vehicle.value(
                   "display_name").toString();

    m_vehicleDetails =
        vehicle.value(
                   "details").toString();


    qDebug()
        << "P.I.S.T.O.N. saved session restored."
        << "Cases:"
        << m_caseCount
        << "Samples:"
        << m_sampleCount;
}

void PistonBackend::clearSession()
{
    // --------------------------------------------------------
    // Live session data
    // --------------------------------------------------------

    m_liveDataModel->clear();

    m_fuelHistoryModel->clear();
    m_catalystHistoryModel->clear();
    m_chargingHistoryModel->clear();


    m_caseCount = 0;
    m_sampleCount = 0;

    m_persistedCases = QJsonArray();


    // --------------------------------------------------------
    // Latest diagnostic state
    // --------------------------------------------------------

    m_latestFuelCondition.clear();
    m_latestFuelActionLevel.clear();

    m_latestCatalystCondition.clear();
    m_latestCatalystActionLevel.clear();

    m_latestChargingCondition.clear();
    m_latestChargingActionLevel.clear();


    // --------------------------------------------------------
    // DTC state
    // --------------------------------------------------------

    m_confirmedDtcs.clear();
    m_pendingDtcs.clear();
    m_permanentDtcs.clear();

    m_confirmedDtcResponseReceived = false;
    m_pendingDtcResponseReceived = false;
    m_permanentDtcResponseReceived = false;

    m_dtcScanInProgress = false;
    m_dtcResultAvailable = false;

    m_dtcError.clear();


    // --------------------------------------------------------
    // Vehicle identity
    // --------------------------------------------------------

    m_vehicleInfoAvailable = false;

    m_vehicleVin.clear();
    m_vehicleDisplayName.clear();
    m_vehicleDetails.clear();

    m_vehicleInfoRequestPending = false;


    // --------------------------------------------------------
    // Delete persistent session
    // --------------------------------------------------------

    const QString filePath =
        sessionFilePath();

    if (!filePath.isEmpty() && QFile::exists(filePath))
    {
        if (!QFile::remove(filePath))
        {
            qWarning()
                << "Could not delete persistent P.I.S.T.O.N. session.";
        }
    }


    // --------------------------------------------------------
    // Notify GUI
    // --------------------------------------------------------

    emit caseCountChanged();
    emit sampleCountChanged();

    emit latestFuelChanged();
    emit latestCatalystChanged();
    emit latestChargingChanged();

    emit dtcChanged();
    emit vehicleInfoChanged();


    qDebug()
        << "P.I.S.T.O.N. session cleared.";
}

void PistonBackend::exportSessionCsv()
{
    if (m_sampleCount <= 0)
    {
        qWarning()
        << "CSV export requested with no session data.";

        return;
    }


    // --------------------------------------------------------
    // Create CSV
    // --------------------------------------------------------

    const QString csv =
        m_liveDataModel->toCsv();

    if (csv.isEmpty())
    {
        qWarning()
        << "CSV export produced no data.";

        return;
    }


    // --------------------------------------------------------
    // Store temporary CSV in private app cache
    // --------------------------------------------------------

    const QString cachePath =
        QStandardPaths::writableLocation(
            QStandardPaths::CacheLocation);

    if (cachePath.isEmpty())
    {
        qWarning()
        << "Could not locate app cache directory.";

        return;
    }


    QDir cacheDirectory(cachePath);

    if (!cacheDirectory.exists() &&
        !cacheDirectory.mkpath(QStringLiteral(".")))
    {
        qWarning()
        << "Could not create app cache directory.";

        return;
    }


    const QString fileName =
        QStringLiteral("PISTON_Session_%1.csv")
            .arg(
                QDateTime::currentDateTime()
                    .toString(
                        QStringLiteral(
                            "yyyy-MM-dd_HHmmss")));


    const QString filePath =
        cacheDirectory.filePath(fileName);


    QFile file(filePath);

    if (!file.open(
            QIODevice::WriteOnly |
            QIODevice::Truncate))
    {
        qWarning()
        << "Could not create CSV file:"
        << file.errorString();

        return;
    }


    const QByteArray csvBytes =
        csv.toUtf8();

    if (file.write(csvBytes) !=
        csvBytes.size())
    {
        qWarning()
        << "Could not completely write CSV file:"
        << file.errorString();

        file.close();
        return;
    }


    file.close();


    qDebug()
        << "P.I.S.T.O.N. CSV created:"
        << filePath;


    #ifdef Q_OS_ANDROID

    // --------------------------------------------------------
    // Share CSV using Android Sharesheet
    // --------------------------------------------------------

    QNativeInterface::QAndroidApplication::
        runOnAndroidMainThread(
            [filePath]()
            {
                const QJniObject context =
                    QNativeInterface::
                    QAndroidApplication::
                    context();

                if (!context.isValid())
                {
                    qWarning()
                    << "Android context unavailable.";

                    return;
                }


                // Get this app's package name.
                const QJniObject packageName =
                    context.callObjectMethod(
                        "getPackageName",
                        "()Ljava/lang/String;");

                const QString authorityString =
                    packageName.toString() +
                    QStringLiteral(".qtprovider");

                const QJniObject authority =
                    QJniObject::fromString(
                        authorityString);

                const QJniObject javaFilePath =
                    QJniObject::fromString(
                        filePath);

                const QJniObject javaFile(
                    "java/io/File",
                    "(Ljava/lang/String;)V",
                    javaFilePath.object<jstring>());


                // Convert the private file path into a
                // temporary shareable content:// URI.
                const QJniObject fileUri =
                    QJniObject::
                    callStaticObjectMethod(
                        "androidx/core/content/FileProvider",
                        "getUriForFile",
                        "(Landroid/content/Context;"
                        "Ljava/lang/String;"
                        "Ljava/io/File;)"
                        "Landroid/net/Uri;",
                        context.object<jobject>(),
                        authority.object<jstring>(),
                        javaFile.object<jobject>());

                if (!fileUri.isValid())
                {
                    qWarning()
                    << "Could not create shareable CSV URI.";

                    return;
                }


                const QJniObject actionSend =
                    QJniObject::fromString(
                        QStringLiteral(
                            "android.intent.action.SEND"));

                QJniObject sendIntent(
                    "android/content/Intent",
                    "(Ljava/lang/String;)V",
                    actionSend.object<jstring>());


                const QJniObject mimeType =
                    QJniObject::fromString(
                        QStringLiteral("text/csv"));

                sendIntent.callObjectMethod(
                    "setType",
                    "(Ljava/lang/String;)"
                    "Landroid/content/Intent;",
                    mimeType.object<jstring>());


                const QJniObject extraStream =
                    QJniObject::fromString(
                        QStringLiteral(
                            "android.intent.extra.STREAM"));

                sendIntent.callObjectMethod(
                    "putExtra",
                    "(Ljava/lang/String;"
                    "Landroid/os/Parcelable;)"
                    "Landroid/content/Intent;",
                    extraStream.object<jstring>(),
                    fileUri.object<jobject>());


                // Give the selected receiving app temporary
                // read permission for this one CSV file.
                constexpr jint
                    FLAG_GRANT_READ_URI_PERMISSION =
                    0x00000001;

                sendIntent.callObjectMethod(
                    "addFlags",
                    "(I)Landroid/content/Intent;",
                    FLAG_GRANT_READ_URI_PERMISSION);


                const QJniObject chooserTitle =
                    QJniObject::fromString(
                        QStringLiteral(
                            "Share P.I.S.T.O.N. CSV"));

                const QJniObject chooser =
                    QJniObject::
                    callStaticObjectMethod(
                        "android/content/Intent",
                        "createChooser",
                        "(Landroid/content/Intent;"
                        "Ljava/lang/CharSequence;)"
                        "Landroid/content/Intent;",
                        sendIntent.object<jobject>(),
                        chooserTitle.object<jstring>());


                context.callMethod<void>(
                    "startActivity",
                    "(Landroid/content/Intent;)V",
                    chooser.object<jobject>());
            });

    #else

    qDebug()
        << "CSV sharing is currently implemented "
           "for Android only.";

    #endif
}

void PistonBackend::startLogging()
{
    sendBleCommand(QByteArray("start_logging"));
}


void PistonBackend::pauseLogging()
{
    sendBleCommand(QByteArray("pause_logging"));
}

void PistonBackend::readDtcs()
{
    if (!m_connected)
    {
        m_dtcError =
            QStringLiteral(
                "Connect to P.I.S.T.O.N. before scanning DTCs.");

        emit dtcChanged();
        return;
    }

    if (!m_loggingStateKnown)
    {
        m_dtcError =
            QStringLiteral(
                "Waiting for the ESP32 logging state.");

        emit dtcChanged();
        return;
    }

    if (m_loggingEnabled)
    {
        m_dtcError =
            QStringLiteral(
                "Pause logging before scanning DTCs.");

        emit dtcChanged();
        return;
    }

    if (m_dtcScanInProgress)
    {
        return;
    }


    // Clear the previous scan before requesting a new one.
    m_confirmedDtcs.clear();
    m_pendingDtcs.clear();
    m_permanentDtcs.clear();

    m_confirmedDtcResponseReceived = false;
    m_pendingDtcResponseReceived = false;
    m_permanentDtcResponseReceived = false;

    m_dtcError.clear();

    m_dtcResultAvailable = false;
    m_dtcScanInProgress = true;

    emit dtcChanged();


    qDebug()
        << "Requesting P.I.S.T.O.N. DTC scan.";

    sendBleCommand(
        QByteArray("read_dtcs"));
}


void PistonBackend::requestBluetoothAccessAndStartScan()
{
    QBluetoothPermission permission;
    permission.setCommunicationModes(
        QBluetoothPermission::Access);

    QCoreApplication* app =
        QCoreApplication::instance();

    if (app == nullptr)
    {
        setConnectionState(
            QStringLiteral("Application error"));
        return;
    }

    switch (app->checkPermission(permission))
    {
    case Qt::PermissionStatus::Granted:
        startScan();
        return;

    case Qt::PermissionStatus::Denied:
        setConnectionState(
            QStringLiteral("Bluetooth permission denied"));
        return;

    case Qt::PermissionStatus::Undetermined:
        setConnectionState(
            QStringLiteral("Waiting for Bluetooth permission"));

        app->requestPermission(
            permission,
            this,
            [this](const QPermission& result)
            {
                if (result.status() ==
                    Qt::PermissionStatus::Granted)
                {
                    startScan();
                }
                else
                {
                    setConnectionState(
                        QStringLiteral(
                            "Bluetooth permission denied"));
                }
            });

        return;
    }
}

void PistonBackend::startScan()
{
    clearBleObjects();
    m_receiveBuffer.clear();

    setConnected(false);
    setConnectedDeviceName(QString());
    setConnectionState(QStringLiteral("Scanning"));

    qDebug() << "Scanning for P.I.S.T.O.N.";

    m_discoveryAgent->start(
        QBluetoothDeviceDiscoveryAgent::LowEnergyMethod);
}

void PistonBackend::onDeviceDiscovered(
    const QBluetoothDeviceInfo& deviceInfo)
{
    const bool lowEnergyDevice =
        deviceInfo.coreConfigurations().testFlag(
            QBluetoothDeviceInfo::
                LowEnergyCoreConfiguration);

    if (!lowEnergyDevice)
    {
        return;
    }

    const bool nameMatches =
        deviceInfo.name().compare(
            kDeviceName,
            Qt::CaseInsensitive) == 0;

    const bool serviceMatches =
        deviceInfo.serviceUuids().contains(kServiceUuid);

    if (!nameMatches && !serviceMatches)
    {
        return;
    }

    qDebug()
        << "Found P.I.S.T.O.N.:"
        << deviceInfo.name();

    if (m_discoveryAgent->isActive())
    {
        m_discoveryAgent->stop();
    }

    setConnectedDeviceName(
        deviceInfo.name().isEmpty()
            ? kDeviceName
            : deviceInfo.name());

    createControllerForDevice(deviceInfo);
}

void PistonBackend::onScanFinished()
{
    if (m_controller == nullptr)
    {
        setConnectionState(
            QStringLiteral("P.I.S.T.O.N. not found"));
    }
}

void PistonBackend::onScanError(
    QBluetoothDeviceDiscoveryAgent::Error error)
{
    Q_UNUSED(error);

    qWarning()
        << "BLE scan error:"
        << m_discoveryAgent->errorString();

    setConnectionState(
        QStringLiteral("Bluetooth scan failed"));
}

void PistonBackend::createControllerForDevice(
    const QBluetoothDeviceInfo& deviceInfo)
{
    setConnectionState(QStringLiteral("Connecting"));

    m_controller =
        QLowEnergyController::createCentral(
            deviceInfo,
            this);

    if (m_controller == nullptr)
    {
        setConnectionState(
            QStringLiteral("Could not create BLE controller"));
        return;
    }

    connect(
        m_controller,
        &QLowEnergyController::connected,
        this,
        &PistonBackend::onControllerConnected);

    connect(
        m_controller,
        &QLowEnergyController::disconnected,
        this,
        &PistonBackend::onControllerDisconnected);

    connect(
        m_controller,
        &QLowEnergyController::serviceDiscovered,
        this,
        &PistonBackend::onServiceDiscovered);

    connect(
        m_controller,
        &QLowEnergyController::discoveryFinished,
        this,
        &PistonBackend::onServiceDiscoveryFinished);

    connect(
        m_controller,
        &QLowEnergyController::errorOccurred,
        this,
        &PistonBackend::onControllerError);

    qDebug() << "Connecting to P.I.S.T.O.N.";

    m_controller->connectToDevice();
}

void PistonBackend::onControllerConnected()
{
    qDebug() << "BLE link connected.";

    m_serviceFound = false;

    setConnectionState(
        QStringLiteral("Discovering services"));

    m_controller->discoverServices();
}

void PistonBackend::onControllerDisconnected()
{
    qDebug() << "BLE link disconnected.";

    setConnected(false);
    setConnectionState(QStringLiteral("Disconnected"));

    m_loggingStateKnown = false;
    emit loggingStateChanged();
    m_vehicleInfoRequestPending = false;

    if (m_dtcScanInProgress)
    {
        m_dtcScanInProgress = false;

        m_dtcError = QStringLiteral("Bluetooth disconnected during DTC scan.");

        emit dtcChanged();
    }

    m_receiveBuffer.clear();

    m_txCharacteristic =
        QLowEnergyCharacteristic();

    m_rxCharacteristic =
        QLowEnergyCharacteristic();

    m_notifyDescriptor =
        QLowEnergyDescriptor();
}

void PistonBackend::onControllerError(
    QLowEnergyController::Error error)
{
    Q_UNUSED(error);

    if (m_controller != nullptr)
    {
        qWarning()
            << "BLE controller error:"
            << m_controller->errorString();
    }

    setConnected(false);
    setConnectionState(
        QStringLiteral("Bluetooth connection failed"));
}

void PistonBackend::onServiceDiscovered(
    const QBluetoothUuid& serviceUuid)
{
    if (serviceUuid == kServiceUuid)
    {
        qDebug()
            << "P.I.S.T.O.N. service found:"
            << serviceUuid.toString();

        m_serviceFound = true;
    }
}

void PistonBackend::onServiceDiscoveryFinished()
{
    if (!m_serviceFound)
    {
        qWarning() << "P.I.S.T.O.N. service was not found.";

        setConnectionState(
            QStringLiteral("P.I.S.T.O.N. service not found"));

        if (m_controller != nullptr)
        {
            m_controller->disconnectFromDevice();
        }

        return;
    }

    m_service =
        m_controller->createServiceObject(
            kServiceUuid,
            this);

    if (m_service == nullptr)
    {
        setConnectionState(
            QStringLiteral("Could not open P.I.S.T.O.N. service"));
        return;
    }

    connect(
        m_service,
        &QLowEnergyService::stateChanged,
        this,
        &PistonBackend::onServiceStateChanged);

    connect(
        m_service,
        &QLowEnergyService::descriptorWritten,
        this,
        &PistonBackend::onDescriptorWritten);

    connect(
        m_service,
        &QLowEnergyService::characteristicChanged,
        this,
        &PistonBackend::onCharacteristicChanged);

    connect(
        m_service,
        &QLowEnergyService::errorOccurred,
        this,
        &PistonBackend::onServiceError);

    setConnectionState(
        QStringLiteral("Discovering characteristics"));

    m_service->discoverDetails();
}

void PistonBackend::onServiceStateChanged(
    QLowEnergyService::ServiceState state)
{
    if (state !=
        QLowEnergyService::RemoteServiceDiscovered)
    {
        return;
    }

    m_txCharacteristic =
        m_service->characteristic(
            kTxCharacteristicUuid);

    if (!m_txCharacteristic.isValid())
    {
        qWarning()
            << "P.I.S.T.O.N. notify characteristic not found.";

        setConnectionState(
            QStringLiteral("Notify characteristic not found"));
        return;
    }

    if (!m_txCharacteristic.properties().testFlag(
            QLowEnergyCharacteristic::Notify))
    {
        qWarning()
            << "P.I.S.T.O.N. characteristic does not support Notify.";

        setConnectionState(
            QStringLiteral("Characteristic cannot notify"));
        return;
    }

    m_rxCharacteristic =
        m_service->characteristic(
            kRxCharacteristicUuid);

    if (!m_rxCharacteristic.isValid())
    {
        qWarning()
        << "P.I.S.T.O.N. write characteristic not found.";

        setConnectionState(
            QStringLiteral("Write characteristic not found"));
        return;
    }


    if (!m_rxCharacteristic.properties().testFlag(
            QLowEnergyCharacteristic::Write))
    {
        qWarning()
        << "P.I.S.T.O.N. characteristic does not support Write.";

        setConnectionState(
            QStringLiteral("Characteristic cannot write"));
        return;
    }


    qDebug()
        << "P.I.S.T.O.N. write characteristic found.";


    m_notifyDescriptor =
        m_txCharacteristic
            .clientCharacteristicConfiguration();

    if (!m_notifyDescriptor.isValid())
    {
        qWarning()
            << "Client Characteristic Configuration descriptor not found.";

        setConnectionState(
            QStringLiteral("Notification descriptor not found"));
        return;
    }

    setConnectionState(QStringLiteral("Subscribing"));

    m_service->writeDescriptor(
        m_notifyDescriptor,
        QLowEnergyCharacteristic::
            CCCDEnableNotification);
}

void PistonBackend::onDescriptorWritten(
    const QLowEnergyDescriptor& descriptor,
    const QByteArray& value)
{
    if (descriptor != m_notifyDescriptor)
    {
        return;
    }

    if (value !=
        QLowEnergyCharacteristic::
            CCCDEnableNotification)
    {
        return;
    }

    qDebug()
        << "Subscribed to P.I.S.T.O.N. notifications.";

    setConnected(true);
    setConnectionState(QStringLiteral("Connected"));
    m_loggingStateKnown = false;
    emit loggingStateChanged();

    m_vehicleInfoRequestPending = true;

    sendBleCommand(QByteArray("get_logging_status"));
}

void PistonBackend::onCharacteristicChanged(
    const QLowEnergyCharacteristic& characteristic,
    const QByteArray& value)
{
    if (characteristic.uuid() != kTxCharacteristicUuid)
    {
        return;
    }


    m_receiveBuffer.append(value);


    // Protect against a corrupted or never-terminated message
    constexpr qsizetype MAX_RECEIVE_BUFFER_SIZE = 65536;

    if (m_receiveBuffer.size() > MAX_RECEIVE_BUFFER_SIZE)
    {
        qWarning()
        << "P.I.S.T.O.N. receive buffer exceeded limit.";

        m_receiveBuffer.clear();
        return;
    }


    // One ESP32 JSON message ends with '\n'
    while (true)
    {
        const qsizetype newlineIndex =
            m_receiveBuffer.indexOf('\n');

        if (newlineIndex < 0)
        {
            break;
        }


        QByteArray completePayload =
            m_receiveBuffer.left(newlineIndex);

        m_receiveBuffer.remove(
            0,
            newlineIndex + 1);


        if (completePayload.isEmpty())
        {
            continue;
        }


        qDebug()
            << "Complete P.I.S.T.O.N. JSON received."
            << "Bytes:"
            << completePayload.size();


        QJsonParseError parseError;

        const QJsonDocument document =
            QJsonDocument::fromJson(
                completePayload,
                &parseError);


        if (parseError.error !=
            QJsonParseError::NoError)
        {
            qWarning()
            << "P.I.S.T.O.N. JSON parse error:"
            << parseError.errorString();

            continue;
        }


        if (!document.isObject())
        {
            qWarning()
            << "P.I.S.T.O.N. JSON root is not an object.";

            continue;
        }


        const QJsonObject rootObject =
            document.object();


        const QString messageType = rootObject.value("type").toString();

        if (messageType == QStringLiteral("logging_status"))
        {
            const QJsonValue enabledValue =
                rootObject.value("enabled");

            if (!enabledValue.isBool())
            {
                qWarning()
                << "Invalid logging_status message.";

                continue;
            }


            m_loggingEnabled =
                enabledValue.toBool();

            m_loggingStateKnown = true;

            emit loggingStateChanged();


            qDebug()
                << "P.I.S.T.O.N. logging status:"
                << (m_loggingEnabled
                        ? "LOGGING"
                        : "PAUSED");


            if (!m_loggingEnabled &&
                m_vehicleInfoRequestPending)
            {
                m_vehicleInfoRequestPending = false;

                qDebug()
                    << "Requesting P.I.S.T.O.N. VIN.";

                sendBleCommand(
                    QByteArray("read_vehicle_info"));
            }


            continue;

            continue;
        }

        if (messageType == QStringLiteral("dtc_result"))
        {
            m_dtcScanInProgress = false;

            m_confirmedDtcs.clear();
            m_pendingDtcs.clear();
            m_permanentDtcs.clear();


            const QJsonArray confirmedArray =
                rootObject.value("confirmed").toArray();

            const QJsonArray pendingArray =
                rootObject.value("pending").toArray();

            const QJsonArray permanentArray =
                rootObject.value("permanent").toArray();


            for (const QJsonValue& value :
                 confirmedArray)
            {
                if (!value.isString())
                {
                    continue;
                }

                const QString code =
                    value.toString();

                if (!code.isEmpty())
                {
                    m_confirmedDtcs.append(code);
                }
            }


            for (const QJsonValue& value :
                 pendingArray)
            {
                if (!value.isString())
                {
                    continue;
                }

                const QString code =
                    value.toString();

                if (!code.isEmpty())
                {
                    m_pendingDtcs.append(code);
                }
            }


            for (const QJsonValue& value :
                 permanentArray)
            {
                if (!value.isString())
                {
                    continue;
                }

                const QString code =
                    value.toString();

                if (!code.isEmpty())
                {
                    m_permanentDtcs.append(code);
                }
            }


            m_confirmedDtcResponseReceived =
                rootObject
                    .value("confirmed_response")
                    .toBool(false);

            m_pendingDtcResponseReceived =
                rootObject
                    .value("pending_response")
                    .toBool(false);

            m_permanentDtcResponseReceived =
                rootObject
                    .value("permanent_response")
                    .toBool(false);


            const bool success =
                rootObject
                    .value("success")
                    .toBool(false);


            if (success)
            {
                m_dtcError.clear();
            }
            else
            {
                m_dtcError =
                    rootObject
                        .value("error")
                        .toString(
                            QStringLiteral(
                                "DTC scan failed."));
            }


            m_dtcResultAvailable = true;

            emit dtcChanged();


            qDebug()
                << "P.I.S.T.O.N. DTC result received."
                << "Confirmed:"
                << m_confirmedDtcs
                << "Pending:"
                << m_pendingDtcs
                << "Permanent:"
                << m_permanentDtcs
                << "Error:"
                << m_dtcError;

            saveSessionToDisk();

            continue;
        }

        if (messageType == QStringLiteral("vehicle_info"))
        {
            const bool success =
                rootObject.value(QStringLiteral("success")).toBool(false);


            const QString vin =
                rootObject.value(QStringLiteral("vin"))
                    .toString().trimmed().toUpper();


            if (!success ||
                vin.length() != 17)
            {
                const QString error =
                    rootObject.value(
                                  QStringLiteral("error"))
                        .toString(
                            QStringLiteral(
                                "VIN unavailable."));


                qWarning()
                    << "P.I.S.T.O.N. vehicle information:"
                    << error;


                continue;
            }


            const int modelYear =
                decodeVinModelYear(vin);

            const QString make =
                lookupMakeFromVin(
                    vin,
                    modelYear);


            m_vehicleVin =
                vin;

            m_vehicleInfoAvailable =
                true;


            if (modelYear >= 2000 &&
                modelYear <= 2039 &&
                !make.isEmpty())
            {
                m_vehicleDisplayName =
                    QStringLiteral("%1 %2")
                        .arg(modelYear)
                        .arg(make);
            }
            else if (modelYear >= 2000 &&
                     modelYear <= 2039)
            {
                m_vehicleDisplayName =
                    QStringLiteral(
                        "%1 Unknown Make")
                        .arg(modelYear);
            }
            else if (!make.isEmpty())
            {
                m_vehicleDisplayName =
                    make;
            }
            else
            {
                m_vehicleDisplayName =
                    QStringLiteral(
                        "Vehicle identified");
            }


            m_vehicleDetails =
                QStringLiteral("VIN: %1")
                    .arg(m_vehicleVin);


            emit vehicleInfoChanged();
            saveSessionToDisk();


            qDebug()
                << "P.I.S.T.O.N. vehicle identified."
                << "VIN:" << m_vehicleVin
                << "Year:" << modelYear
                << "Make:"
                << (make.isEmpty()
                        ? QStringLiteral("Unknown")
                        : make);


            continue;
        }


        const int protocolVersion =
            rootObject.value("protocol_version").toInt();

        const int caseId =
            rootObject.value("case_id").toInt();

        const int sessionCaseNumber =
            m_caseCount + 1;

        if (messageType !=
            QStringLiteral("case_complete"))
        {
            qWarning()
            << "Unexpected P.I.S.T.O.N. message type:"
            << messageType;

            continue;
        }


        if (protocolVersion != 1)
        {
            qWarning()
            << "Unsupported P.I.S.T.O.N. protocol version:"
            << protocolVersion;

            continue;
        }


        qDebug()
            << "Valid P.I.S.T.O.N. case received."
            << "Case ID:"
            << caseId;

        // --------------------------------------------------------
        // Raw samples
        // --------------------------------------------------------

        const QJsonValue bank2KnownValue =
            rootObject.value("bank2_availability_known");

        const QJsonValue hasBank2Value =
            rootObject.value("has_bank_2");

        if (bank2KnownValue.isBool() && hasBank2Value.isBool())
        {
            m_liveDataModel->setBank2Availability(
                bank2KnownValue.toBool(),
                hasBank2Value.toBool());
        }

        const QJsonArray samples =
            rootObject.value("samples").toArray();

        m_liveDataModel->appendSamples(
            sessionCaseNumber,
            samples);


        // --------------------------------------------------------
        // Diagnostic outputs
        // --------------------------------------------------------

        const QJsonObject outputs =
            rootObject.value("outputs").toObject();


        // ---------------- Fuel ----------------

        const QJsonObject fuel =
            outputs.value("fuel").toObject();

        const QString fuelCondition =
            fuel.value("condition")
                .toString(
                    QStringLiteral("Unavailable"));

        const QString fuelActionLevel =
            fuel.value("action_level")
                .toString(
                    QStringLiteral("Unavailable"));

        m_fuelHistoryModel->prependResult(
            sessionCaseNumber,
            fuelCondition,
            fuelActionLevel,
            fuel.value("evidence_source")
                .toString(QStringLiteral("None")),
            fuel.value("description")
                .toString());

        m_latestFuelCondition =
            fuelCondition;

        m_latestFuelActionLevel =
            fuelActionLevel;


        // ---------------- Catalyst ----------------

        const QJsonObject catalyst =
            outputs.value("catalyst").toObject();

        const QString catalystCondition =
            catalyst.value("condition")
                .toString(
                    QStringLiteral("Unavailable"));

        const QString catalystActionLevel =
            catalyst.value("action_level")
                .toString(
                    QStringLiteral("Unavailable"));

        m_catalystHistoryModel->prependResult(
            sessionCaseNumber,
            catalystCondition,
            catalystActionLevel,
            catalyst.value("evidence_source")
                .toString(QStringLiteral("None")),
            catalyst.value("description")
                .toString());

        m_latestCatalystCondition =
            catalystCondition;

        m_latestCatalystActionLevel =
            catalystActionLevel;


        // ---------------- Charging ----------------

        const QJsonObject charging =
            outputs.value("charging").toObject();

        const QString chargingCondition =
            charging.value("condition")
                .toString(
                    QStringLiteral("Unavailable"));

        const QString chargingActionLevel =
            charging.value("action_level")
                .toString(
                    QStringLiteral("Unavailable"));

        m_chargingHistoryModel->prependResult(
            sessionCaseNumber,
            chargingCondition,
            chargingActionLevel,
            charging.value("evidence_source")
                .toString(QStringLiteral("None")),
            charging.value("description")
                .toString());

        m_latestChargingCondition =
            chargingCondition;

        m_latestChargingActionLevel =
            chargingActionLevel;


        // --------------------------------------------------------
        // Session counts
        // --------------------------------------------------------

        ++m_caseCount;

        m_sampleCount +=
            samples.size();


        emit caseCountChanged();
        emit sampleCountChanged();

        emit latestFuelChanged();
        emit latestCatalystChanged();
        emit latestChargingChanged();

        m_persistedCases.append(rootObject);

        saveSessionToDisk();

        qDebug()
            << "Session updated."
            << "Cases:"
            << m_caseCount
            << "Samples:"
            << m_sampleCount;
    }
}

void PistonBackend::onServiceError(
    QLowEnergyService::ServiceError error)
{
    Q_UNUSED(error);

    qWarning() << "P.I.S.T.O.N. BLE service error.";

    setConnected(false);
    setConnectionState(
        QStringLiteral("BLE service error"));
}

void PistonBackend::setConnected(bool connected)
{
    if (m_connected == connected)
    {
        return;
    }

    m_connected = connected;
    emit connectedChanged();
}

void PistonBackend::setConnectionState(
    const QString& state)
{
    if (m_connectionState == state)
    {
        return;
    }

    m_connectionState = state;

    qDebug()
        << "P.I.S.T.O.N. connection state:"
        << m_connectionState;

    emit connectionStateChanged();
}

void PistonBackend::setConnectedDeviceName(
    const QString& name)
{
    if (m_connectedDeviceName == name)
    {
        return;
    }

    m_connectedDeviceName = name;
    emit connectedDeviceNameChanged();
}

void PistonBackend::sendBleCommand(
    const QByteArray& command)
{
    if (!m_connected)
    {
        qWarning()
        << "Cannot send BLE command: not connected.";
        return;
    }

    if (m_service == nullptr)
    {
        qWarning()
        << "Cannot send BLE command: service unavailable.";
        return;
    }

    if (!m_rxCharacteristic.isValid())
    {
        qWarning()
        << "Cannot send BLE command: write characteristic unavailable.";
        return;
    }

    qDebug()
        << "Sending BLE command:"
        << command;

    m_service->writeCharacteristic(
        m_rxCharacteristic,
        command,
        QLowEnergyService::WriteWithResponse);
}

void PistonBackend::clearBleObjects()
{
    m_serviceFound = false;

    m_txCharacteristic =
        QLowEnergyCharacteristic();

    m_rxCharacteristic =
        QLowEnergyCharacteristic();

    m_notifyDescriptor =
        QLowEnergyDescriptor();

    if (m_service != nullptr)
    {
        m_service->deleteLater();
        m_service = nullptr;
    }

    if (m_controller != nullptr)
    {
        m_controller->disconnect(this);
        m_controller->disconnectFromDevice();
        m_controller->deleteLater();
        m_controller = nullptr;
    }
}
