#pragma once

#include <QObject>
#include <QString>
#include <QStringList>
#include <QAbstractItemModel>

#include <QBluetoothDeviceDiscoveryAgent>
#include <QBluetoothDeviceInfo>
#include <QBluetoothUuid>
#include <QLowEnergyCharacteristic>
#include <QLowEnergyController>
#include <QLowEnergyDescriptor>
#include <QLowEnergyService>

#include <QJsonArray>
#include <QJsonObject>

#include "LiveDataModel.h"
#include "DiagnosticHistoryModel.h"

class PistonBackend : public QObject
{
    Q_OBJECT

    Q_PROPERTY(bool connected READ connected NOTIFY connectedChanged)
    Q_PROPERTY(QString connectionState READ connectionState NOTIFY connectionStateChanged)
    Q_PROPERTY(QString connectedDeviceName READ connectedDeviceName NOTIFY connectedDeviceNameChanged)

    Q_PROPERTY(int caseCount READ caseCount NOTIFY caseCountChanged)
    Q_PROPERTY(int sampleCount READ sampleCount NOTIFY sampleCountChanged)

    Q_PROPERTY(bool vehicleInfoAvailable READ vehicleInfoAvailable NOTIFY vehicleInfoChanged)
    Q_PROPERTY(QString vehicleVin READ vehicleVin NOTIFY vehicleInfoChanged)
    Q_PROPERTY(QString vehicleDisplayName READ vehicleDisplayName NOTIFY vehicleInfoChanged)
    Q_PROPERTY(QString vehicleDetails READ vehicleDetails NOTIFY vehicleInfoChanged)

    Q_PROPERTY(QAbstractItemModel* liveDataModel READ liveDataModel CONSTANT)
    Q_PROPERTY(QAbstractItemModel* fuelHistoryModel READ fuelHistoryModel CONSTANT)
    Q_PROPERTY(QAbstractItemModel* catalystHistoryModel READ catalystHistoryModel CONSTANT)
    Q_PROPERTY(QAbstractItemModel* chargingHistoryModel READ chargingHistoryModel CONSTANT)

    Q_PROPERTY(QString latestFuelCondition READ latestFuelCondition NOTIFY latestFuelChanged)
    Q_PROPERTY(QString latestFuelActionLevel READ latestFuelActionLevel NOTIFY latestFuelChanged)

    Q_PROPERTY(QString latestCatalystCondition READ latestCatalystCondition NOTIFY latestCatalystChanged)
    Q_PROPERTY(QString latestCatalystActionLevel READ latestCatalystActionLevel NOTIFY latestCatalystChanged)

    Q_PROPERTY(QString latestChargingCondition READ latestChargingCondition NOTIFY latestChargingChanged)
    Q_PROPERTY(QString latestChargingActionLevel READ latestChargingActionLevel NOTIFY latestChargingChanged)

    Q_PROPERTY(bool loggingEnabled READ loggingEnabled NOTIFY loggingStateChanged)
    Q_PROPERTY(bool loggingStateKnown READ loggingStateKnown NOTIFY loggingStateChanged)

    Q_PROPERTY(QStringList confirmedDtcs READ confirmedDtcs NOTIFY dtcChanged)
    Q_PROPERTY(QStringList pendingDtcs READ pendingDtcs NOTIFY dtcChanged)
    Q_PROPERTY(QStringList permanentDtcs READ permanentDtcs NOTIFY dtcChanged)

    Q_PROPERTY(bool confirmedDtcResponseReceived
                   READ confirmedDtcResponseReceived
                       NOTIFY dtcChanged)

    Q_PROPERTY(bool pendingDtcResponseReceived
                   READ pendingDtcResponseReceived
                       NOTIFY dtcChanged)

    Q_PROPERTY(bool permanentDtcResponseReceived
                   READ permanentDtcResponseReceived
                       NOTIFY dtcChanged)

    Q_PROPERTY(bool dtcScanInProgress
                   READ dtcScanInProgress
                       NOTIFY dtcChanged)

    Q_PROPERTY(bool dtcResultAvailable
                   READ dtcResultAvailable
                       NOTIFY dtcChanged)

    Q_PROPERTY(QString dtcError
                   READ dtcError
                       NOTIFY dtcChanged)

public:
    explicit PistonBackend(QObject* parent = nullptr);
    ~PistonBackend() override;

    bool connected() const;
    QString connectionState() const;
    QString connectedDeviceName() const;

    int caseCount() const;
    int sampleCount() const;

    bool vehicleInfoAvailable() const;
    QString vehicleVin() const;
    QString vehicleDisplayName() const;
    QString vehicleDetails() const;

    QAbstractItemModel* liveDataModel() const;
    QAbstractItemModel* fuelHistoryModel() const;
    QAbstractItemModel* catalystHistoryModel() const;
    QAbstractItemModel* chargingHistoryModel() const;

    QString latestFuelCondition() const;
    QString latestFuelActionLevel() const;

    QString latestCatalystCondition() const;
    QString latestCatalystActionLevel() const;

    QString latestChargingCondition() const;
    QString latestChargingActionLevel() const;

    bool loggingEnabled() const;
    bool loggingStateKnown() const;

    QStringList confirmedDtcs() const;
    QStringList pendingDtcs() const;
    QStringList permanentDtcs() const;

    bool confirmedDtcResponseReceived() const;
    bool pendingDtcResponseReceived() const;
    bool permanentDtcResponseReceived() const;

    bool dtcScanInProgress() const;
    bool dtcResultAvailable() const;

    QString dtcError() const;

    Q_INVOKABLE void connectToPiston();
    Q_INVOKABLE void disconnectFromPiston();

    Q_INVOKABLE void clearSession();
    Q_INVOKABLE void exportSessionCsv();

    Q_INVOKABLE void startLogging();
    Q_INVOKABLE void pauseLogging();

    Q_INVOKABLE void readDtcs();


signals:
    void connectedChanged();
    void connectionStateChanged();
    void connectedDeviceNameChanged();

    void caseCountChanged();
    void sampleCountChanged();

    void vehicleInfoChanged();

    void latestFuelChanged();
    void latestCatalystChanged();
    void latestChargingChanged();
    void loggingStateChanged();
    void dtcChanged();

private:
    void requestBluetoothAccessAndStartScan();
    void startScan();

    void onDeviceDiscovered(const QBluetoothDeviceInfo& deviceInfo);
    void onScanFinished();
    void onScanError(QBluetoothDeviceDiscoveryAgent::Error error);

    void createControllerForDevice(const QBluetoothDeviceInfo& deviceInfo);

    void onControllerConnected();
    void onControllerDisconnected();
    void onControllerError(QLowEnergyController::Error error);

    void onServiceDiscovered(const QBluetoothUuid& serviceUuid);
    void onServiceDiscoveryFinished();

    void onServiceStateChanged(QLowEnergyService::ServiceState state);
    void onServiceError(QLowEnergyService::ServiceError error);

    void onDescriptorWritten(
        const QLowEnergyDescriptor& descriptor,
        const QByteArray& value);

    void onCharacteristicChanged(
        const QLowEnergyCharacteristic& characteristic,
        const QByteArray& value);

    void setConnected(bool connected);
    void setConnectionState(const QString& state);
    void setConnectedDeviceName(const QString& name);

    void clearBleObjects();

    QString sessionFilePath() const;

    void saveSessionToDisk() const;
    void loadSessionFromDisk();

    void restoreCaseFromJson(const QJsonObject& caseObject, int sessionCaseNumber);

    void sendBleCommand(const QByteArray& command);

    static const QString kDeviceName;
    static const QBluetoothUuid kServiceUuid;
    static const QBluetoothUuid kTxCharacteristicUuid;
    static const QBluetoothUuid kRxCharacteristicUuid;

    QBluetoothDeviceDiscoveryAgent* m_discoveryAgent = nullptr;

    QLowEnergyController* m_controller = nullptr;
    QLowEnergyService* m_service = nullptr;

    QLowEnergyCharacteristic m_txCharacteristic;
    QLowEnergyCharacteristic m_rxCharacteristic;
    QLowEnergyDescriptor m_notifyDescriptor;

    QByteArray m_receiveBuffer;

    bool m_serviceFound = false;
    bool m_connected = false;

    QString m_connectionState = QStringLiteral("Disconnected");
    QString m_connectedDeviceName;

    int m_caseCount = 0;
    int m_sampleCount = 0;
    QJsonArray m_persistedCases;

    bool m_vehicleInfoAvailable = false;

    QString m_vehicleVin;
    QString m_vehicleDisplayName;
    QString m_vehicleDetails;

    bool m_vehicleInfoRequestPending = false;

    QString m_latestFuelCondition;
    QString m_latestFuelActionLevel;

    QString m_latestCatalystCondition;
    QString m_latestCatalystActionLevel;

    QString m_latestChargingCondition;
    QString m_latestChargingActionLevel;

    bool m_loggingEnabled = false;
    bool m_loggingStateKnown = false;
    QStringList m_confirmedDtcs;
    QStringList m_pendingDtcs;
    QStringList m_permanentDtcs;

    bool m_confirmedDtcResponseReceived = false;
    bool m_pendingDtcResponseReceived = false;
    bool m_permanentDtcResponseReceived = false;

    bool m_dtcScanInProgress = false;
    bool m_dtcResultAvailable = false;

    QString m_dtcError;

    LiveDataModel* m_liveDataModel = nullptr;

    DiagnosticHistoryModel* m_fuelHistoryModel = nullptr;
    DiagnosticHistoryModel* m_catalystHistoryModel = nullptr;
    DiagnosticHistoryModel* m_chargingHistoryModel = nullptr;

};
