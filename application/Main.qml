import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

ApplicationWindow {
    id: root

    visible: true
    width: 390
    height: 844
    minimumWidth: 340
    minimumHeight: 640
    title: "P.I.S.T.O.N."
    color: backgroundColor

    // ------------------------------------------------------------
    // Theme
    // ------------------------------------------------------------

    readonly property color backgroundColor: "#0D1117"
    readonly property color surfaceColor: "#151B23"
    readonly property color surfaceRaisedColor: "#1B232D"
    readonly property color borderColor: "#2A3440"
    readonly property color primaryColor: "#2F81F7"
    readonly property color primaryPressedColor: "#1F6FEB"

    readonly property color textPrimaryColor: "#F0F6FC"
    readonly property color textSecondaryColor: "#9DA7B3"
    readonly property color textMutedColor: "#6E7884"

    readonly property color successColor: "#3FB950"
    readonly property color warningColor: "#D29922"
    readonly property color dangerColor: "#F85149"
    readonly property color infoColor: "#58A6FF"

    readonly property int pageMargin: 18
    readonly property int cardRadius: 16

    // ------------------------------------------------------------
    // C++ backend contract
    //
    // Later, main.cpp will expose a context property or singleton named
    // "pistonBackend". Until then, the UI remains usable as a shell and
    // clearly shows that the backend is not attached.
    //
    // Expected properties:
    //   bool connected
    //   string connectionState
    //   string connectedDeviceName
    //   int caseCount
    //   int sampleCount
    //   bool vehicleInfoAvailable
    //   string vehicleDisplayName
    //   string vehicleDetails
    //   var liveDataModel
    //   var fuelHistoryModel
    //   var catalystHistoryModel
    //   var chargingHistoryModel
    //   string latestFuelCondition
    //   string latestFuelActionLevel
    //   string latestCatalystCondition
    //   string latestCatalystActionLevel
    //   string latestChargingCondition
    //   string latestChargingActionLevel
    //   bool loggingEnabled
    //   bool loggingStateKnown
    //   var confirmedDtcs
    //   var pendingDtcs
    //   var permanentDtcs
    //   bool confirmedDtcResponseReceived
    //   bool pendingDtcResponseReceived
    //   bool permanentDtcResponseReceived
    //   bool dtcScanInProgress
    //   bool dtcResultAvailable
    //   string dtcError
    //
    // Expected methods:
    //   connectToPiston()
    //   disconnectFromPiston()
    //   clearSession()
    //   exportSessionCsv() 
    //   startLogging()
    //   pauseLogging()
    //   readDtcs()
    // ------------------------------------------------------------

    property var backend: (typeof pistonBackend !== "undefined") ? pistonBackend : null

    readonly property bool backendAvailable: backend !== null

    readonly property bool bluetoothConnected:
        backendAvailable ? backend.connected : false

    readonly property string connectionState:
        backendAvailable ? backend.connectionState : "BLE backend not attached"

    readonly property string connectedDeviceName:
        backendAvailable && backend.connectedDeviceName.length > 0
        ? backend.connectedDeviceName
        : "P.I.S.T.O.N."

    readonly property int caseCount:
        backendAvailable ? backend.caseCount : 0

    readonly property int sampleCount:
        backendAvailable ? backend.sampleCount : 0

    readonly property bool vehicleInfoAvailable:
        backendAvailable ? backend.vehicleInfoAvailable : false

    readonly property string vehicleDisplayName:
        backendAvailable && backend.vehicleDisplayName.length > 0
        ? backend.vehicleDisplayName
        : "Vehicle identification unavailable"

    readonly property string vehicleDetails:
        backendAvailable && backend.vehicleDetails.length > 0
        ? backend.vehicleDetails
        : "Live OBD-II data can still be collected without a saved vehicle profile."

    readonly property var liveDataModel:
        backendAvailable ? backend.liveDataModel : null

    readonly property var fuelHistoryModel:
        backendAvailable ? backend.fuelHistoryModel : null

    readonly property var catalystHistoryModel:
        backendAvailable ? backend.catalystHistoryModel : null

    readonly property var chargingHistoryModel:
        backendAvailable ? backend.chargingHistoryModel : null

    readonly property string latestFuelCondition:
        backendAvailable && backend.latestFuelCondition.length > 0
        ? backend.latestFuelCondition
        : "No completed case yet"

    readonly property string latestFuelAction:
        backendAvailable && backend.latestFuelActionLevel.length > 0
        ? backend.latestFuelActionLevel
        : "Waiting for data"

    readonly property string latestCatalystCondition:
        backendAvailable && backend.latestCatalystCondition.length > 0
        ? backend.latestCatalystCondition
        : "No completed case yet"

    readonly property string latestCatalystAction:
        backendAvailable && backend.latestCatalystActionLevel.length > 0
        ? backend.latestCatalystActionLevel
        : "Waiting for data"

    readonly property string latestChargingCondition:
        backendAvailable && backend.latestChargingCondition.length > 0
        ? backend.latestChargingCondition
        : "No completed case yet"

    readonly property string latestChargingAction:
        backendAvailable && backend.latestChargingActionLevel.length > 0
        ? backend.latestChargingActionLevel
        : "Waiting for data"

    readonly property bool loggingEnabled:
        backendAvailable ? backend.loggingEnabled : false

    readonly property bool loggingStateKnown:
        backendAvailable ? backend.loggingStateKnown : false

    readonly property var confirmedDtcs:
        backendAvailable ? backend.confirmedDtcs : []

    readonly property var pendingDtcs:
        backendAvailable ? backend.pendingDtcs : []

    readonly property var permanentDtcs:
        backendAvailable ? backend.permanentDtcs : []

    readonly property bool confirmedDtcResponseReceived:
        backendAvailable
        ? backend.confirmedDtcResponseReceived
        : false

    readonly property bool pendingDtcResponseReceived:
        backendAvailable
        ? backend.pendingDtcResponseReceived
        : false

    readonly property bool permanentDtcResponseReceived:
        backendAvailable
        ? backend.permanentDtcResponseReceived
        : false

    readonly property bool dtcScanInProgress:
        backendAvailable
        ? backend.dtcScanInProgress
        : false

    readonly property bool dtcResultAvailable:
        backendAvailable
        ? backend.dtcResultAvailable
        : false

    readonly property string dtcError:
        backendAvailable
        ? backend.dtcError
        : ""

    function actionColor(actionLevel) {
        if (actionLevel === "Immediate Attention")
            return dangerColor

        if (actionLevel === "Inspect Soon")
            return "#F0883E"

        if (actionLevel === "Keep an Eye Out" ||
            actionLevel === "Monitor")
            return warningColor

        if (actionLevel === "Normal Operation" ||
            actionLevel === "Normal")
            return successColor

        return textMutedColor
    }

    function connectionColor() {
        if (!backendAvailable)
            return textMutedColor

        if (bluetoothConnected)
            return successColor

        if (connectionState === "Scanning" ||
            connectionState === "Connecting")
            return warningColor

        return textMutedColor
    }

    function connectOrDisconnect() {
        if (!backendAvailable)
            return

        if (bluetoothConnected)
            backend.disconnectFromPiston()
        else
            backend.connectToPiston()
    }

    function toggleLogging() {
        if (!backendAvailable ||
            !bluetoothConnected ||
            !loggingStateKnown)
            return

        if (loggingEnabled) backend.pauseLogging()
        else backend.startLogging()
    }

    function requestDtcScan() {
        if (!backendAvailable ||
            !bluetoothConnected ||
            !loggingStateKnown ||
            loggingEnabled ||
            dtcScanInProgress)
            return

        backend.readDtcs()
    }

    StackView {
        id: stack
        anchors.fill: parent
        initialItem: homePage
    }

    // ============================================================
    // HOME
    // ============================================================

    Component {
        id: homePage

        Page {
            background: Rectangle {
                color: root.backgroundColor
            }

            ScrollView {
                anchors.fill: parent
                contentWidth: availableWidth
                clip: true

                ColumnLayout {
                    width: parent.width
                    spacing: 16

                    Item {
                        Layout.fillWidth: true
                        Layout.preferredHeight: 10
                    }

                    // Logo / application identity
                    Rectangle {
                        Layout.fillWidth: true
                        Layout.leftMargin: root.pageMargin
                        Layout.rightMargin: root.pageMargin
                        Layout.preferredHeight: 112

                        radius: root.cardRadius
                        color: root.surfaceColor
                        border.color: root.borderColor
                        border.width: 1
                        clip: true

                        Image {
                            id: pistonLogo
                            anchors.fill: parent
                            anchors.margins: 6
                            source: "images/logo.png"
                            fillMode: Image.PreserveAspectFit
                            smooth: true
                            mipmap: true
                            sourceClipRect: Qt.rect(90, 180, 850, 540)

                            visible: status === Image.Ready
                        }

                        Column {
                            anchors.centerIn: parent
                            spacing: 4
                            visible: pistonLogo.status !== Image.Ready

                            Text {
                                anchors.horizontalCenter: parent.horizontalCenter
                                text: "P.I.S.T.O.N."
                                color: root.textPrimaryColor
                                font.pixelSize: 28
                                font.bold: true
                            }

                            Text {
                                anchors.horizontalCenter: parent.horizontalCenter
                                text: "Vehicle Diagnostics"
                                color: root.textSecondaryColor
                                font.pixelSize: 13
                            }
                        }
                    }

                    Text {
                        Layout.fillWidth: true
                        Layout.leftMargin: root.pageMargin + 4
                        Layout.rightMargin: root.pageMargin + 4

                        text: "Predictive Intelligence System for Troubleshooting Onboard Networks"
                        color: root.textSecondaryColor
                        font.pixelSize: 12
                        wrapMode: Text.WordWrap
                        horizontalAlignment: Text.AlignHCenter
                    }

                    // BLE connection
                    SectionCard {
                        Layout.leftMargin: root.pageMargin
                        Layout.rightMargin: root.pageMargin
                        Layout.preferredHeight: connectionCardContent.implicitHeight + 32

                        ColumnLayout {
                            id: connectionCardContent
                            anchors.fill: parent
                            anchors.margins: 16
                            spacing: 12

                            RowLayout {
                                Layout.fillWidth: true

                                ColumnLayout {
                                    Layout.fillWidth: true
                                    spacing: 3

                                    Text {
                                        text: "ESP32 Connection"
                                        color: root.textSecondaryColor
                                        font.pixelSize: 13
                                    }

                                    Text {
                                        text: root.bluetoothConnected
                                              ? root.connectedDeviceName
                                              : "Not connected"
                                        color: root.textPrimaryColor
                                        font.pixelSize: 21
                                        font.bold: true
                                        elide: Text.ElideRight
                                        Layout.fillWidth: true
                                    }
                                }

                                StatusPill {
                                    text: root.connectionState
                                    statusColor: root.connectionColor()
                                }
                            }

                            Rectangle {
                                Layout.fillWidth: true
                                height: 1
                                color: root.borderColor
                            }

                            Text {
                                Layout.fillWidth: true

                                text: !root.backendAvailable
                                      ? "The QML interface is ready. The C++ Bluetooth backend still needs to be attached."
                                      : !root.bluetoothConnected
                                        ? "Connect to the P.I.S.T.O.N. ESP32 to begin a diagnostic session."
                                        : !root.loggingStateKnown
                                          ? "Connected. Checking the ESP32 logging state..."
                                          : root.loggingEnabled
                                            ? "Logging is active. P.I.S.T.O.N. is collecting OBD-II samples."
                                            : "Connected. Logging is paused. Start logging when you are ready."
                                color: root.textSecondaryColor
                                font.pixelSize: 13
                                wrapMode: Text.WordWrap
                            }

                            PrimaryButton {
                                Layout.fillWidth: true

                                text: root.bluetoothConnected
                                      ? "Disconnect"
                                      : "Connect to P.I.S.T.O.N."

                                enabled: root.backendAvailable

                                onClicked: root.connectOrDisconnect()
                            }

                            SecondaryButton {
                                Layout.fillWidth: true

                                text: !root.bluetoothConnected
                                      ? "Start Logging"
                                      : !root.loggingStateKnown
                                        ? "Checking Logging Status..."
                                        : root.loggingEnabled
                                          ? "Pause Logging"
                                          : "Start Logging"

                                enabled: root.backendAvailable &&
                                         root.bluetoothConnected &&
                                         root.loggingStateKnown

                                onClicked: root.toggleLogging()
                            }

                        }
                    }

                    // Session summary
                    SectionCard {
                        Layout.leftMargin: root.pageMargin
                        Layout.rightMargin: root.pageMargin
                        Layout.preferredHeight: sessionCardContent.implicitHeight + 32

                        ColumnLayout {
                            id: sessionCardContent
                            anchors.fill: parent
                            anchors.margins: 16
                            spacing: 12

                            RowLayout {
                                Layout.fillWidth: true

                                Text {
                                    text: "Current Session"
                                    color: root.textPrimaryColor
                                    font.pixelSize: 18
                                    font.bold: true
                                    Layout.fillWidth: true
                                }

                                Text {
                                    text: root.caseCount > 0 ? "ACTIVE" : "EMPTY"
                                    color: root.caseCount > 0
                                           ? root.successColor
                                           : root.textMutedColor
                                    font.pixelSize: 11
                                    font.bold: true
                                }
                            }

                            RowLayout {
                                Layout.fillWidth: true
                                spacing: 10

                                MetricTile {
                                    Layout.fillWidth: true
                                    title: "Cases"
                                    value: root.caseCount.toString()
                                }

                                MetricTile {
                                    Layout.fillWidth: true
                                    title: "Samples"
                                    value: root.sampleCount.toString()
                                }
                            }

                            Text {
                                Layout.fillWidth: true
                                text: root.vehicleDisplayName
                                color: root.textPrimaryColor
                                font.pixelSize: 15
                                font.bold: true
                                wrapMode: Text.WordWrap
                            }

                            Text {
                                Layout.fillWidth: true
                                text: root.vehicleDetails
                                color: root.textSecondaryColor
                                font.pixelSize: 12
                                wrapMode: Text.WordWrap
                            }
                        }
                    }

                    PrimaryButton {
                        Layout.fillWidth: true
                        Layout.leftMargin: root.pageMargin
                        Layout.rightMargin: root.pageMargin

                        text: "Open Vehicle Session"
                        onClicked: stack.push(sessionHubPage)
                    }

                    TextButton {
                        Layout.alignment: Qt.AlignHCenter
                        text: "Clear Current Session"
                        enabled: root.backendAvailable &&
                                 (root.sampleCount > 0 ||
                                  root.dtcResultAvailable || root.vehicleInfoAvailable)
                        onClicked: {
                            if (root.backendAvailable)
                                root.backend.clearSession()
                        }
                    }

                    Item {
                        Layout.fillWidth: true
                        Layout.preferredHeight: 18
                    }
                }
            }
        }
    }

    // ============================================================
    // VEHICLE SESSION HUB
    // ============================================================

    Component {
        id: sessionHubPage

        Page {
            background: Rectangle {
                color: root.backgroundColor
            }

            ColumnLayout {
                anchors.fill: parent
                spacing: 0

                PageHeader {
                    title: "Vehicle Session"
                    subtitle: root.bluetoothConnected
                              ? "Connected to " + root.connectedDeviceName
                              : "Bluetooth disconnected"

                    onBackRequested: stack.pop()
                }

                ScrollView {
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    contentWidth: availableWidth
                    clip: true

                    ColumnLayout {
                        width: parent.width
                        spacing: 14

                        Item {
                            Layout.preferredHeight: 4
                        }

                        SectionCard {
                            Layout.leftMargin: root.pageMargin
                            Layout.rightMargin: root.pageMargin
                            Layout.preferredHeight: vehicleCardContent.implicitHeight + 32

                            ColumnLayout {
                                id: vehicleCardContent
                                anchors.fill: parent
                                anchors.margins: 16
                                spacing: 6

                                Text {
                                    text: "Vehicle"
                                    color: root.textSecondaryColor
                                    font.pixelSize: 12
                                }

                                Text {
                                    Layout.fillWidth: true
                                    text: root.vehicleDisplayName
                                    color: root.textPrimaryColor
                                    font.pixelSize: 20
                                    font.bold: true
                                    wrapMode: Text.WordWrap
                                }

                                Text {
                                    Layout.fillWidth: true
                                    text: root.vehicleDetails
                                    color: root.textSecondaryColor
                                    font.pixelSize: 12
                                    wrapMode: Text.WordWrap
                                }

                                RowLayout {
                                    Layout.fillWidth: true

                                    Text {
                                        text: root.caseCount + " cases"
                                        color: root.textSecondaryColor
                                        font.pixelSize: 12
                                    }

                                    Item {
                                        Layout.fillWidth: true
                                    }

                                    Text {
                                        text: root.sampleCount + " samples"
                                        color: root.textSecondaryColor
                                        font.pixelSize: 12
                                    }
                                }
                            }
                        }

                        NavigationRow {
                            Layout.leftMargin: root.pageMargin
                            Layout.rightMargin: root.pageMargin

                            title: "Live Data"
                            subtitle: "All raw OBD-II samples received during this session"
                            badgeText: root.sampleCount.toString()

                            onClicked: stack.push(liveDataPage)
                        }

                        NavigationRow {
                            Layout.leftMargin: root.pageMargin
                            Layout.rightMargin: root.pageMargin

                            title: "P.I.S.T.O.N. Output"
                            subtitle: "Fuel, catalyst, and charging diagnostic history"
                            badgeText: root.caseCount.toString()

                            onClicked: stack.push(predictiveAnalysisPage)
                        }

                        NavigationRow {
                            Layout.leftMargin: root.pageMargin
                            Layout.rightMargin: root.pageMargin

                            title: "DTC Check"
                            subtitle: "Confirmed, pending, and permanent diagnostic trouble codes"
                            badgeText: ""

                            onClicked: stack.push(dtcPage)
                        }

                        Item {
                            Layout.fillWidth: true
                            Layout.preferredHeight: 18
                        }
                    }
                }
            }
        }
    }

    // ============================================================
    // LIVE DATA
    // ============================================================

    Component {
        id: liveDataPage

        Page {
            background: Rectangle {
                color: root.backgroundColor
            }

            ColumnLayout {
                anchors.fill: parent
                spacing: 0

                PageHeader {
                    title: "Live Data"
                    subtitle: root.sampleCount + " samples received this session"

                    onBackRequested: stack.pop()
                }

                RowLayout {
                    Layout.fillWidth: true
                    Layout.leftMargin: root.pageMargin
                    Layout.rightMargin: root.pageMargin
                    Layout.topMargin: 12
                    Layout.bottomMargin: 10
                    spacing: 8


                    SecondaryButton {
                        Layout.fillWidth: true
                        text: "Export CSV"
                        enabled: root.backendAvailable && root.sampleCount > 0

                        onClicked: {
                            if (root.backendAvailable)
                                root.backend.exportSessionCsv()
                        }
                    }
                }

                Rectangle {
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    Layout.leftMargin: root.pageMargin
                    Layout.rightMargin: root.pageMargin
                    Layout.bottomMargin: root.pageMargin

                    radius: 12
                    color: root.surfaceColor
                    border.color: root.borderColor
                    border.width: 1
                    clip: true

                    ColumnLayout {
                        anchors.fill: parent
                        spacing: 0

                        HorizontalHeaderView {
                            id: liveHeader
                            Layout.fillWidth: true
                            Layout.preferredHeight: 42

                            syncView: liveTable
                            clip: true

                            delegate: Rectangle {
                                required property var display

                                implicitWidth: 128
                                implicitHeight: 42
                                color: root.surfaceRaisedColor
                                border.color: root.borderColor
                                border.width: 1

                                Text {
                                    anchors.fill: parent
                                    anchors.margins: 8

                                    text: parent.display === undefined
                                          ? ""
                                          : parent.display

                                    color: root.textPrimaryColor
                                    font.pixelSize: 11
                                    font.bold: true
                                    verticalAlignment: Text.AlignVCenter
                                    horizontalAlignment: Text.AlignHCenter
                                    elide: Text.ElideRight
                                }
                            }
                        }

                        TableView {
                            id: liveTable
                            Layout.fillWidth: true
                            Layout.fillHeight: true

                            clip: true
                            model: root.liveDataModel

                            columnSpacing: 1
                            rowSpacing: 1

                            columnWidthProvider: function(column) {
                                if (column === 0)
                                    return 84

                                if (column === 1)
                                    return 110

                                return 128
                            }

                            rowHeightProvider: function(row) {
                                return 40
                            }

                            delegate: Rectangle {
                                required property var display
                                required property int row
                                required property int column

                                color: row % 2 === 0
                                       ? root.surfaceColor
                                       : root.surfaceRaisedColor

                                border.color: root.borderColor
                                border.width: 1

                                Text {
                                    anchors.fill: parent
                                    anchors.margins: 7

                                    text: parent.display === undefined ||
                                          parent.display === null
                                          ? "—"
                                          : parent.display

                                    color: root.textPrimaryColor
                                    font.pixelSize: 11
                                    verticalAlignment: Text.AlignVCenter
                                    horizontalAlignment: Text.AlignHCenter
                                    elide: Text.ElideRight
                                }
                            }
                        }
                    }

                    EmptyState {
                        anchors.centerIn: parent
                        width: Math.min(parent.width - 32, 320)

                        visible: root.sampleCount === 0

                        title: "No live data yet"
                        message: root.bluetoothConnected
                                 ? "Waiting for the ESP32 to finish and transmit the first 20-sample case."
                                 : "Connect to the P.I.S.T.O.N. ESP32 to begin receiving vehicle data."
                    }
                }
            }
        }
    }

    // ============================================================
    // PREDICTIVE ANALYSIS
    // ============================================================

    Component {
        id: predictiveAnalysisPage

        Page {
            background: Rectangle {
                color: root.backgroundColor
            }

            ColumnLayout {
                anchors.fill: parent
                spacing: 0

                PageHeader {
                    title: "P.I.S.T.O.N. Output"
                    subtitle: root.caseCount + " completed diagnostic cases"

                    onBackRequested: stack.pop()
                }

                ScrollView {
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    contentWidth: availableWidth
                    clip: true

                    ColumnLayout {
                        width: parent.width
                        spacing: 14

                        Item {
                            Layout.preferredHeight: 4
                        }

                        SubsystemSummaryCard {
                            Layout.leftMargin: root.pageMargin
                            Layout.rightMargin: root.pageMargin

                            title: "Fuel Delivery"
                            condition: root.latestFuelCondition
                            actionLevel: root.latestFuelAction

                            onClicked: {
                                stack.push(
                                    diagnosticHistoryPage,
                                    {
                                        pageTitle: "Fuel Delivery",
                                        historyModel: root.fuelHistoryModel
                                    }
                                )
                            }
                        }

                        SubsystemSummaryCard {
                            Layout.leftMargin: root.pageMargin
                            Layout.rightMargin: root.pageMargin

                            title: "Catalyst"
                            condition: root.latestCatalystCondition
                            actionLevel: root.latestCatalystAction

                            onClicked: {
                                stack.push(
                                    diagnosticHistoryPage,
                                    {
                                        pageTitle: "Catalyst",
                                        historyModel: root.catalystHistoryModel
                                    }
                                )
                            }
                        }

                        SubsystemSummaryCard {
                            Layout.leftMargin: root.pageMargin
                            Layout.rightMargin: root.pageMargin

                            title: "Charging System"
                            condition: root.latestChargingCondition
                            actionLevel: root.latestChargingAction

                            onClicked: {
                                stack.push(
                                    diagnosticHistoryPage,
                                    {
                                        pageTitle: "Charging System",
                                        historyModel: root.chargingHistoryModel
                                    }
                                )
                            }
                        }

                        Text {
                            Layout.fillWidth: true
                            Layout.leftMargin: root.pageMargin + 4
                            Layout.rightMargin: root.pageMargin + 4
                            Layout.topMargin: 4

                            text: "Each subsystem keeps every result received during the current session. The newest result appears at the top of its history."
                            color: root.textMutedColor
                            font.pixelSize: 11
                            wrapMode: Text.WordWrap
                        }

                        Item {
                            Layout.preferredHeight: 18
                        }
                    }
                }
            }
        }
    }

    // ============================================================
    // SUBSYSTEM HISTORY
    // ============================================================

    Component {
        id: diagnosticHistoryPage

        Page {
            id: historyPage

            property string pageTitle: "Subsystem"
            property var historyModel: null

            background: Rectangle {
                color: root.backgroundColor
            }

            ColumnLayout {
                anchors.fill: parent
                spacing: 0

                PageHeader {
                    title: historyPage.pageTitle
                    subtitle: root.caseCount + " completed cases in session"

                    onBackRequested: stack.pop()
                }

                ListView {
                    id: historyList

                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    Layout.leftMargin: root.pageMargin
                    Layout.rightMargin: root.pageMargin
                    Layout.topMargin: 10
                    Layout.bottomMargin: root.pageMargin

                    spacing: 12
                    clip: true
                    model: historyPage.historyModel

                    delegate: Rectangle {
                        id: historyCard

                        required property int caseId
                        required property string condition
                        required property string actionLevel
                        required property string evidenceSource
                        required property string description

                        width: historyList.width
                        height: historyContent.implicitHeight + 28

                        radius: root.cardRadius
                        color: root.surfaceColor
                        border.color: root.borderColor
                        border.width: 1

                        ColumnLayout {
                            id: historyContent

                            anchors.left: parent.left
                            anchors.right: parent.right
                            anchors.top: parent.top
                            anchors.margins: 14
                            spacing: 10

                            RowLayout {
                                Layout.fillWidth: true

                                Text {
                                    text: "Case " + historyCard.caseId
                                    color: root.textPrimaryColor
                                    font.pixelSize: 17
                                    font.bold: true
                                    Layout.fillWidth: true
                                }

                                StatusPill {
                                    text: historyCard.actionLevel
                                    statusColor: root.actionColor(historyCard.actionLevel)
                                }
                            }

                            Text {
                                Layout.fillWidth: true
                                text: historyCard.condition
                                color: root.textPrimaryColor
                                font.pixelSize: 15
                                font.bold: true
                                wrapMode: Text.WordWrap
                            }

                            RowLayout {
                                Layout.fillWidth: true
                                spacing: 6

                                Text {
                                    text: "Evidence:"
                                    color: root.textMutedColor
                                    font.pixelSize: 11
                                }

                                Text {
                                    text: historyCard.evidenceSource
                                    color: root.infoColor
                                    font.pixelSize: 11
                                    font.bold: true
                                    Layout.fillWidth: true
                                }
                            }

                            Rectangle {
                                Layout.fillWidth: true
                                height: 1
                                color: root.borderColor
                            }

                            Text {
                                Layout.fillWidth: true
                                text: historyCard.description
                                color: root.textSecondaryColor
                                font.pixelSize: 13
                                wrapMode: Text.WordWrap
                                lineHeight: 1.15
                            }
                        }
                    }

                    ScrollBar.vertical: ScrollBar {
                        policy: ScrollBar.AsNeeded
                    }
                }

                EmptyState {
                    anchors.centerIn: parent
                    width: Math.min(parent.width - 40, 320)

                    visible: historyList.count === 0

                    title: "No results yet"
                    message: "This subsystem will populate after the ESP32 transmits the first completed diagnostic case."
                }
            }
        }
    }

    // ============================================================
    // DTC CHECK
    // ============================================================

    Component {
        id: dtcPage

        Page {
            background: Rectangle {
                color: root.backgroundColor
            }

            ColumnLayout {
                anchors.fill: parent
                spacing: 0

                PageHeader {
                    title: "DTC Check"

                    subtitle: !root.bluetoothConnected
                              ? "Bluetooth disconnected"
                              : root.dtcScanInProgress
                                ? "Scanning vehicle..."
                                : root.dtcResultAvailable
                                  ? "Latest vehicle scan"
                                  : "Confirmed, pending, and permanent codes"

                    onBackRequested: stack.pop()
                }

                ScrollView {
                    Layout.fillWidth: true
                    Layout.fillHeight: true

                    contentWidth: availableWidth
                    clip: true

                    ColumnLayout {
                        width: parent.width
                        spacing: 14

                        Item {
                            Layout.preferredHeight: 4
                        }

                        // ------------------------------------------------
                        // Scan controls
                        // ------------------------------------------------

                        SectionCard {
                            Layout.leftMargin: root.pageMargin
                            Layout.rightMargin: root.pageMargin
                            Layout.preferredHeight:
                                dtcScanCardContent.implicitHeight + 32

                            ColumnLayout {
                                id: dtcScanCardContent

                                anchors.fill: parent
                                anchors.margins: 16
                                spacing: 12

                                Text {
                                    Layout.fillWidth: true

                                    text: "Vehicle DTC Scan"
                                    color: root.textPrimaryColor

                                    font.pixelSize: 18
                                    font.bold: true
                                }

                                Text {
                                    Layout.fillWidth: true

                                    text: !root.bluetoothConnected
                                          ? "Connect to the P.I.S.T.O.N. ESP32 before scanning."
                                          : !root.loggingStateKnown
                                            ? "Waiting for the ESP32 logging state."
                                            : root.loggingEnabled
                                              ? "Pause logging before scanning for diagnostic trouble codes."
                                              : root.dtcScanInProgress
                                                ? "Reading confirmed, pending, and permanent trouble codes from the vehicle."
                                                : root.dtcResultAvailable
                                                  ? "Scan complete. You can run another scan at any time while logging is paused."
                                                  : "Ready to scan confirmed, pending, and permanent diagnostic trouble codes."

                                    color: root.textSecondaryColor
                                    font.pixelSize: 13
                                    wrapMode: Text.WordWrap
                                }

                                PrimaryButton {
                                    Layout.fillWidth: true

                                    text: !root.bluetoothConnected
                                          ? "Scan DTCs"
                                          : root.loggingEnabled
                                            ? "Pause Logging to Scan"
                                            : root.dtcScanInProgress
                                              ? "Scanning..."
                                              : "Scan DTCs"

                                    enabled:
                                        root.backendAvailable &&
                                        root.bluetoothConnected &&
                                        root.loggingStateKnown &&
                                        !root.loggingEnabled &&
                                        !root.dtcScanInProgress

                                    onClicked:
                                        root.requestDtcScan()
                                }
                            }
                        }

                        // ------------------------------------------------
                        // Error
                        // ------------------------------------------------

                        Text {
                            Layout.fillWidth: true
                            Layout.leftMargin: root.pageMargin + 4
                            Layout.rightMargin: root.pageMargin + 4

                            visible: root.dtcError.length > 0

                            text: root.dtcError
                            color: root.dangerColor

                            font.pixelSize: 12
                            font.bold: true

                            wrapMode: Text.WordWrap
                        }

                        // ------------------------------------------------
                        // Results
                        // ------------------------------------------------

                        DtcCategoryCard {
                            Layout.leftMargin: root.pageMargin
                            Layout.rightMargin: root.pageMargin

                            visible: root.dtcResultAvailable

                            title: "Confirmed DTCs"
                            subtitle:
                                "Stored trouble codes reported by OBD-II Mode 03."

                            codes: root.confirmedDtcs

                            responseReceived:
                                root.confirmedDtcResponseReceived
                        }

                        DtcCategoryCard {
                            Layout.leftMargin: root.pageMargin
                            Layout.rightMargin: root.pageMargin

                            visible: root.dtcResultAvailable

                            title: "Pending DTCs"
                            subtitle:
                                "Pending trouble codes reported by OBD-II Mode 07."

                            codes: root.pendingDtcs

                            responseReceived:
                                root.pendingDtcResponseReceived
                        }

                        DtcCategoryCard {
                            Layout.leftMargin: root.pageMargin
                            Layout.rightMargin: root.pageMargin

                            visible: root.dtcResultAvailable

                            title: "Permanent DTCs"
                            subtitle:
                                "Permanent trouble codes reported by OBD-II Mode 0A."

                            codes: root.permanentDtcs

                            responseReceived:
                                root.permanentDtcResponseReceived
                        }

                        EmptyState {
                            Layout.fillWidth: true
                            Layout.leftMargin: root.pageMargin
                            Layout.rightMargin: root.pageMargin
                            Layout.topMargin: 40

                            visible:
                                !root.dtcResultAvailable &&
                                !root.dtcScanInProgress

                            title: "No DTC scan yet"

                            message:
                                root.bluetoothConnected
                                ? "Run a DTC scan while logging is paused."
                                : "Connect to the P.I.S.T.O.N. ESP32 to scan the vehicle."
                        }

                        Item {
                            Layout.fillWidth: true
                            Layout.preferredHeight: 18
                        }
                    }
                }
            }
        }
    }


    // ============================================================
    // REUSABLE COMPONENTS
    // ============================================================

    component SectionCard: Rectangle {
        Layout.fillWidth: true
        implicitHeight: 100

        radius: root.cardRadius
        color: root.surfaceColor
        border.color: root.borderColor
        border.width: 1
    }

    component StatusPill: Rectangle {
        property string text: ""
        property color statusColor: root.textMutedColor

        implicitWidth: pillText.implicitWidth + 20
        implicitHeight: 28

        radius: 14
        color: Qt.rgba(statusColor.r, statusColor.g, statusColor.b, 0.14)
        border.color: statusColor
        border.width: 1

        Text {
            id: pillText
            anchors.centerIn: parent

            text: parent.text
            color: parent.statusColor
            font.pixelSize: 10
            font.bold: true
        }
    }

    component MetricTile: Rectangle {
        property string title: ""
        property string value: ""

        implicitHeight: 70
        radius: 12
        color: root.surfaceRaisedColor
        border.color: root.borderColor
        border.width: 1

        Column {
            anchors.centerIn: parent
            spacing: 3

            Text {
                anchors.horizontalCenter: parent.horizontalCenter
                text: value
                color: root.textPrimaryColor
                font.pixelSize: 22
                font.bold: true
            }

            Text {
                anchors.horizontalCenter: parent.horizontalCenter
                text: title
                color: root.textSecondaryColor
                font.pixelSize: 11
            }
        }
    }

    component PrimaryButton: Button {
        id: primaryButton

        implicitHeight: 52

        background: Rectangle {
            radius: 13

            color: !primaryButton.enabled
                   ? "#26303A"
                   : primaryButton.down
                     ? root.primaryPressedColor
                     : root.primaryColor

            border.color: !primaryButton.enabled
                          ? root.borderColor
                          : root.primaryColor

            border.width: 1
        }

        contentItem: Text {
            text: primaryButton.text

            color: primaryButton.enabled
                   ? "white"
                   : root.textMutedColor

            font.pixelSize: 14
            font.bold: true

            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignVCenter
            elide: Text.ElideRight
        }
    }

    component SecondaryButton: Button {
        id: secondaryButton

        implicitHeight: 44

        background: Rectangle {
            radius: 11

            color: secondaryButton.down
                   ? root.surfaceRaisedColor
                   : root.surfaceColor

            border.color: secondaryButton.enabled
                          ? root.borderColor
                          : "#202832"

            border.width: 1
        }

        contentItem: Text {
            text: secondaryButton.text

            color: secondaryButton.enabled
                   ? root.textPrimaryColor
                   : root.textMutedColor

            font.pixelSize: 12
            font.bold: true

            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignVCenter
            elide: Text.ElideRight
        }
    }

    component TextButton: Button {
        id: textButton

        flat: true
        implicitHeight: 38

        background: Rectangle {
            color: "transparent"
        }

        contentItem: Text {
            text: textButton.text
            color: textButton.enabled
                   ? root.textSecondaryColor
                   : root.textMutedColor

            font.pixelSize: 12
            font.bold: true

            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignVCenter
        }
    }

    component PageHeader: Rectangle {
        id: pageHeader

        property string title: ""
        property string subtitle: ""

        signal backRequested()

        Layout.fillWidth: true
        Layout.preferredHeight: 82

        color: root.surfaceColor

        Rectangle {
            anchors.left: parent.left
            anchors.right: parent.right
            anchors.bottom: parent.bottom
            height: 1
            color: root.borderColor
        }

        RowLayout {
            anchors.fill: parent
            anchors.leftMargin: 10
            anchors.rightMargin: 14
            spacing: 8

            Button {
                Layout.preferredWidth: 48
                Layout.preferredHeight: 48

                background: Rectangle {
                    radius: 12
                    color: parent.down
                           ? root.surfaceRaisedColor
                           : "transparent"
                }

                contentItem: Text {
                    text: "‹"
                    color: root.textPrimaryColor
                    font.pixelSize: 34
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                }

                onClicked: pageHeader.backRequested()
            }

            ColumnLayout {
                Layout.fillWidth: true
                spacing: 1

                Text {
                    Layout.fillWidth: true
                    text: pageHeader.title
                    color: root.textPrimaryColor
                    font.pixelSize: 22
                    font.bold: true
                    elide: Text.ElideRight
                }

                Text {
                    Layout.fillWidth: true
                    text: pageHeader.subtitle
                    color: root.textSecondaryColor
                    font.pixelSize: 11
                    elide: Text.ElideRight
                }
            }
        }
    }

    component NavigationRow: Rectangle {
        id: navigationRow

        property string title: ""
        property string subtitle: ""
        property string badgeText: ""

        signal clicked()

        Layout.fillWidth: true
        implicitHeight: 86

        radius: root.cardRadius
        color: mouseArea.pressed
               ? root.surfaceRaisedColor
               : root.surfaceColor

        border.color: root.borderColor
        border.width: 1

        RowLayout {
            anchors.fill: parent
            anchors.margins: 15
            spacing: 12

            ColumnLayout {
                Layout.fillWidth: true
                spacing: 4

                Text {
                    Layout.fillWidth: true
                    text: navigationRow.title
                    color: root.textPrimaryColor
                    font.pixelSize: 17
                    font.bold: true
                    elide: Text.ElideRight
                }

                Text {
                    Layout.fillWidth: true
                    text: navigationRow.subtitle
                    color: root.textSecondaryColor
                    font.pixelSize: 12
                    wrapMode: Text.WordWrap
                    maximumLineCount: 2
                    elide: Text.ElideRight
                }
            }

            Rectangle {
                visible: navigationRow.badgeText.length > 0

                implicitWidth: badgeValue.implicitWidth + 18
                implicitHeight: 28

                radius: 14
                color: root.surfaceRaisedColor
                border.color: root.borderColor
                border.width: 1

                Text {
                    id: badgeValue
                    anchors.centerIn: parent
                    text: navigationRow.badgeText
                    color: root.textSecondaryColor
                    font.pixelSize: 11
                    font.bold: true
                }
            }

            Text {
                text: "›"
                color: root.textMutedColor
                font.pixelSize: 30
            }
        }

        MouseArea {
            id: mouseArea
            anchors.fill: parent
            onClicked: navigationRow.clicked()
        }
    }

    component SubsystemSummaryCard: Rectangle {
        id: subsystemCard

        property string title: ""
        property string condition: ""
        property string actionLevel: ""

        signal clicked()

        Layout.fillWidth: true
        implicitHeight: 122

        radius: root.cardRadius
        color: subsystemMouseArea.pressed
               ? root.surfaceRaisedColor
               : root.surfaceColor

        border.color: root.borderColor
        border.width: 1

        ColumnLayout {
            anchors.fill: parent
            anchors.margins: 15
            spacing: 8

            RowLayout {
                Layout.fillWidth: true

                Text {
                    text: subsystemCard.title
                    color: root.textPrimaryColor
                    font.pixelSize: 18
                    font.bold: true
                    Layout.fillWidth: true
                }

                StatusPill {
                    text: subsystemCard.actionLevel
                    statusColor: root.actionColor(subsystemCard.actionLevel)
                }
            }

            Text {
                Layout.fillWidth: true
                text: subsystemCard.condition
                color: root.textSecondaryColor
                font.pixelSize: 14
                wrapMode: Text.WordWrap
                maximumLineCount: 2
                elide: Text.ElideRight
            }

            RowLayout {
                Layout.fillWidth: true

                Text {
                    text: root.caseCount > 0
                          ? "View session history"
                          : "Waiting for first completed case"

                    color: root.textMutedColor
                    font.pixelSize: 11
                    Layout.fillWidth: true
                }

                Text {
                    text: "›"
                    color: root.textMutedColor
                    font.pixelSize: 24
                }
            }
        }

        MouseArea {
            id: subsystemMouseArea
            anchors.fill: parent
            onClicked: subsystemCard.clicked()
        }
    }

    component DtcCategoryCard: Rectangle {
        id: dtcCard

        property string title: ""
        property string subtitle: ""

        property var codes: []

        property bool responseReceived: false

        Layout.fillWidth: true

        implicitHeight:
            dtcCardContent.implicitHeight + 30

        radius: root.cardRadius

        color: root.surfaceColor

        border.color: root.borderColor
        border.width: 1

        ColumnLayout {
            id: dtcCardContent

            anchors.left: parent.left
            anchors.right: parent.right
            anchors.top: parent.top

            anchors.margins: 15

            spacing: 10

            RowLayout {
                Layout.fillWidth: true

                Text {
                    Layout.fillWidth: true

                    text: dtcCard.title
                    color: root.textPrimaryColor

                    font.pixelSize: 18
                    font.bold: true
                }

                StatusPill {
                    text: !dtcCard.responseReceived
                          ? "No response"
                          : dtcCard.codes.length === 1
                            ? "1 code"
                            : dtcCard.codes.length + " codes"

                    statusColor:
                        !dtcCard.responseReceived
                        ? root.textMutedColor
                        : dtcCard.codes.length > 0
                          ? root.warningColor
                          : root.successColor
                }
            }

            Text {
                Layout.fillWidth: true

                text: dtcCard.subtitle
                color: root.textSecondaryColor

                font.pixelSize: 12

                wrapMode: Text.WordWrap
            }

            Rectangle {
                Layout.fillWidth: true
                height: 1

                color: root.borderColor
            }

            Text {
                Layout.fillWidth: true

                visible:
                    !dtcCard.responseReceived ||
                    dtcCard.codes.length === 0

                text: !dtcCard.responseReceived
                      ? "The vehicle did not return a response for this DTC mode."
                      : "No trouble codes were reported."

                color: root.textSecondaryColor

                font.pixelSize: 13

                wrapMode: Text.WordWrap
            }

            Repeater {
                model:
                    dtcCard.responseReceived
                    ? dtcCard.codes
                    : []

                delegate: Rectangle {
                    required property string modelData

                    Layout.fillWidth: true
                    implicitHeight: dtcCodeContent.implicitHeight + 24

                    radius: 10
                    color: root.surfaceRaisedColor
                    border.color: root.borderColor
                    border.width: 1

                    ColumnLayout {
                        id: dtcCodeContent

                        anchors.left: parent.left
                        anchors.right: parent.right
                        anchors.top: parent.top
                        anchors.margins: 12

                        spacing: 5

                        Text {
                            Layout.fillWidth: true

                            text: modelData
                            color: root.textPrimaryColor

                            font.pixelSize: 16
                            font.bold: true
                        }

                        Text {
                            Layout.fillWidth: true

                            text: root.backendAvailable
                                  ? root.backend.dtcDescription(modelData)
                                  : ""

                            color: root.textSecondaryColor

                            font.pixelSize: 12
                            wrapMode: Text.WordWrap
                            lineHeight: 1.12
                        }
                    }
                }
            }
        }
    }


    component EmptyState: Column {
        property string title: ""
        property string message: ""

        spacing: 8

        Text {
            width: parent.width
            text: parent.title
            color: root.textPrimaryColor
            font.pixelSize: 18
            font.bold: true
            horizontalAlignment: Text.AlignHCenter
            wrapMode: Text.WordWrap
        }

        Text {
            width: parent.width
            text: parent.message
            color: root.textSecondaryColor
            font.pixelSize: 12
            horizontalAlignment: Text.AlignHCenter
            wrapMode: Text.WordWrap
            lineHeight: 1.15
        }
    }
}
