#include "LiveDataModel.h"

#include <QJsonObject>
#include <QJsonValue>
#include <QStringList>
#include <QTextStream>

namespace
{
QVariant jsonValueForDisplay(
    const QJsonObject& object,
    const char* key)
{
    const QJsonValue value =
        object.value(QLatin1String(key));

    if (value.isNull() ||
        value.isUndefined())
    {
        return QVariant();
    }

    return value.toVariant();
}
}

LiveDataModel::LiveDataModel(QObject* parent)
    : QAbstractTableModel(parent)
{
}

int LiveDataModel::rowCount(
    const QModelIndex& parent) const
{
    if (parent.isValid())
    {
        return 0;
    }

    return m_rows.size();
}

int LiveDataModel::columnCount(
    const QModelIndex& parent) const
{
    if (parent.isValid())
    {
        return 0;
    }

    if (m_bank2AvailabilityKnown && !m_hasBank2) return 11;
    return 16;
}

QVariant LiveDataModel::data(
    const QModelIndex& index,
    int role) const
{
    if (!index.isValid() ||
        index.row() < 0 ||
        index.row() >= m_rows.size() ||
        index.column() < 0 ||
        index.column() >= columnCount())
    {
        return QVariant();
    }

    if (role != Qt::DisplayRole)
    {
        return QVariant();
    }

    return m_rows
        .at(index.row())
        .values
        .at(index.column());
}

QVariant LiveDataModel::headerData(
    int section,
    Qt::Orientation orientation,
    int role) const
{
    if (role != Qt::DisplayRole ||
        orientation != Qt::Horizontal)
    {
        return QVariant();
    }

    static const QStringList headers =
        {
            "Case",
            "Sample",
            "Time (s)",
            "RPM",
            "Engine Load (%)",
            "Module Voltage (V)",

            "STFT B1 (%)",
            "LTFT B1 (%)",
            "O2 B1S1 (V)",
            "O2 B1S2 (V)",
            "O2 B1S1 Eq",

            "STFT B2 (%)",
            "LTFT B2 (%)",
            "O2 B2S1 (V)",
            "O2 B2S2 (V)",
            "O2 B2S1 Eq"
        };

    if (section < 0 ||
        section >= headers.size())
    {
        return QVariant();
    }

    return headers.at(section);
}

void LiveDataModel::setBank2Availability(bool known, bool hasBank2)
{
    if (m_bank2AvailabilityKnown == known && m_hasBank2 == hasBank2)
    {
        return;
    }

    beginResetModel();

    m_bank2AvailabilityKnown = known;
    m_hasBank2 = hasBank2;

    endResetModel();
}

void LiveDataModel::appendSamples(
    int caseId,
    const QJsonArray& samples)
{
    if (samples.isEmpty())
    {
        return;
    }

    const int firstRow =
        m_rows.size();

    const int lastRow =
        firstRow + samples.size() - 1;

    beginInsertRows(
        QModelIndex(),
        firstRow,
        lastRow);

    for (const QJsonValue& sampleValue : samples)
    {
        const QJsonObject sample = sampleValue.toObject();

        Row row;

        row.values =
            {
                caseId,

                jsonValueForDisplay(
                    sample,
                    "sample_index"),

                sample.value("time_ms").isNull()
                    ? QVariant()
                    : QVariant(
                          sample.value("time_ms").toDouble()
                          / 1000.0),

                jsonValueForDisplay(
                    sample,
                    "rpm"),

                jsonValueForDisplay(
                    sample,
                    "engine_load"),

                jsonValueForDisplay(
                    sample,
                    "control_module_voltage"),

                jsonValueForDisplay(
                    sample,
                    "stft_b1"),

                jsonValueForDisplay(
                    sample,
                    "ltft_b1"),

                jsonValueForDisplay(
                    sample,
                    "o2_b1s1_voltage"),

                jsonValueForDisplay(
                    sample,
                    "o2_b1s2_voltage"),

                jsonValueForDisplay(
                    sample,
                    "o2_b1s1_equiv"),

                jsonValueForDisplay(
                    sample,
                    "stft_b2"),

                jsonValueForDisplay(
                    sample,
                    "ltft_b2"),

                jsonValueForDisplay(
                    sample,
                    "o2_b2s1_voltage"),

                jsonValueForDisplay(
                    sample,
                    "o2_b2s2_voltage"),

                jsonValueForDisplay(
                    sample,
                    "o2_b2s1_equiv")
            };

        m_rows.append(row);
    }

    endInsertRows();
}

void LiveDataModel::clear()
{
    if (m_rows.isEmpty() && !m_bank2AvailabilityKnown && !m_hasBank2)
    {
        return;
    }

    beginResetModel();

    m_rows.clear();

    m_bank2AvailabilityKnown = false;
    m_hasBank2 = false;

    endResetModel();
}

QString LiveDataModel::toCsv() const
{
    QString csv;

    QTextStream stream(&csv);


    // Header row
    for (int column = 0;
         column < columnCount();
         ++column)
    {
        if (column > 0)
        {
            stream << ",";
        }

        stream << headerData(
                      column,
                      Qt::Horizontal,
                      Qt::DisplayRole)
                      .toString();
    }

    stream << "\n";


    // Data rows
    for (const Row& row : m_rows)
    {
        for (int column = 0;
             column < columnCount();
             ++column)
        {
            if (column > 0)
            {
                stream << ",";
            }


            const QVariant& value =
                row.values.at(column);

            if (!value.isValid() ||
                value.isNull())
            {
                // Leave unavailable values blank.
                continue;
            }


            QString text =
                value.toString();

            // Standard CSV escaping
            text.replace(
                "\"",
                "\"\"");

            if (text.contains(",") ||
                text.contains("\"") ||
                text.contains("\n"))
            {
                text =
                    "\"" + text + "\"";
            }

            stream << text;
        }

        stream << "\n";
    }


    return csv;
}
