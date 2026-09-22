#pragma once

#include <QAbstractTableModel>
#include <QJsonArray>
#include <QVariant>
#include <QVector>
#include <QString>

class LiveDataModel : public QAbstractTableModel
{
    Q_OBJECT

public:
    explicit LiveDataModel(QObject* parent = nullptr);

    int rowCount(
        const QModelIndex& parent = QModelIndex()) const override;

    int columnCount(
        const QModelIndex& parent = QModelIndex()) const override;

    QVariant data(
        const QModelIndex& index,
        int role = Qt::DisplayRole) const override;

    QVariant headerData(
        int section,
        Qt::Orientation orientation,
        int role = Qt::DisplayRole) const override;

    void appendSamples(
        int caseId,
        const QJsonArray& samples);

    void clear();
    QString toCsv() const;

private:
    struct Row
    {
        QVector<QVariant> values;
    };

    QVector<Row> m_rows;
};
