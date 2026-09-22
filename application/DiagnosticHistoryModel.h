#pragma once

#include <QAbstractListModel>
#include <QString>
#include <QVector>

class DiagnosticHistoryModel :
                               public QAbstractListModel
{
    Q_OBJECT

public:
    enum Roles
    {
        CaseIdRole = Qt::UserRole + 1,
        ConditionRole,
        ActionLevelRole,
        EvidenceSourceRole,
        DescriptionRole
    };

    explicit DiagnosticHistoryModel(
        QObject* parent = nullptr);

    int rowCount(
        const QModelIndex& parent =
        QModelIndex()) const override;

    QVariant data(
        const QModelIndex& index,
        int role =
        Qt::DisplayRole) const override;

    QHash<int, QByteArray>
    roleNames() const override;

    void prependResult(
        int caseId,
        const QString& condition,
        const QString& actionLevel,
        const QString& evidenceSource,
        const QString& description);

    void clear();

private:
    struct Entry
    {
        int caseId = 0;

        QString condition;
        QString actionLevel;
        QString evidenceSource;
        QString description;
    };

    QVector<Entry> m_entries;
};
