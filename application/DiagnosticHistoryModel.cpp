#include "DiagnosticHistoryModel.h"

DiagnosticHistoryModel::
    DiagnosticHistoryModel(QObject* parent)
    : QAbstractListModel(parent)
{
}

int DiagnosticHistoryModel::rowCount(
    const QModelIndex& parent) const
{
    if (parent.isValid())
    {
        return 0;
    }

    return m_entries.size();
}

QVariant DiagnosticHistoryModel::data(
    const QModelIndex& index,
    int role) const
{
    if (!index.isValid() ||
        index.row() < 0 ||
        index.row() >= m_entries.size())
    {
        return QVariant();
    }

    const Entry& entry =
        m_entries.at(index.row());

    switch (role)
    {
    case CaseIdRole:
        return entry.caseId;

    case ConditionRole:
        return entry.condition;

    case ActionLevelRole:
        return entry.actionLevel;

    case EvidenceSourceRole:
        return entry.evidenceSource;

    case DescriptionRole:
        return entry.description;

    default:
        return QVariant();
    }
}

QHash<int, QByteArray>
DiagnosticHistoryModel::roleNames() const
{
    return
        {
            { CaseIdRole, "caseId" },
            { ConditionRole, "condition" },
            { ActionLevelRole, "actionLevel" },
            { EvidenceSourceRole, "evidenceSource" },
            { DescriptionRole, "description" }
        };
}

void DiagnosticHistoryModel::prependResult(
    int caseId,
    const QString& condition,
    const QString& actionLevel,
    const QString& evidenceSource,
    const QString& description)
{
    beginInsertRows(
        QModelIndex(),
        0,
        0);

    m_entries.prepend(
        {
            caseId,
            condition,
            actionLevel,
            evidenceSource,
            description
        });

    endInsertRows();
}

void DiagnosticHistoryModel::clear()
{
    if (m_entries.isEmpty())
    {
        return;
    }

    beginResetModel();

    m_entries.clear();

    endResetModel();
}
