#pragma once

#include <Arduino.h>
#include <math.h>
#include <stdint.h>

struct PistonIFNode
{
    int16_t left;
    int16_t right;
    int8_t feature;
    float threshold;
    float leafPath;
};


// Returns the same score definition used in V4:
// -model.decision_function(X)
//
// Higher score = more abnormal.
static inline float pistonIsolationForestScore(
    const PistonIFNode* nodes,
    const uint32_t* treeOffsets,
    uint16_t treeCount,
    float normalization,
    float modelOffset,
    const float* featureValues,
    const float* imputerMedians)
{
    float depthSum = 0.0f;

    for (
        uint16_t treeIndex = 0;
        treeIndex < treeCount;
        treeIndex++
    )
    {
        uint32_t treeStart =
            treeOffsets[treeIndex];

        int16_t nodeIndex = 0;

        while (true)
        {
            const PistonIFNode& node =
                nodes[
                    treeStart
                    + nodeIndex
                ];

            // leaf reached
            if (node.feature < 0)
            {
                depthSum +=
                    node.leafPath;

                break;
            }

            float value =
                featureValues[
                    node.feature
                ];

            // use the same median fill as Python
            if (isnan(value))
            {
                value =
                    imputerMedians[
                        node.feature
                    ];
            }

            if (value <= node.threshold)
            {
                nodeIndex =
                    node.left;
            }
            else
            {
                nodeIndex =
                    node.right;
            }
        }
    }

    float anomalyScore =
        powf(
            2.0f,
            -depthSum
            /
            normalization
        );

    return (
        anomalyScore
        +
        modelOffset
    );
}


static inline bool pistonIsolationForestAlert(
    float score,
    float threshold)
{
    return score >= threshold;
}
