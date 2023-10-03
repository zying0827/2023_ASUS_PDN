#include "VoltEigen.h"

void VoltEigen::setMatrix(size_t rowNodeId, size_t colNodeId, double resistance) {
    _G[rowNodeId][rowNodeId] += (1/resistance);
    _G[rowNodeId][colNodeId] -= (1/resistance);
}
void VoltEigen::setInputVector(size_t rowNodeId, double inputVolt, double resistance) {
    _G[rowNodeId][rowNodeId] += (1/resistance);
    _I[rowNodeId] += (inputVolt/resistance);
}