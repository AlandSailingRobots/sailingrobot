#include "CANService.h"

bool registerForReading(CanPGNReceiver& node, uint32_t PGN) {
  m_RegisteredNodes.insert(PGN, node);
}
