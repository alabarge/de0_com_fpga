#include "DeviceBridge.h"

DeviceBridge *DeviceBridge::s_instance = nullptr;

DeviceBridge::DeviceBridge(QObject *parent)
    : QObject(parent)
{
}

uint32_t DeviceBridge::init()
{
    s_instance = this;
    return cm_init(&DeviceBridge::trafficThunk);
}

void DeviceBridge::trafficThunk(uint8_t opCode, pcm_msg_t msg)
{
    if (s_instance)
        emit s_instance->cmTraffic(opCode, reinterpret_cast<quintptr>(msg));
}
