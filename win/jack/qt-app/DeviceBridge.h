#pragma once

#include <QObject>

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>

#include "cm_const.h"
#include "cm_msg.h"
#include "cmapi.h"

// Owns the cm_init() callback registration and re-publishes CM traffic as a
// Qt signal. This is the direct replacement for every
// PostMessage(WM_CM_MESSAGE/WM_CM_LOG/..., ...) in MainFrm.cpp: the cmio_t
// callback fires on whatever thread the device I/O lives on, and emitting a
// signal from there is safe because Qt detects the sender/receiver thread
// mismatch at delivery time and queues it onto the receiver's (GUI) thread
// automatically - no explicit invokeMethod/PostMessage needed.
class DeviceBridge : public QObject
{
    Q_OBJECT

public:
    explicit DeviceBridge(QObject *parent = nullptr);

    // Registers this instance with cmapi. Only one DeviceBridge may be
    // active at a time (cm_init() takes a single global callback).
    uint32_t init();

signals:
    void cmTraffic(quint8 opCode, quintptr msg);

private:
    static void trafficThunk(uint8_t opCode, pcm_msg_t msg);

    static DeviceBridge *s_instance;
};
