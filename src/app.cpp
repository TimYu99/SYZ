//------------------------------------------ Includes ----------------------------------------------

#include "app.h"
#include "platform/debug.h"
#include "global.h" // 包含 global.h
#include "data_logger.h"

#include <map>
#include <string>

using namespace IslSdk;
char sonar1[] = "sonar1 \r\n";
char sonar2 [] = "sonar2 \r\n";

// 每一个声呐的工作状态；1：正常工作；0：断线重连中；
extern std::map < std::string, int > sonar_status;
// 每一个声呐的断线重连次数
extern std::map < std::string, int > sonar_reconnect_count;
// 是否有一个声呐正在工作
extern bool sonar_working_flag;

//--------------------------------------------------------------------------------------------------
App::App(const std::string& name) : name(name), m_device(nullptr)
{
}
//--------------------------------------------------------------------------------------------------
App::~App()
{
}
//--------------------------------------------------------------------------------------------------
void App::setDevice(const Device::SharedPtr& device)
{
    if (m_device != nullptr)
    {
        m_device->onError.disconnect(slotError);
        m_device->onDelete.disconnect(slotDelete);
        m_device->onConnect.disconnect(slotConnect);
        m_device->onDisconnect.disconnect(slotDisconnect);
        m_device->onPortAdded.disconnect(slotPortAdded);
        m_device->onPortChanged.disconnect(slotPortChanged);
        m_device->onPortRemoved.disconnect(slotPortRemoved);
        m_device->onInfoChanged.disconnect(slotDeviceInfo);
        m_device->onPacketCount.disconnect(slotPacketCount);

        disconnectSignals(*m_device);
    }

    m_device = device;

    if (m_device != nullptr)
    {
        m_device->onError.connect(slotError);
        m_device->onDelete.connect(slotDelete);
        m_device->onConnect.connect(slotConnect);
        m_device->onDisconnect.connect(slotDisconnect);
        m_device->onPortAdded.connect(slotPortAdded);
        m_device->onPortChanged.connect(slotPortChanged);
        m_device->onPortRemoved.connect(slotPortRemoved);
        m_device->onInfoChanged.connect(slotDeviceInfo);
        //m_device->onPacketCount.connect(slotPacketCount);

        connectSignals(*m_device);
    }
}
//--------------------------------------------------------------------------------------------------
void App::doTask(int_t key, const std::string& path)
{
}
//--------------------------------------------------------------------------------------------------
void App::callbackError(Device& device, const std::string& msg)
{
    Debug::log(Debug::Severity::Error, name.c_str(), "%s", msg.c_str());
}
//--------------------------------------------------------------------------------------------------
void App::callbackDeleteted(Device& device)
{
    Debug::log(Debug::Severity::Warning, name.c_str(), "%04u.%04u deleteted", device.info.pn, device.info.sn);
}
//--------------------------------------------------------------------------------------------------
void App::callbackConnect(Device& device)
{
    Debug::log(Debug::Severity::Notice, name.c_str(), "%04u.%04u connected and ready%s", device.info.pn, device.info.sn, device.bootloaderMode() ? " (bootloader mode)" : "");

    // 这次是重连
    if (sonar_status[device.info.pnSnAsStr()] == 0) {
        sonar_reconnect_count[device.info.pnSnAsStr()] = 0; // 重连次数清零
        printf("%s 设备重连计数器清零: %d\n", device.info.pnSnAsStr().c_str(), sonar_status[device.info.pnSnAsStr()]);
        sonar_working_flag = 1; // 设置工作标志为1，有机器在工作
    }

    // 保存设备信息到全局变量
    globalPn = device.info.pn;
    globalSn = device.info.sn;
    if (globalPn == 2255 && globalSn == 10)
    {
        globalstatus = 0x02; // 设置 Bit1 为 1

        saveData("D:/ceshi/Seriallog.txt", sonar2, strlen(sonar2), "Work:", 0);
    }
    else if (globalPn == 2254 && globalSn == 25)
    {
        globalstatus = 0x01; // 设置 Bit0 为 1
        saveData("D:/ceshi/Seriallog.txt", sonar1, strlen(sonar1), "Work:", 0);
    }
    else {
        globalstatus = 0x00; // 设成默认值
    }
    connectEvent(device);
}
//--------------------------------------------------------------------------------------------------
void App::callbackDisconnect(Device& device)
{
    // 更新 sonar_status 中的设备信息
    std::string key = device.info.pnSnAsStr();
    sonar_status[key] = 0; // 设置状态为 2，表示断线重连中
    bool temp_flag = 0;
    for (auto& it : sonar_status) {
        printf("%s 设备的状态：%d\n", it.first.c_str(), it.second);
        temp_flag |= it.second; // 检查是否有设备在工作
    }
    sonar_working_flag = temp_flag;
    printf("是否有设备在工作：%d\n", sonar_working_flag);

    Debug::log(Debug::Severity::Notice, name.c_str(), "%04u.%04u disconnected", device.info.pn, device.info.sn);
}
//--------------------------------------------------------------------------------------------------
void App::callbackPortAdded(Device& device, SysPort& port, const ConnectionMeta& meta)
{
    Debug::log(Debug::Severity::Info, name.c_str(), "Port %s added to %04u.%04u", port.name.c_str(), device.info.pn, device.info.sn);
}
//--------------------------------------------------------------------------------------------------
void App::callbackPortChanged(Device& device, SysPort& port, const ConnectionMeta& meta)
{
    if (port.type == SysPort::Type::Serial)
    {
        Debug::log(Debug::Severity::Info, name.c_str(), "Baudrate for %04u.%04u on port %s changed to %u", device.info.pn, device.info.sn, port.name.c_str(), meta.baudrate);
    }
    else if (port.type == SysPort::Type::Net)
    {
        Debug::log(Debug::Severity::Info, name.c_str(), "IP address for %04u.%04u on port %s changed to %u.%u.%u.%u", device.info.pn, device.info.sn, port.name.c_str(), meta.ipAddress & 0xff, (meta.ipAddress >> 8) & 0xff, (meta.ipAddress >> 16) & 0xff, (meta.ipAddress >> 24) & 0xff);
    }
}
//--------------------------------------------------------------------------------------------------
void App::callbackPortRemoved(Device& device, SysPort& port)
{
    Debug::log(Debug::Severity::Info, name.c_str(), "Port %s removed from %04u.%04u", port.name.c_str(), device.info.pn, device.info.sn);
}
//--------------------------------------------------------------------------------------------------
void App::callbackDeviceInfo(Device& device, const Device::Info& info)
{
    Debug::log(Debug::Severity::Info, name.c_str(), "Device Info changed %s", device.bootloaderMode() ? "now in bootloader mode" : "");
}
//--------------------------------------------------------------------------------------------------
void App::callbackPacketCount(Device& device, uint_t tx, uint_t rx, uint_t resent, uint_t missed)
{
    Debug::log(Debug::Severity::Info, name.c_str(), "Packet stats for %04u.%04u Tx:%u, Rx:%u, Resent:%u, Missed:%u", FMT_U(device.info.pn), FMT_U(device.info.sn), tx, rx, resent, missed);
}
//--------------------------------------------------------------------------------------------------

