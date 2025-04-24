//------------------------------------------ Includes ----------------------------------------------

#include "comms/discovery/islDeviceDiscovery.h"
#include "comms/islHdlc.h"
#include "comms/ports/sysPort.h"
#include "platform/timeUtils.h"
#include "platform/debug.h"
#include "utils/stringUtils.h"
#include "data_logger.h"
#include "sonarApp.h"

#include <map>
#include <string>

using namespace IslSdk;

// 定义在 device.cpp 中
// 每一个声呐的工作状态；1：正常工作；2：断线重连中；
extern std::map < std::string, int > sonar_status;
// 声呐对应的端口名字
extern std::map < std::string, std::string > sonar_port_name;
// 每一个声呐的断线重连次数
extern std::map < std::string, int > sonar_reconnect_count;
// 是否有一个声呐正在工作
extern bool sonar_working_flag;

extern SeriallPort serialPort2;//串口2

int isl_device_discovery_bytes_written = 0;;

void IslDeviceDiscovery::sendToNextLevel(const std::string& portName, const std::string& message)
{
    uint32_t baudrate = 115200;  // 根据需要调整波特率
    const char* buffer = message.c_str();
    serialPort2.write(buffer, strlen(buffer), isl_device_discovery_bytes_written);
    saveData("D:/ceshi/Seriallog.txt", buffer, strlen(buffer), "COM2 Send", 0);
    // 处理完后暂停1秒
    //std::this_thread::sleep_for(std::chrono::seconds(1));
    //uartPort10_.write(reinterpret_cast<const uint8_t*>(message.c_str()), message.size(), ConnectionMeta(115200));
}

//--------------------------------------------------------------------------------------------------
IslDeviceDiscovery::IslDeviceDiscovery(SysPort& sysPort) : AutoDiscovery(sysPort, Type::Isl ), m_timeoutMs(0)
{
}
//--------------------------------------------------------------------------------------------------
IslDeviceDiscovery::~IslDeviceDiscovery()
{
    m_sysPort.unblock(m_sysPort.id);
}
//--------------------------------------------------------------------------------------------------
bool_t IslDeviceDiscovery::run()
{
    if (!m_discoveryParam.empty())
    {
        DiscoveryParam& param = m_discoveryParam.front();
        uint64_t timeMs = Time::getTimeMs();

        if (m_timeoutMs == 0)
        {
            debugLog("IslDeviceDiscovery", "%s discovery started", m_sysPort.name.c_str());
            m_sysPort.onDiscoveryStarted(m_sysPort, AutoDiscovery::Type::Isl);
        }

        if (timeMs >= m_timeoutMs)
        {
            const std::vector<uint8_t> buf = IslHdlc::BuildDiscovery(param.pid, param.pn, param.sn);
            m_sysPort.unblock(m_sysPort.id);
            bool_t written = m_sysPort.write(&buf[0], buf.size(), param.meta);
            m_sysPort.block(m_sysPort.id);

            if (written)
            {
                m_timeoutMs = timeMs + param.timeoutMs;
                param.count--;
                debugLog("IslDeviceDiscovery", "%s discovering at %s", m_sysPort.name.c_str(), m_sysPort.type == SysPort::Type::Net ? (StringUtils::ipToStr(param.meta.ipAddress) + ":" + StringUtils::toStr(param.meta.port)).c_str() : StringUtils::toStr(param.meta.baudrate).c_str());

                // 遍历连接过的设备
                if (sonar_port_name.size() != 0) {
                    for (auto& it : sonar_port_name) {
                        // 如果当前没有任何一个在工作，那么增加各个设备的超时轮询
                        // 如果有设备在工作，就不增加任何设备的超时轮询，因为在交替工作
                        if (sonar_working_flag == false) {
                            // 给串口对应的设备增加超时轮询
                            if (std::string(m_sysPort.name.c_str()) == it.second) {
                                sonar_reconnect_count[it.first]++;
                                printf("%s 设备增加超时轮询次数：%d\n", it.first.c_str(), sonar_reconnect_count[it.first]);

                                // 断线之后多少次了，通知单片机
                                if (sonar_reconnect_count[it.first] > 10000) {
                                    if (it.first == "2255.0025") {
                                        std::string temp_send_string;
                                        temp_send_string = "$SMSN,OFTWO,2*";
                                        temp_send_string += IslSdk::SeriallPort::calculateChecksum(temp_send_string) + "\r\n";
                                        sendToNextLevel("COM10", temp_send_string);
                                    }
                                }
                            }
                        }
                    }
                }
                
                m_sysPort.onDiscoveryEvent(m_sysPort, param.meta, AutoDiscovery::Type::Isl, discoveryCount);

                if (param.count == 0)
                {
                    m_discoveryParam.pop_front();
                }
            }
            else
            {
                m_timeoutMs = timeMs + 10;
            }
        }
    }
    else if (m_isDiscovering)
    {
        if (Time::getTimeMs() >= m_timeoutMs)
        {
            m_timeoutMs = 0;
            m_isDiscovering = false;
            m_sysPort.unblock(m_sysPort.id);
            debugLog("IslDeviceDiscovery", "%s discovery finished", m_sysPort.name.c_str());
            m_sysPort.onDiscoveryFinished(m_sysPort, AutoDiscovery::Type::Isl, m_discoveryCount, false);
        }
    }

    return !m_isDiscovering;
}
//--------------------------------------------------------------------------------------------------
void IslDeviceDiscovery::stop()
{
    m_timeoutMs = 0;
    m_discoveryParam.clear();

    if (m_isDiscovering)
    {
        m_sysPort.unblock(m_sysPort.id);
        m_sysPort.onDiscoveryFinished(m_sysPort, AutoDiscovery::Type::Isl, m_discoveryCount, true);
    }

    m_isDiscovering = false;
}
//--------------------------------------------------------------------------------------------------
// 创建轮询任务
void IslDeviceDiscovery::addTask(uint16_t pid, uint16_t pn, uint16_t sn, const ConnectionMeta& meta, uint_t timeoutMs, uint_t count)
{
    for (DiscoveryParam& param : m_discoveryParam)
    {
        if (param.pid == pid && param.pn == pn && param.sn == sn && !param.meta.isDifferent(meta))
        {
            param.timeoutMs = timeoutMs;
            param.count += count;
            return;
        }
    }

    if (count)
    {
        if (m_discoveryParam.empty())
        {
            m_timeoutMs = 0;
            m_discoveryCount = 0;
            m_isDiscovering = true;
        }
        m_discoveryParam.emplace_back(DiscoveryParam(pid, pn, sn, meta, timeoutMs, count));
    }
}
//--------------------------------------------------------------------------------------------------
void IslDeviceDiscovery::removeTask(uint16_t pid, uint16_t pn, uint16_t sn)
{
    std::list<DiscoveryParam>::iterator it = m_discoveryParam.begin();

    while (it != m_discoveryParam.end())
    {
        if (it->pid == pid && it->pn == pn && it->sn == sn)
        {
            if (it == m_discoveryParam.begin())
            {
                m_timeoutMs = Time::getTimeMs();
            }
            it = m_discoveryParam.erase(it);
            continue;
        }
        it++;
    }
}
//--------------------------------------------------------------------------------------------------
