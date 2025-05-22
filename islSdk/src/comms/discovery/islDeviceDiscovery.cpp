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

// ������ device.cpp ��
// ÿһ�����ŵĹ���״̬��1������������2�����������У�
extern std::map < std::string, int > sonar_status;
// ���Ŷ�Ӧ�Ķ˿�����:��һ��������id���ڶ����Ƕ˿�����
extern std::map < std::string, std::string > sonar_port_name;
// ÿһ�����ŵĶ�����������
extern std::map < std::string, int > sonar_reconnect_count;
// �Ƿ���һ���������ڹ���
extern bool sonar_working_flag;

extern SeriallPort serialPort2;//����2

int sonar1_first_connect_search_count = 0; // ��һ�����ӵĳ�ʱ�ж�
int sonar2_first_connect_search_count = 0; // ��һ�����ӵĳ�ʱ�ж�

int isl_device_discovery_bytes_written = 0;;

void IslDeviceDiscovery::sendToNextLevel(const std::string& portName, const std::string& message)
{
    uint32_t baudrate = 115200;  // ������Ҫ����������
    const char* buffer = message.c_str();
    serialPort2.write(buffer, strlen(buffer), isl_device_discovery_bytes_written);
    saveData("D:/ceshi/Seriallog.txt", buffer, strlen(buffer), "System Wrong", 0);
    // ���������ͣ1��
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

                // �������ӹ����豸
                if (sonar_port_name.size() != 0) {
                    for (auto& it : sonar_port_name) {
                        // �����ǰû���κ�һ���ڹ�������ô���Ӹ����豸�ĳ�ʱ��ѯ
                        // ������豸�ڹ������Ͳ������κ��豸�ĳ�ʱ��ѯ����Ϊ�ڽ��湤��
                        if (sonar_working_flag == false) 
                        {
                            // �����ڶ�Ӧ���豸���ӳ�ʱ��ѯ
                            if (std::string(m_sysPort.name.c_str()) == it.second) {
                                sonar_reconnect_count[it.first]++;
                                printf("%s �豸���ӳ�ʱ��ѯ������%d\n", it.first.c_str(), sonar_reconnect_count[it.first]);

                                // ���Ŷ���֮����ٴ��ˣ�֪ͨ��Ƭ��
                                if (sonar_reconnect_count[it.first] == 10000) 
                                {
                                    // ����1����֮����ٴ��ˣ�֪ͨ��Ƭ��
                                    if (it.first =="2255.0024" ) //�ǵøĻ���//3��������
                                    {
                                        std::string temp_send_string;
                                        temp_send_string = "$SMSN,OFF,0*";//�رս��湤��
                                        temp_send_string += IslSdk::SeriallPort::calculateChecksum(temp_send_string) + "\r\n";
                                        sendToNextLevel("COM10", temp_send_string);
                                        temp_send_string = "$SMSN,ONTWO,2*";//��������2
                                        temp_send_string += IslSdk::SeriallPort::calculateChecksum(temp_send_string) + "\r\n";
                                        sendToNextLevel("COM10", temp_send_string);
                                    }
                                    // ����2����֮����ٴ��ˣ�֪ͨ��Ƭ��
                                    if (it.first == "2254.0025")
                                    {
                                        std::string temp_send_string;
                                        temp_send_string = "$SMSN,OFF,0*";
                                        temp_send_string += IslSdk::SeriallPort::calculateChecksum(temp_send_string) + "\r\n";
                                        sendToNextLevel("COM10", temp_send_string);
                                        temp_send_string = "$SMSN,ONONE,1*";//��������2
                                        temp_send_string += IslSdk::SeriallPort::calculateChecksum(temp_send_string) + "\r\n";
                                        sendToNextLevel("COM10", temp_send_string);
                                    }
                                }
                                

                            }
                        }
                    }
                }
                else {
                    // ��һ�����ӵĳ�ʱ�ж�
                    if (std::string(m_sysPort.name.c_str()) == "COM3") {
                        sonar1_first_connect_search_count++;
                        printf("����1��һ�����ӵĳ�ʱ�ж�++��%d\n", sonar1_first_connect_search_count);
                    }

                    // ����һ�����ӵ���ѯ��������10�Σ����͸���Ƭ��
                    if (std::string(m_sysPort.name.c_str()) == "COM3" && sonar1_first_connect_search_count == 60) {
                            printf("���ŵ�һ�����ӳ��׳�ʱ�������͸���Ƭ����\n");
                            std::string temp_send_string;
                            temp_send_string = "$SMSN,OFF,0*";
                            temp_send_string += IslSdk::SeriallPort::calculateChecksum(temp_send_string) + "\r\n";
                            sendToNextLevel("COM10", temp_send_string);
                            temp_send_string = "$SMSN,ONTWO,2*";
                            temp_send_string += IslSdk::SeriallPort::calculateChecksum(temp_send_string) + "\r\n";
                            sendToNextLevel("COM10", temp_send_string);
                    }

                    if (std::string(m_sysPort.name.c_str()) == "COM4")
                    {
                        printf("����2��һ�����ӵĳ�ʱ�ж�++��%d\n", sonar2_first_connect_search_count);
                        sonar2_first_connect_search_count++;
                    }
                    // ������2���ӵ���ѯ��������20�Σ������ڵ�һ���ϵ��һ��ʱ�䣬��ô���͸���Ƭ��
                    if (std::string(m_sysPort.name.c_str()) == "COM4" && sonar2_first_connect_search_count == 120) {
                        printf("����2��һ�����ӳ��׳�ʱ�������͸���Ƭ����\n");
                        if (true){}
                        // do something...
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
// ������ѯ����
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
