#ifndef SONARAPP_H_
#define SONARAPP_H_

//------------------------------------------ Includes ----------------------------------------------

#include "app.h"
#include "devices/sonar.h"
#include "imuManager.h"
#include "helpers/sonarDataStore.h"
#include "helpers/sonarImage.h"
#include <string>
#include <vector>
#include <mutex>
#include <windows.h> // 添加 Windows API 头文件
#include <opencv2/opencv.hpp>


//--------------------------------------- Class Definition -----------------------------------------

namespace IslSdk
{
    struct model_param {
        double mu;      // 均值
        double sigma2;  // 方差
        int t;          // 当前迭代了几帧
    };

    struct Message {
        char header[7];
        uint16_t length;
        uint16_t year;
        uint8_t month;
        uint8_t day;
        uint8_t hour;
        uint8_t minute;
        uint8_t second;
        uint8_t status;
        uint16_t angle;
        uint8_t speed;
        uint8_t minrange;
        uint8_t maxrange;
        uint16_t reserved;
        uint16_t checksum;
        uint8_t end1;
        uint8_t end2;
    };
    std::string messageToString(const Message& msg);
    class SonarApp : public App
    {
    public:
        SonarApp(void);
        std::string header;
        uint16_t length;
        uint16_t year;
        uint8_t month;
        uint8_t day;
        uint8_t hour;
        uint8_t minute;
        uint8_t second;
        uint8_t status;
        uint16_t angle;
        uint8_t speed;
        uint32_t reserved;
        uint16_t checksum;
        uint8_t end1;
        uint8_t end2;
        int m_sonar_app_index;

        // 新算法所需
        model_param model_array[101][200];//1、这里得改，2、角度映射，前面的
        int circle_times = 0;
        int flag_making_model = -1; // -1表示未建模，1表示建模中，0表示建模完成
        int g_frame_count = 0;
        cv::Mat qiang_du_tu;
        cv::Mat is_goal; // 目标图像


        void processReceivedData(const std::vector<uint8_t>& data);
        void SonarApp::readUartData(const std::string& portName, uint32_t baudrate);
        void initializeTime();
        uint16_t addcalculateChecksum(const uint8_t* data, size_t size);

        int convert_angle2index(int angle);
        double inverse_normal_cdf(double p);
        void recursive_update(model_param& params, double x, double learning_rate);
        bool detect_anomaly(const model_param& params, double current_value);
        void write_model_param(std::string file_path);
        double lognormalPDF(double x, double mu, double sigma);

        void SonarApp::sendFormattedData
        (
            const std::string& portName,
            uint32_t baudrate,
            uint16_t year, uint8_t month, uint8_t day,
            uint8_t hour, uint8_t minute, uint8_t second,
            uint8_t status, uint16_t angle, uint8_t speed,
            uint8_t minrange,
            uint8_t maxrange
        );
        ~SonarApp(void);
        void renderPalette(const std::string& path);
        void connectSignals(Device& device) override;
        void disconnectSignals(Device& device) override;
        void doTask(int_t key, const std::string& path) override;

        Slot<Sonar&, bool_t, Sonar::Settings::Type> slotSettingsUpdated{ this, &SonarApp::callbackSettingsUpdated };
        Slot<Sonar&, const Sonar::HeadIndexes&> slotHeadIndexesAcquired{ this, &SonarApp::callbackHeadIndexesAcquired };
        Slot<Sonar&, const Sonar::Ping&> slotPingData{ this, &SonarApp::callbackPingData };
        Slot<Sonar&, const Sonar::Echos&> slotEchoData{ this, &SonarApp::callbackEchoData };
        Slot<Sonar&, const Sonar::CpuPowerTemp& > slotPwrAndTemp{ this, &SonarApp::callbackPwrAndTemp };
        Slot<Sonar&> slotMotorSlip { this, &SonarApp::callbackMotorSlip };
        Slot<Sonar&, bool_t> slotMotorMoveComplete { this, &SonarApp::callbackMotorMoveComplete };
        void sendUartData(const std::string& portName, uint32_t baudrate, const std::vector<uint8_t>& data);
        void saveShanxingToFile(const float* shanxing, int rows, int cols, const std::string& filename);
        
    private:
        //新添加
        std::string dataFolder_;
        std::string allPingDataFilename_;
        bool m_scanning;
        std::ofstream m_outputFile;
        int sequenceNumber_; // 序号变量
        int currentCol = 0;// 当前存储的列
        void setDataFolder(const std::string& basePath);
        void updatePingDataFilename();
        void writeInitialLog();
        void SonarApp::recordPingData(const Sonar& iss360, const Sonar::Ping& ping, uint_t txPulseLengthMm);
        void saveImageWithTimestamp_beijing(const cv::Mat& image);
        
        #define MAX_QUEUE_SIZE 1000
        std::queue <Sonar::Ping> pingQueue;
        std::mutex pingQueueMutex;
        std::condition_variable consumerCondition;
        Sonar* m_piss360;
        void consumePingData();

        AhrsManager ahrs;
        GyroManager gyro;
        AccelManager accel;

        Palette m_palette;
        SonarImage m_circular;
        SonarImage m_texture;
        uint_t m_pingCount;
        SonarDataStore sonarDataStore;
        virtual void connectEvent(Device& device);
        
        void callbackSettingsUpdated(Sonar& sonar, bool_t ok, Sonar::Settings::Type settingsType);
        void callbackHeadIndexesAcquired(Sonar& sonar, const Sonar::HeadIndexes& data);
        void callbackEchoData(Sonar& sonar, const Sonar::Echos& data);
        void callbackPingData(Sonar& sonar, const Sonar::Ping& data);
        void callbackPwrAndTemp(Sonar& sonar, const Sonar::CpuPowerTemp& data);
        void callbackMotorSlip(Sonar& sonar);
        void callbackMotorMoveComplete(Sonar& sonar, bool_t ok);
    };
    class SeriallPort {
    private:
        HANDLE hComm;  // 串口句柄
        DCB dcbSerialParams;  // 串口参数
        COMMTIMEOUTS timeouts;  // 超时参数

    public:
        SeriallPort();
        ~SeriallPort();
        bool open(const std::string& portName, int baudRate);
        bool close();
        bool read(char* buffer, int bufferSize, int& bytesRead);
        bool write(const char* buffer, int bufferSize, int& bytesWritten);

        static std::string calculateChecksum(const std::string& message);

    };
}

//--------------------------------------------------------------------------------------------------
#endif
