#ifndef _SERIAL_CLASS_
#define _SERIAL_CLASS_

#include <boost/asio.hpp>
#include <string>
#include <iostream>
#include <jsoncpp/json/json.h>
#include <fstream>
#include <termios.h>

using namespace std;
using namespace boost::asio;

class SERIAL
{
private:
    io_service io;
    serial_port serial;
    bool is_open_serial;
    string portName{};
    int serial_fd;
public:
    /// @brief 构造函数，串口参数设置
    /// @param port 端口号“/dev/ttyUSB0”
    SERIAL(const string &portName);


    /// @brief 析构，关闭串口
    ~SERIAL();

    /// @brief 串口参数设置
    /// @param baudRate 波特率 115200
    /// @param character_size 数据位 8
    /// @param Calibration 校验位 0：none，1：odd，2：even
    /// @param stop 停止位 0：onepointfive, 1：one， 2：two
    /// @return 
    void set_serail_option(int baudRate, int character_size, int Calibration, int stop);

    /// @brief 打开串口
    /// @return true：成功；false：失败
    bool openSerial();

    void readSome(string *outData);

    void witeSome(double *witeBuff, size_t size_);

};

/// @brief 读取参数文件，加载串口配置
/// @param root 输出参数配置
void getParam(Json::Value *root);

#endif
