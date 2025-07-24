#include "serial.hpp"


SERIAL::SERIAL(const string &portName_): serial(io, portName_)
{
    is_open_serial = false;
    portName = portName_;
}

SERIAL::~SERIAL()
{
    if (is_open_serial)
        serial.close();
}

void SERIAL::set_serail_option(int baudRate, int character_size, int Calibration, int stop)
{
    serial.set_option(serial_port_base::baud_rate(baudRate));
    serial.set_option(serial_port_base::character_size(character_size));
    switch (Calibration)
    {
    case 0:
        serial.set_option(serial_port_base::parity(serial_port_base::parity::none));
        break;
    case 1:
        serial.set_option(serial_port_base::parity(serial_port_base::parity::odd));
        break;
    case 2:
        serial.set_option(serial_port_base::parity(serial_port_base::parity::even));
        break;
    
    default:
        serial.set_option(serial_port_base::parity(serial_port_base::parity::none));
        break;
    }
    switch (stop)
    {
    case 0:
        serial.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::onepointfive));
        break;
    case 1:
        serial.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
        break;
    case 2:
        serial.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::two));
        break;
    default:
        serial.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
        break;
    }
    serial.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));
}

bool SERIAL::openSerial()
{
    is_open_serial = serial.is_open();
    if (!is_open_serial)
    {
        serial.open(portName);
    }
    is_open_serial = serial.is_open();
    return is_open_serial;
}



void SERIAL::readSome(double *outData)
{
    float dataBuff[9];
    size_t _i = read(serial, buffer(dataBuff, 9));
    for (size_t i = 0; i < _i; i++)
       outData[i] = dataBuff[i];
}


void SERIAL::witeSome(double *witeBuff, size_t size_)
{
    float dataBuff[size_];
    cout << "写： " ;
    for (size_t i = 0; i < size_; i++)
    {
        dataBuff[i] = witeBuff[i];
        cout << dataBuff[i] << " " ;
    }
    cout << endl;
    
    //write(serial, buffer(dataBuff, 6));
}


void getParam(Json::Value *root)
{
    ifstream paramFile;
    string filePath = "../config/serial.json";
    paramFile.open(filePath);
    if(!paramFile.is_open())
    {
        cout << "串口配置文件打开失败，检查文件 ：" << filePath.c_str() << endl;
        return;
    }
    Json::Reader reader_{};
    Json::Value Param{};
    reader_.parse(paramFile, Param);
    *root = Param;
    paramFile.close();
}