#include "serial.hpp"


SERIAL::SERIAL(const string &portName_): serial(io, portName_)
{
    is_open_serial = false;
    portName = portName_;
    serial_fd = serial.native_handle();

    tcflush(serial_fd, TCIOFLUSH);
}

SERIAL::~SERIAL()
{
    if (is_open_serial)
        serial.close();
}

void SERIAL::set_serail_option(int baudRate, int character_size, int Calibration, int stop)
{
    serial.set_option(serial_port_base::baud_rate(baudRate));//波特率设置
    serial.set_option(serial_port_base::character_size(character_size));//数据位设置
    switch (Calibration)//校验位设置
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
    switch (stop)//停止位
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
    serial.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));//流控
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



void SERIAL::readSome(string *outData)
{
    char *Buffdata = new char[1024];
    string dataBuff{}, data{};
    
    size_t _i = read(serial, buffer(Buffdata, 1024));
    dataBuff += Buffdata;
    //cout << "i:" << _i << " buff:"<<dataBuff.data() << endl;
    if (0 < _i)
    {
        size_t pos_hander, pos_end;
        pos_hander = dataBuff.find_first_of("x");
        pos_end = dataBuff.find_first_of("m");

        if (pos_hander > pos_end)
        {
            dataBuff = dataBuff.substr(pos_hander);
            pos_end = dataBuff.find_first_of("m");
            data = dataBuff.substr(0, pos_end);
        }
        else
            data = dataBuff.substr(pos_hander, pos_end - pos_hander);
    }
    *outData = data;
    delete[] Buffdata;
    tcflush(serial_fd, TCIFLUSH);
}

void SERIAL::witeSome(double *witeBuff, size_t size_)
{
    // string write_buffee = "v1" + to_string(witeBuff[0]) 
    //                     + "v2" + to_string(witeBuff[1])
    //                     + "v3" + to_string(witeBuff[2])
    //                     + "v4" + to_string(witeBuff[3])
    //                     + "v5" + to_string(witeBuff[4])
    //                     + "v6" + to_string(witeBuff[5])
    //                     + "m";

    string write_buffee{};
    for (size_t i = 0; i < 6; i++)
    {
        write_buffee += "v" + to_string(i) + to_string(witeBuff[i]);
        if(6 == i)
            write_buffee += "m";
    }
    

    int witeIdx = write(serial, buffer(write_buffee.c_str(), write_buffee.length()));
    cout << "计数：" << witeIdx << "  [ " << write_buffee.data() << " ]\n";
    tcflush(serial_fd, TCOFLUSH);
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