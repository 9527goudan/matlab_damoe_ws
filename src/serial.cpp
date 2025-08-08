#include "serial.hpp"


SERIAL::SERIAL(const string &portName_): serial(io, portName_)
{
    is_open_serial = false;
    is_asyncRuning = false;
    is_updata = false;
    portName = portName_;
    serial_fd = serial.native_handle();

    asyncBuffLenger = 256;
    readBuff = vector<char> (asyncBuffLenger);

    tcflush(serial_fd, TCIOFLUSH);
}

SERIAL::~SERIAL()
{
    this->Close();
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



void SERIAL::readSome(string *outData, size_t cache_ = 1024)
{
    char *Buffdata = new char[cache_];
    string dataBuff{}, data{};
    
    size_t _i = read(serial, buffer(Buffdata, cache_));
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

void SERIAL::witeSome(double *witeBuff)
{
    string write_buffee{};
    for (size_t i = 0; i < 6; i++)
        write_buffee += "v" + to_string(i + 1) + to_string(witeBuff[i]);
    write_buffee += "m";

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


void SERIAL::asyncRun()
{
    is_asyncRuning = true;
    io.run();
}

void SERIAL::asyncStop()
{
    io.stop();
}

void SERIAL::Close()
{
    if(this->is_open_serial)
    {
        if(is_asyncRuning)
        {
            this->asyncStop();
            is_asyncRuning = false;
        }
        this->serial.close();
    }
}

void SERIAL::handleRead(const boost::system::error_code &error_, size_t byte_read)
{
    if (is_asyncRuning && !error_)
    {
        for (size_t i = 0; i < byte_read; i++)
            matlabV6Async += readBuff[i];

        size_t pos_hander, pos_end;
        pos_hander = matlabV6Async.find_first_of("x");
        pos_end = matlabV6Async.find_first_of("m");
        if(pos_hander < pos_end && pos_end < asyncBuffLenger)
        {
            cout << "witeBuff : [" << matlabV6Async << " ]" << endl;
            float Pf[6] = {0, 0 , 700};
            float sensor_lenger[6]{};

            sscanf(witeBuff.c_str(), "x%fy%fz%fp1%fp2%fp3%fp4%fp5%fp6%fm", &Pf[3], &Pf[4], &Pf[5], 
                    &sensor_lenger[0], &sensor_lenger[1], &sensor_lenger[2], &sensor_lenger[3], &sensor_lenger[4], &sensor_lenger[5]);

            witeBuff.clear();
            double sensor_lenger_[6]{}, Pf_[6]{};
                    
            for (size_t i = 0; i < 6; i++)
            {
                sensor_lenger_[i] = sensor_lenger[i];
                Pf_[i] = Pf[i]; 
            }                
            //读-->数据处理-->计算-->写   测试暂用同步处理，效果不理想改异步处理，可能改善效果
            double vec[6]{};
            stewart_control_function_V4_part_test_1(this->planning_.data, this->planning_.dataLenger, this->planning_.dataSize, sensor_lenger_, Pf_, vec);
            this->witeSome(vec);
            matlabV6Async.clear();
        }
        if(asyncBuffLenger < matlabV6Async.length())
            matlabV6Async.clear();
    
        startAsyncRead();
    }
}


void SERIAL::startAsyncRead()
{
    this->serial.async_read_some(buffer(readBuff),
                                boost::bind(&SERIAL::handleRead,
                                            this,
                                            boost::asio::placeholders::error,
                                            boost::asio::placeholders::bytes_transferred));
}


void SERIAL::serialAsyncRunSerive()
{
    this->startAsyncRead();
    this->asyncRun();
}


void SERIAL::setAsyncBuffLenger(int lenger_ = 256)
{
    asyncBuffLenger = lenger_;
    readBuff = vector<char> (asyncBuffLenger);
}

void SERIAL::setPlanningStorage(double *planningData, int dataLenger, int *dataSize)
{
    planning_.data = planningData;
    planning_.dataLenger = dataLenger;
    planning_.dataSize = dataSize;
}


void SERIAL::handleWite(const boost::system::error_code &error_)
{
    if (is_asyncRuning && !error_)
    {
        /* code */
    }
    

}


void SERIAL::AsyncWite(string witBuff)
{
    serial.async_write_some(buffer(witBuff), 
                            boost::bind(&SERIAL::handleWite,
                            this,
                            boost::asio::placeholders::error));

}

void SERIAL::speedPlanning()
{
    if (is_updata)
    {
        cout << "witeBuff : [" << witeBuff << " ]" << endl;
        float Pf[6] = {0, 0 , 700};
        float sensor_lenger[6]{};

        sscanf(witeBuff.c_str(), "x%fy%fz%fp1%fp2%fp3%fp4%fp5%fp6%fm", &Pf[3], &Pf[4], &Pf[5], 
                &sensor_lenger[0], &sensor_lenger[1], &sensor_lenger[2], &sensor_lenger[3], &sensor_lenger[4], &sensor_lenger[5]);

        witeBuff.clear();
        double sensor_lenger_[6]{}, Pf_[6]{};
                
        for (size_t i = 0; i < 6; i++)
        {
            sensor_lenger_[i] = sensor_lenger[i];
            Pf_[i] = Pf[i]; 
        }                
        //读-->数据处理-->计算-->写   测试暂用同步处理，效果不理想改异步处理，可能改善效果
        double vec[6]{};
        stewart_control_function_V4_part_test_1(this->planning_.data, this->planning_.dataLenger, this->planning_.dataSize, sensor_lenger_, Pf_, vec);
        this->witeSome(vec);
    }
    is_updata = false;
}

