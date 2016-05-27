
#include <thread>

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <mikrokopter_node/Control.h>
#include <boost/thread/mutex.hpp>
#include "mikrokopter_node/MotorTest.h"
#include "mikrokopter_node/Calibration.h"
#include "mikrokopter_node/Beep.h"
#include "mikrokopter_node/Arm.h"
#include <mikrokopter_node/FCStatus.h>


#define FC_ADDRESS      'a' + 1  //(b)

namespace mikrokopter{

//This struct is copied from uart.h of mikrokopter flight control source code
struct str_VersionInfo
{
  unsigned char SWMajor;
  unsigned char SWMinor;
  unsigned char ProtoMajor;
  unsigned char LabelTextCRC;
  unsigned char SWPatch;
  unsigned char HardwareError[2];
  unsigned char HWMajor;
  unsigned char BL_Firmware;
  unsigned char Flags;
};

// bitmask for HardwareError[0]
#define FC_ERROR0_GYRO_NICK             0x01
#define FC_ERROR0_GYRO_ROLL             0x02
#define FC_ERROR0_GYRO_YAW              0x04
#define FC_ERROR0_ACC_NICK              0x08
#define FC_ERROR0_ACC_ROLL              0x10
#define FC_ERROR0_ACC_TOP               0x20
#define FC_ERROR0_PRESSURE              0x40
#define FC_ERROR0_CAREFREE              0x80

// bitmask for HardwareError[1]
#define FC_ERROR1_I2C                   0x01
#define FC_ERROR1_BL_MISSING            0x02
#define FC_ERROR1_SPI_RX                0x04
#define FC_ERROR1_PPM                   0x08
#define FC_ERROR1_MIXER                 0x10
#define FC_ERROR1_RES1                  0x20
#define FC_ERROR1_RES2                  0x40
#define FC_ERROR1_RES3                  0x80

#define SERIAL_BUFFER_SIZE              256


struct str_DebugOut {
    unsigned char Status[2];
    int16_t Analog[32];    // Note: we have 16 bit integer here!
};


struct AnalogLabel {
    int8_t index;
    char text[16];
};

enum DebugOutAnalog {
    ANGLE_NICK = 0, // in 0.1 deg
    ANGLE_ROLL, // in 0.1 deg
    ACC_NICK,
    ACC_ROLL,
    YAW_GYRO,
    HEIGHT_VALUE,
    ACC_Z,
    GAS,
    COMPASS_VALUE,
    VOLTAGE, // 0.1V
    RECEIVER_LEVEL,
    GYRO_COMPASS,
    MOTOR_1,
    MOTOR_2,
    MOTOR_3,
    MOTOR_4,
    STICK_PITCH,  // ONLY in custom build
    STICK_ROLL,   // ONLY in custom build
    STICK_YAW,    // ONLY in custom build
    STICK_THRUST, // ONLY in custom build
    SERVO = 20,
    HOVERGAS,
    CURRENT, // 0.1A
    CAPACITY, // mAh
    HEIGHT_SET_POINT,
    ACTIVATED_EXTERNAL_CONTROL, // ONLY in custom build
    CALIBRATION_DONE, // ONLY in custom build
    COMPASS_SET_POINT = 27,
    I2C_ERROR,
    BL_LIMIT,
    GPS_NICK,
    GPS_ROLL
};

/**
 * There seems to be a dependency on the language settings of the system that
 * results in a different tty uart interpretation of the termination symbol
 * Actually it is supposed to be '\r' but with English locales we need to configure '\n'
 */
enum TERMINATION_SYMBOL  : unsigned char
  { NEWLINE = '\n', CARRIAGE_RETURN = '\r'
};

class Mikrokopter {
public:
    Mikrokopter(ros::NodeHandle& nh);

    ~Mikrokopter();

    bool isReady() const { return !_stop; }

private:
    int serialPort(const char* tty_port);

    void sendData(unsigned char cmd, unsigned char *snd, unsigned char len);

    int isReadReady();

    int readData(unsigned char addr, unsigned char command_id);

    bool readDebugData();

    void waitForDebugUnsubscription();

    bool getData(unsigned char addr, unsigned char id, void* data, unsigned int data_size);

    static void decode(unsigned char* input, unsigned char* output, unsigned int len);

    bool checksum(int max);

    void debugRequest();
    void debugUnsubscribe();

    void versionRequest();

    void run();

    void stop() { _stop = true; }

    void getVersion();

    void checkHardwareError();

    void getAnalogLabels();

    void requestAnalogLabel(unsigned char index);

    void publishDebugOut();

    void publishIMU(std_msgs::Header& header);

    void publishBatteryStatus(std_msgs::Header& header);

    void publishFCStatus(std_msgs::Header& header);

    void controlCallback(const mikrokopter_node::Control& control);

    bool setMotorTest(mikrokopter_node::MotorTest::Request &test, mikrokopter_node::MotorTest::Response &res);

    bool runCalibration(mikrokopter_node::Calibration::Request &calib, mikrokopter_node::Calibration::Response &res);

    bool beep(mikrokopter_node::Beep::Request &beep, mikrokopter_node::Beep::Response &res);

    bool arm(mikrokopter_node::Arm::Request &arm, mikrokopter_node::Arm::Response &res);

    void switchTerminationSymbol();

    void initTopics(ros::NodeHandle& nh);

private:
    bool _stop;
    int _fd_tty;
    unsigned char _read_buf[SERIAL_BUFFER_SIZE];
    unsigned char _send_buf[SERIAL_BUFFER_SIZE];

    TERMINATION_SYMBOL _current_termination_symbol;

    std::thread _listen_thread;

    boost::mutex _mtx_serial;

    std_msgs::Header _header_published;
    mikrokopter_node::FCStatus fc_status_;

    ros::Publisher _imu_pub;
    ros::Publisher _battery_pub;
    ros::Publisher _fc_pub;
    ros::Subscriber _sub_ctrl;
    ros::ServiceServer _service_motor_test;
    ros::ServiceServer _service_arming;
    ros::ServiceServer _service_beep;
    ros::ServiceServer _service_calibration;

    fd_set _read_fds;
    struct timeval _timeout;

    str_VersionInfo _versionInfo;
    str_DebugOut _debug_out;
    std::vector<std::string> _analog_labels;
};

}
