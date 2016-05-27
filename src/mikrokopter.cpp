
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <tf2/LinearMath/Quaternion.h>
#include <sensor_msgs/Imu.h>
#include <mikrokopter_node/BatteryStatus.h>
#include "mikrokopter_node/mikrokopter.h"

using namespace std;
using namespace mikrokopter_node;
using namespace mikrokopter;

/**
 * Error codes of the UART/Serial communication
 */
enum DATA_READ_ERROR_CODES{
  WAITING_FOR_START = -10,
  WAITING_FOR_ADDRESS = -9,
  WAITING_FOR_COMMAND = -8,
  BUFFER_OVERFLOW = -7,
  WAITING_FOR_DATA = -6,
  READ_ERROR = -5,
  UNKNOWN = -4,
};


void Mikrokopter::initTopics(ros::NodeHandle& nh)
{
  _imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 10);
  _battery_pub = nh.advertise<mikrokopter_node::BatteryStatus>("batterystatus", 10);
  _fc_pub = nh.advertise<mikrokopter_node::FCStatus>("fc_status", 10);
  _sub_ctrl = nh.subscribe("control", 1, &Mikrokopter::controlCallback, this);
  _service_motor_test = nh.advertiseService("test_motors", &Mikrokopter::setMotorTest, this);
  _service_arming = nh.advertiseService("arm_motors", &Mikrokopter::arm, this);
  _service_beep = nh.advertiseService("beep", &Mikrokopter::beep, this);
  _service_calibration = nh.advertiseService("calibration", &Mikrokopter::runCalibration, this);
}

Mikrokopter::Mikrokopter(ros::NodeHandle& nh):
    _stop(true), _current_termination_symbol(NEWLINE)
{
    std::string tty_port;
    nh.param<std::string>("tty_port", tty_port, "/dev/ttyUSB0");

    ROS_INFO("Using tty port: %s", tty_port.c_str());

    _fd_tty = serialPort(tty_port.c_str());
    if(_fd_tty < 0) {
        ROS_ERROR("Can not connect to mikrokopter via %s", tty_port.c_str());
    }
    else {
        ROS_INFO("Connected to mikrokopter via %s", tty_port.c_str());
        _stop = false;

        //first disable debug output if it was enabled before
        waitForDebugUnsubscription();

        ROS_INFO("Requesting FLIGHTCTRL Version");
        getVersion();
        ROS_INFO("FLIGHTCTRL_INFO: FlightCtrl V%d.%d-p%d", _versionInfo.SWMajor, _versionInfo.SWMinor, _versionInfo.SWPatch);

        ROS_INFO("Checking for Hardware Errors");
        checkHardwareError();

        initTopics(nh);

        _listen_thread = std::thread(&Mikrokopter::run, this);

        //request debug information from MikroKopter Flight Control
        //Refreshing the subscription is not required because of the modified firmware
        debugRequest();
    }
}

#define CHECK_ERROR(id, value, error) \
    if (value & FC_ERROR##id##error) { \
        ROS_ERROR("FLIGHTCTRL_ERROR: Hardware error: %s", #error); }


void Mikrokopter::checkHardwareError() {
    auto error0 = _versionInfo.HardwareError[0];
    auto error1 = _versionInfo.HardwareError[1];

    if (error0 == 0 && error1 == 0) {
        return;
    }

    CHECK_ERROR(0_, error0, GYRO_ROLL);
    CHECK_ERROR(0_, error0, GYRO_YAW);
    CHECK_ERROR(0_, error0, ACC_NICK);
    CHECK_ERROR(0_, error0, ACC_ROLL);
    CHECK_ERROR(0_, error0, ACC_TOP);
    CHECK_ERROR(0_, error0, PRESSURE);
    CHECK_ERROR(0_, error0, CAREFREE);

    CHECK_ERROR(1_, error1, I2C);
    CHECK_ERROR(1_, error1, BL_MISSING);
    CHECK_ERROR(1_, error1, SPI_RX);
    CHECK_ERROR(1_, error1, PPM);
    CHECK_ERROR(1_, error1, MIXER);
    CHECK_ERROR(1_, error1, RES1);
    CHECK_ERROR(1_, error1, RES2);
    CHECK_ERROR(1_, error1, RES3);
}

Mikrokopter::~Mikrokopter() {
    if (isReady()) {
        stop();
        debugUnsubscribe();
        _listen_thread.join();
        close(_fd_tty);
        ROS_INFO("Disconnected from mikrokopter");
    }
}

void Mikrokopter::getVersion() {
    _read_buf[0] = 0;
    _read_buf[1] = 0;
    bool valid = false;

    int max_attempts_per_config = 8;
    int current_attempts = 0;

    // TODO do not wait endless for connection
    // add a time check
    do {
        versionRequest();
        sleep(1);
        valid = getData(FC_ADDRESS, 'V', &_versionInfo, sizeof(_versionInfo));
        //if we do not get a valid response it is possible that we are working
        //with the wrong termination symbol
        if(!valid ){
          if(++current_attempts % max_attempts_per_config == 0){
            switchTerminationSymbol();
          }
        }
    } while (!valid);

    if(!valid){
      std::string error("Could not get version information!");
      ROS_ERROR("%s", error.c_str());
      throw -1; //TODO use proper exception class
    }
}

void Mikrokopter::getAnalogLabels() {
    // Note: this method is very slow TODO: Why?

    _analog_labels.clear();
    _analog_labels.resize(32);
    AnalogLabel label;
    for(unsigned char i=0; i<32; i++) {
        bool valid = false;
        do {
            requestAnalogLabel(i);
            valid = getData(FC_ADDRESS, 'A', &label, sizeof(label));
            valid = valid && (label.index == i);
        } while(!valid);
        _analog_labels[i] = label.text;
        cout<<int(i)<<" "<<label.text<<endl;
    }
}

int Mikrokopter::isReadReady() {
    FD_ZERO(&_read_fds);
    FD_SET(_fd_tty,&_read_fds);
    _timeout.tv_sec = 1;
    _timeout.tv_usec = 0;

    auto rc = select(_fd_tty+1, &_read_fds, NULL, NULL, &_timeout);

    if (rc < 0)
        return -1;

    return FD_ISSET(_fd_tty,&_read_fds) ? 1 : 0;
}

void Mikrokopter::sendData(unsigned char cmd, unsigned char *snd, unsigned char len) {
    boost::mutex::scoped_lock scoped_lock(_mtx_serial);
    unsigned int pt = 0,i,tmpCRC = 0;
    unsigned char a,b,c;
    unsigned char ptr = 0;
    _send_buf[pt++] = '#';
    _send_buf[pt++] = FC_ADDRESS;
    _send_buf[pt++] = cmd;
    while(len) {
        if(len) { a=snd[ptr++]; len--; } else a=0;
        if(len) { b=snd[ptr++]; len--; } else b=0;
        if(len) { c=snd[ptr++]; len--; } else c=0;
        _send_buf[pt++] = '=' + (a >> 2);
        _send_buf[pt++] = '=' + (((a & 0x03) << 4) | ((b & 0xf0) >> 4));
        _send_buf[pt++] = '=' + (((b & 0x0f) << 2) | ((c & 0xc0) >> 6));
        _send_buf[pt++] = '=' + (c & 0x3f);
    }
    for(i = 0; i < pt; i++) {
        tmpCRC += _send_buf[i];
    }
    tmpCRC %= 4096;

    _send_buf[i++] = '=' + tmpCRC / 64;
    _send_buf[i++] = '=' + tmpCRC % 64;
    _send_buf[i++] = _current_termination_symbol;

    write(_fd_tty, _send_buf, i);
    tcdrain(_fd_tty);
}

//XXX it would be possible to wait in this routine until we receive the correct message with a timeout...
/**
 * Protocol http://www.mikrokopter.de/ucwiki/en/SerialProtocol
 */
int Mikrokopter::readData(unsigned char addr, unsigned char command_id) {

  /**
   * UART/SERIAL state machine state definitions
   */
  enum UartState{
    START = 1,
    ADDRESS,
    COMMAND,
    PAYLOAD
  };

  boost::mutex::scoped_lock scoped_lock(_mtx_serial);

  UartState state = START;

  int count = 0;
  unsigned char c;
  while(!_stop) {
      if (isReadReady() == 1) {
          if(read(_fd_tty, &c, 1) < 0) {
              ROS_ERROR("Cannot read char");
              return READ_ERROR;
          }

          if(state == START && c == '#')
          {
              //command start
              count = 0;
              state = ADDRESS;
          }else{
            if (state == ADDRESS && c == addr)
            {
              state = COMMAND;
            }else if (state == COMMAND && c == command_id)
            {
              state = PAYLOAD;
            }else if (state == PAYLOAD)
            {
              if(c == _current_termination_symbol)
              {
                break;
              }else if(count >= SERIAL_BUFFER_SIZE) {
                ROS_ERROR("FLIGHTCTRL_ERROR: buffer overflow");
                return BUFFER_OVERFLOW;
              }
            }
            else{
              int return_code;

              //TODO make this only for debug
              switch(state){
                case START:
                  ROS_INFO("FLIGHTCTRL_INFO: Waiting for start-symbol # != %c", c);
                  return_code = WAITING_FOR_START;
                  break;
                case ADDRESS:
                  ROS_INFO("FLIGHTCTRL_INFO: Waiting for address-symbol %c != %c", addr, c);
                  return_code = WAITING_FOR_ADDRESS;
                  break;
                case COMMAND:
                  ROS_INFO("FLIGHTCTRL_INFO: Waiting for command-symbol %c != %c", command_id, c);
                  return_code = WAITING_FOR_COMMAND;
                  break;
                default:
                  ROS_INFO("FLIGHTCTRL_INFO: Unknown Error State: %c", c);
                  return_code = UNKNOWN;
              }
              //not the expected message
              state = START;
              //would also be possible to continue directly but this would require an timeout
              return return_code;
            }
            _read_buf[count] = c;
            count++;
          }
      } else {
          ROS_WARN("FLIGHTCTRL_WARN: Waiting for data from FightCtrl");
          return WAITING_FOR_DATA;
      }
  }
  return count;
}

void Mikrokopter::decode(unsigned char* input, unsigned char* output, unsigned int len) {
    unsigned char a,b,c,d;
    unsigned char ptrIn = 0;
    unsigned char ptrOut = 0;
    unsigned char x,y,z;
    while(len) {
        a = input[ptrIn++] - '=';
        b = input[ptrIn++] - '=';
        c = input[ptrIn++] - '=';
        d = input[ptrIn++] - '=';
        x = (a << 2) | (b >> 4);
        y = ((b & 0x0f) << 4) | (c >> 2);
        z = ((c & 0x03) << 6) | d;
        if(len--) output[ptrOut++] = x; else break;
        if(len--) output[ptrOut++] = y; else break;
        if(len--) output[ptrOut++] = z; else break;
    }
}

void Mikrokopter::switchTerminationSymbol()
{
  ROS_INFO("Switching the symbol now");
  if (_current_termination_symbol == NEWLINE)
  {
    _current_termination_symbol = CARRIAGE_RETURN;
  }
  else
  {
    _current_termination_symbol = NEWLINE;
  }
}

/**
 * addr address of the device usually it is the FC_ADDRESS
 * id id of the command we want to receive
 * data buffer pointer
 * data_size size of the buffer
 */
bool Mikrokopter::getData(unsigned char addr, unsigned char id, void* data, unsigned int data_size) {
    int read_bytes = readData(addr, id);

    if(read_bytes == BUFFER_OVERFLOW){
      ROS_WARN("BUFFER OVERFLOW should only occur with wrong termination symbol. ");
      switchTerminationSymbol();
    }

    if(read_bytes <= 0){
      return false;
    }

    if (!checksum(read_bytes)) {
        _read_buf[read_bytes] = '\0';
        ROS_ERROR("FLIGHTCTRL_ERROR: Received data has error! \ndata = %s", _read_buf);
        return false;
    }

    // base64 data should be bigger than requested data
    unsigned int base64_size = (read_bytes - 4) / 4 * 3;
    ROS_ASSERT(base64_size >= data_size);
    ROS_ASSERT(base64_size - 3 < data_size);

    decode(&_read_buf[2], static_cast<unsigned char*>(data), data_size);
    return true;
}

bool Mikrokopter::readDebugData()
{
  return getData(FC_ADDRESS, 'D', &_debug_out, sizeof(_debug_out));
}

void Mikrokopter::run()
{
    while(!_stop) {
        auto valid = readDebugData();
        if (valid) {
          publishDebugOut();
        }else{
          debugRequest();
        }
        //TODO usleep(10000) check correct value
    }
}

void Mikrokopter::waitForDebugUnsubscription(){

  int max_attempts_per_config = 8;
  int current_attempts = 0;
  //while we are still receiving debug information at startup we try to unsubscribe it
  ROS_INFO("Waiting for debug subscription termination");

  while(readData(FC_ADDRESS, 'D') != WAITING_FOR_DATA ){
    debugUnsubscribe();
    sleep(2); //required to make flush work, for some reason
    tcflush(_fd_tty, TCIOFLUSH); //flush all old data in the buffer

    //after some attempts we switch the termination symbol
    if(++current_attempts % max_attempts_per_config == 0){
      switchTerminationSymbol();
    }

    ROS_INFO_STREAM(".");
  }
  ROS_INFO("Waiting for debug subscription finished");
}


void Mikrokopter::publishIMU(std_msgs::Header& header)
{
  if(_imu_pub.getNumSubscribers() < 1 ){
    return;
  }
  sensor_msgs::Imu imu;
  imu.header = header;

  tf2::Quaternion orientation;
  const tf2Scalar angle_factor = 0.1 * M_PI / 360;
  tf2Scalar roll = _debug_out.Analog[DebugOutAnalog::ANGLE_ROLL] * angle_factor;
  tf2Scalar pitch = _debug_out.Analog[DebugOutAnalog::ANGLE_NICK] * angle_factor;
  tf2Scalar yaw = _debug_out.Analog[DebugOutAnalog::GYRO_COMPASS] * angle_factor;
  orientation.setRPY(roll, pitch, yaw);
  imu.orientation.w = orientation.w();
  imu.orientation.x = orientation.x();
  imu.orientation.y = orientation.y();
  imu.orientation.z = orientation.z();

  imu.angular_velocity.z = _debug_out.Analog[DebugOutAnalog::YAW_GYRO]; // TODO: convert to SI

  // TODO: test, see "MikroKopter Documentation" from Cyphy Laboratory
  imu.linear_acceleration.x = _debug_out.Analog[DebugOutAnalog::ACC_ROLL] / 606.3;
  imu.linear_acceleration.y = _debug_out.Analog[DebugOutAnalog::ACC_NICK] / 603.44;
  imu.linear_acceleration.z = _debug_out.Analog[DebugOutAnalog::ACC_Z] / 15.5 + 9.8;

  _imu_pub.publish(imu);
}

void Mikrokopter::publishBatteryStatus(std_msgs::Header& header)
{
  if(_battery_pub.getNumSubscribers() < 1 ){
    return;
  }
  mikrokopter_node::BatteryStatus battery_status;
  battery_status.header = header;

  battery_status.ActualCurrent_A = _debug_out.Analog[DebugOutAnalog::CURRENT] / 10.0f; // debug is 0.1A steps
  battery_status.UsedCapacity_mAh = (float)_debug_out.Analog[DebugOutAnalog::CAPACITY];
  battery_status.Voltage_V = _debug_out.Analog[DebugOutAnalog::VOLTAGE] / 10.0; // debug is 0.1V steps

  _battery_pub.publish(battery_status);
}

void Mikrokopter::publishFCStatus(std_msgs::Header& header)
{
  if( _fc_pub.getNumSubscribers() < 1 ){
    return;
  }
  fc_status_.header = header;

  fc_status_.stickPitch = _debug_out.Analog[DebugOutAnalog::STICK_PITCH];
  fc_status_.stickRoll = _debug_out.Analog[DebugOutAnalog::STICK_ROLL];
  fc_status_.stickYaw = _debug_out.Analog[DebugOutAnalog::STICK_YAW];
  fc_status_.stickThrust = _debug_out.Analog[DebugOutAnalog::STICK_THRUST];
  fc_status_.motor1 = _debug_out.Analog[DebugOutAnalog::MOTOR_1];
  fc_status_.motor2 = _debug_out.Analog[DebugOutAnalog::MOTOR_2];
  fc_status_.motor3 = _debug_out.Analog[DebugOutAnalog::MOTOR_3];
  fc_status_.motor4 = _debug_out.Analog[DebugOutAnalog::MOTOR_4];
  fc_status_.hoverThrottle = _debug_out.Analog[DebugOutAnalog::HOVERGAS];
  fc_status_.externalControl = _debug_out.Analog[DebugOutAnalog::ACTIVATED_EXTERNAL_CONTROL] == 1;
  fc_status_.calibrationDone = _debug_out.Analog[DebugOutAnalog::CALIBRATION_DONE] == 1;
  fc_status_.motorsArmed = fc_status_.motor1 > 0 && fc_status_.motor2 > 0 && fc_status_.motor3 > 0 && fc_status_.motor4 > 0;

  _fc_pub.publish(fc_status_);
}

void Mikrokopter::publishDebugOut() {
    _header_published.seq++;
    _header_published.stamp = ros::Time::now();
    _header_published.frame_id = "mikrokopter";

    publishIMU(_header_published);
    publishBatteryStatus(_header_published);
    publishFCStatus(_header_published);
}

bool Mikrokopter::checksum(int max) {
    max -= 2;
    unsigned int tmpCRC = '#';
    for(int i = 0; i<max; i++) {
        tmpCRC += _read_buf[i];
    }
    tmpCRC %= 4096;
    unsigned char CRC1 = '=' + tmpCRC / 64;
    unsigned char CRC2 = '=' + tmpCRC % 64;

    return (CRC1 == _read_buf[max]) && (CRC2 == _read_buf[max + 1]);
}

/**
 * Start debug data stream from MikroKopter FlightControl
 */
void Mikrokopter::debugRequest() {
    ROS_INFO("Sending debug data request");
    u_int8_t interval = 1; // interval in 10 hertz steps
    sendData('d', &interval, sizeof(interval));
}

/**
 * Stop debug data stream from MikroKopter FlightControl
 */
void Mikrokopter::debugUnsubscribe() {
    u_int8_t interval = 0; //0 = disable
    sendData('d', &interval, sizeof(interval));
}

void Mikrokopter::versionRequest() {
    sendData('v', NULL, 0);
}

void Mikrokopter::requestAnalogLabel(unsigned char index) {
    sendData('a', &index, 1);
}

/**
* \brief  Serial Setup
*
*         Function to set up the serial connection
*         to the mikrokopter.
*
* \return file descriptor, negative in case of errors
*
*/
int Mikrokopter::serialPort(const char* tty_port) {
    int fd = open(tty_port, O_RDWR | O_NOCTTY);
    if(fd == -1) {ROS_ERROR("Connection to serial port failed: could not connect"); return -1;}

    fcntl(fd, F_SETFL, 0);
    struct termios config;

    if(!isatty(fd)) {ROS_ERROR("Connection to serial port failed: port is not a tty"); return -3;}
    if(tcgetattr(fd, &config) < 0) {ROS_ERROR("Connection to serial port failed: unable to open settings"); return -2;}

    //set baudrate
    cfsetispeed(&config, B57600);
    cfsetospeed(&config, B57600);

    config.c_cflag &= ~PARENB;
    config.c_cflag &= ~CSTOPB;
    config.c_cflag &= ~CSIZE;
    config.c_cflag |= CS8;

    tcsetattr(fd, TCSANOW, &config);

    return fd;
}

void Mikrokopter::controlCallback(const mikrokopter_node::Control& control){
  /* Roll/Pitch value influence coordinates in Mikrokopter flight control
   * positive pitch moves forward, positive roll to the left
  *           Pitch
  *            (+)
  *             ^
  *             |
  * (+)Roll  <--|--
  *             |
  *
  * Neutral is 128
  * but we transfer the addition to the stick position and there neutral is 128
  *
  */

  unsigned char extCtrl[11] = {0,0,0,(unsigned char)(-1) * control.pitch,(unsigned char)(-1) * control.roll,(unsigned char)control.yaw,(unsigned char)control.thrust,0,0,1,1};
  sendData('b', extCtrl, 11);
}

bool Mikrokopter::setMotorTest(MotorTest::Request &test, MotorTest::Response &res){
  unsigned char motortest[16] = {test.front,test.rear,test.right,test.left,0,0,0,0,0,0,0,0,0,0,0,0};
  sendData('t', motortest, sizeof(motortest));
  return true;
}

bool Mikrokopter::runCalibration(Calibration::Request &calib, Calibration::Response &res){

  unsigned char calibrationMode;

  if(calib.withPersistence){
    ROS_INFO("Calibrate and Store");
    calibrationMode = 2;
  }else{
    ROS_INFO("Calibrate");
    calibrationMode = 1;
  }

  // wait and check if this was really executed
  //try 10 times in 10 seconds
  for(int i = 0; i< 10; i++)
  {
    sendData('r', &calibrationMode, 1);
    sleep(1);
    if(fc_status_.calibrationDone){
      return true;
    }
  }

  return false;
}

bool Mikrokopter::beep(Beep::Request &beep, Beep::Response &res){

  unsigned char beepLength[3] = {0};

  memcpy(beepLength, &beep.beepLength, sizeof(beep.beepLength));

  beepLength[2] = beep.numberOfBeeps;

  ROS_INFO("Send beep");
  sendData('o', beepLength, sizeof(beepLength));

  return true;
}

bool Mikrokopter::arm(Arm::Request &arm, Arm::Response &res){

  unsigned char cmd;
  if(arm.motorsRunning){
    ROS_INFO("Arming motors");
    cmd = 'x';
  }else{
    ROS_INFO("Unarming motors");
    cmd = 'e';
  }

  // wait and check if this was really executed
  //try 10 times in 10 seconds
  for(int i = 0; i< 10; i++)
  {
    sendData(cmd, NULL, 0);
      sleep(1);
      if(fc_status_.motorsArmed == arm.motorsRunning){
        return true;
      }
  }

  return true;
}
