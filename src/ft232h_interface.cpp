
#include "ros/ros.h"

#include "std_msgs/Bool.h"
#include "std_msgs/Byte.h"
#include "std_msgs/ByteMultiArray.h"
#include "std_msgs/String.h"

#include <sstream>

#include <stdio.h>
#include <stdlib.h>
#include <ftdi.h>



int ret;//, i;
struct ftdi_context *ftdi;
struct ftdi_device_list *devlist, *curdev;
// char manufacturer[128], description[128];
int mnf_len, desc_len, serial_len;
int retval = EXIT_SUCCESS;
struct ftdi_version_info version;

struct libusb_device *device;
char *manufacturer, *description;

int V_ID = 0x0403;
int P_ID = 0x6014;
// char *SERIAL_NO = NULL;
int FTDI_CHIP_TYPE = TYPE_232H;
const char *serial_no = NULL;

// Buffer for configuring port direction
unsigned char port_dir = 0x00;
// Buffer for setting port state
unsigned char buf[1];
// Number of (used) pins, default to 8
int pins = 8;
int pin_dir[8] = {1,1,1,1,1,1,1,1};



// Subscriber call-back functions
void set_pin(const std_msgs::Bool::ConstPtr& msg, int index);
void set_pins(const std_msgs::Byte::ConstPtr& msg);

// FTDI functions
int ftdi_setup(void);
int ftdi_cleanup(void);


int main(int argc, char **argv){
  // ROS
  ros::init(argc, argv, "ftdi_interface");
  ros::NodeHandle nh("~");

  // Check for test mode
  bool testMode = false;
  nh.getParam("test", testMode);
  if(testMode) printf("Running in test mode.\n");
  // ToDo: Properly implement test mode

  // Get device parameters
  XmlRpc::XmlRpcValue deviceParameters;
  if(!nh.getParam("device", deviceParameters)){
      ROS_INFO("No device configuration given. Exiting.");
      return 0;
  }
  printf("Device paramters: %d.\n", deviceParameters.size());   //debug

  // Determine serial number to be used
  std::string deviceName = deviceParameters["name"];
  std::string deviceSerialNo = deviceParameters["serialNo"];
  std::string systemSerialNo;
  nh.getParam("serial_no", systemSerialNo);
  // Override config file setting if serial number is specified via launch file argument
  if(systemSerialNo.length() > 0){
    serial_no = systemSerialNo.c_str();
    // Debug output
    printf("Looking for system device '%s' with serial number '%s'.\n", deviceName.c_str(), systemSerialNo.c_str());
  }
  else{
    serial_no = deviceSerialNo.c_str();
    // Debug output
    printf("Looking for device '%s' with serial number '%s'.\n", deviceName.c_str(), deviceSerialNo.c_str());
  }

  // Get pin configuration
  XmlRpc::XmlRpcValue pinConfig;
  nh.getParam("pins", pinConfig);
  printf("Found %d pin configurations for topics:\n", pinConfig.size());

  // No. of utilized pins
  pins = pinConfig.size();

  // Initialize publishers and subscribers; here still unknown how they are split among *pins*
  ros::Publisher pinPub[pins];
  ros::Subscriber pinSub[pins];

  std_msgs::Bool boolMsg;
  std::string topic;
  std::string direction;
  // Configure pins according to pinConfig
  for(int i=0; i<pins; i++){
    topic = (std::string)pinConfig[i]["topic"];
    direction = (std::string)pinConfig[i]["direction"];

    printf("\t%s,\t%s\n", topic.c_str(), direction.c_str());   //debug

    // Pin output -> ROS subscriber
    if(direction == "output"){
      pinSub[i] = nh.subscribe<std_msgs::Bool>(topic, 1000, boost::bind(set_pin, _1, i));
      pin_dir[i] = 1;
    }
    // Pin input -> ROS publisher
    else if(direction == "input"){
      pinPub[i] = nh.advertise<std_msgs::Bool>(topic, 1000);
      pin_dir[i] = 0;
      // ToDo: Implement continuous/stateChange differentiation
    }
    // Robustness in case of erroneous config file
    else{
      ROS_INFO("Unknown pin direction. Exiting.");
      return 0;
    }
  }

  // Publisher for whole IO port
  ros::Publisher portPub;
  std::string devicePortTopic = deviceParameters["portTopic"];
  if(devicePortTopic.c_str()){
    printf("%s\n", devicePortTopic.c_str());
    portPub = nh.advertise<std_msgs::Byte>(devicePortTopic.c_str(), 1000);
  }

  int loopRate = deviceParameters["loopRate"];
  ros::Rate loop_rate(loopRate);  //Specify loop rate/frequency in Hz
  // /ROS


  // Setup FT232H, exit if setup fails or no devices are found
  if(ftdi_setup() == EXIT_FAILURE){
    printf("FTDI setup failed. Exiting.\n");
    return 0;
  }


  int count = 0;
  while (ros::ok())
  {
    // Publish pin/port state
    // Single/multiple pins: read whole port, publish only advertised pins

    // Read port
    buf[0] = 0;
    if(ret = ftdi_read_pins(ftdi, buf) < 0){
      fprintf(stderr, "Reading from device failed: %d (%s)\n", ret, ftdi_get_error_string(ftdi));
      ftdi_free(ftdi);
      return 0;
    }

    for(int i=0; i<pins; i++){
      direction = (std::string)pinConfig[i]["direction"];
      if(direction == "input"){
        boolMsg.data = (buf[0] & (1 << i));
        pinPub[i].publish(boolMsg);
      }
    }

    // Publish whole IO port (if publisher was set up earlier)
    std_msgs::Byte byteMsg;
    byteMsg.data = buf[0];
    portPub.publish(byteMsg);


    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  ftdi_cleanup();

  return 0;
}


int ftdi_setup(void){
  // Create new FTDI context
  if ((ftdi = ftdi_new()) == 0){
    fprintf(stderr, "ftdi_new failed\n");
    return EXIT_FAILURE;
  }

  // Get version (check if library works)
  version = ftdi_get_library_version();
  printf("Initialized libftdi %s (major: %d, minor: %d, micro: %d, snapshot ver: %s)\n",
      version.version_str, version.major, version.minor, version.micro, version.snapshot_str);

  // Look for devices
  if((ret = ftdi_usb_find_all(ftdi, &devlist, 0, 0)) < 0){
    fprintf(stderr, "USB device discovery failed: %d (%s)\n", ret, ftdi_get_error_string(ftdi));
    ftdi_free(ftdi);
    return EXIT_FAILURE;
  }
  else if(ret > 0){
    printf("Discovered %d device(s).\n", ret);
  }
  else{
    printf("No devices found. Exiting.\n");
    ftdi_free(ftdi);
    return EXIT_FAILURE;
  }

  // Open specific device of multiple identical ones (specified by serial number)
  if ((ret = ftdi_usb_open_desc(ftdi, V_ID, P_ID, 0, serial_no)) < 0){
    fprintf(stderr, "Unable to open ftdi device: %d (%s)\n", ret, ftdi_get_error_string(ftdi));
    ftdi_free(ftdi);
    return EXIT_FAILURE;
  }
  printf("ftdi open succeeded: %d\n", ret);

  // Read out FTDIChip-ID
  if (ftdi->type == FTDI_CHIP_TYPE){
      unsigned int chipid;
      printf("ftdi_read_chipid: %d\n", ftdi_read_chipid(ftdi, &chipid));
      printf("FTDI chipid: %X\n", chipid);
  }
  else{
    printf("Chip type not recognised. Type is: %d\n", ftdi->type);
  }

  // Configure IO port
  for(int pin_no = 0; pin_no < pins; pin_no++){
    port_dir = port_dir | (pin_dir[pin_no] << pin_no) ;
  }
  printf("Port config: %0X\n", port_dir);

  printf("Enabling bitbang mode\n");
  ftdi_set_bitmode(ftdi, port_dir, BITMODE_BITBANG);

  buf[0] = 0;
  printf("Turning everything off\n");
  ret = ftdi_write_data(ftdi, buf, 1);
  if (ret < 0){
    fprintf(stderr,"Write failed for 0x%x, error %d (%s)\n",buf[0],ret, ftdi_get_error_string(ftdi));
  }

  return EXIT_SUCCESS;
}

int ftdi_cleanup(void){
  // Turn everything off
  buf[0] = 0;
  printf("Turning everything off\n");
  ret = ftdi_write_data(ftdi, buf, 1);
  if (ret < 0){
    fprintf(stderr,"Write failed for 0x%x, error %d (%s)\n",buf[0],ret, ftdi_get_error_string(ftdi));
  }

  // Cleanup
  // ftdi_disable_bitbang(ftdi);
  // Close device
  if ((ret = ftdi_usb_close(ftdi)) < 0){
      fprintf(stderr, "Unable to close ftdi device: %d (%s)\n", ret, ftdi_get_error_string(ftdi));
      ftdi_free(ftdi);
      return EXIT_FAILURE;
  }
  // Deinitialize and free FTDI context
  ftdi_free(ftdi);
}

void set_pin(const std_msgs::Bool::ConstPtr& msg, int index){
  // Change buffer
  if(msg->data){
    // Set bit
    buf[0] |= (1 << index);
  }
  else{
    // Reset bit
    buf[0] &= ~(1 << index);
  }
  ftdi_write_data(ftdi, buf, 1);
}


void set_pins(const std_msgs::Byte::ConstPtr& msg){
  ROS_INFO("I heard: [%d]", msg->data);

  // Set buffer
  // buf[0] = 0xFF;
  buf[0] = msg->data;
  // printf("turning everything on\n");
  printf("buffer: %d\n", buf[0]);
  // Commit buffer to device
  ret = ftdi_write_data(ftdi, buf, 1);
  if (ret < 0){
    fprintf(stderr,"write failed for 0x%x, error %d (%s)\n",buf[0],ret, ftdi_get_error_string(ftdi));
  }
}
// rostopic pub /pin_input_states std_msgs/Byte 0
