#include <HardwareControlInterface.h>

map<int, float> motorValues;
ros::Publisher sensorPublisher;
ros::ServiceServer motorService;

serial::Serial hcSerial;
unsigned long baud = 5600; //i don't remember what the actual baud rate is
string hcDescription = "CWRUBOTIX CONTROLLER"; //change when i find out what it actually is called

uint8_t setOutputsByte = 0x51;
uint8_t readValuesByte = 0x52;
uint8_t testByte = 0x53;
uint8_t syncTimeByte = 0x54;


bool addMotorValue(int ID, float value){
    //we don't care if this overwrites an existing pair
    motorValues.erase(ID);
    motorValues.insert(std::pair<int,float>(ID,value));
    return true;
}

bool addMotorCallback(hci::motorCommand::Request &request, hci::motorCommand::Response &response){
    addMotorValue(request.motorID, request.value);
    response.success = true;
    return true;
}

void enumeratePorts(void){
    vector<serial::PortInfo> devices_found = serial::list_ports();
    vector<serial::PortInfo>::iterator iter = devices_found.begin();
    while( iter != devices_found.end() )
    {
        serial::PortInfo device = *iter++;
        ROS_INFO( "(%s, %s, %s)\n", device.port.c_str(), device.description.c_str(), device.hardware_id.c_str() );
    }
}

string findHardwareControllerPort(void){
    vector<serial::PortInfo> devices_found = serial::list_ports();
    vector<serial::PortInfo>::iterator iter = devices_found.begin();
    while( iter != devices_found.end() )
    {
        serial::PortInfo device = *iter++;
        ROS_INFO( "(%s, %s, %s)\n", device.port.c_str(), device.description.c_str(), device.hardware_id.c_str() );
        if(string(device.description.c_str()).compare(hcDescription) == 0){
            return device.port.c_str();
        }
    }

    ROS_WARN("Didn't find the Hardware Controller board!");
    return "0";
}




int main(int argc, char** argv) {

    ros::init(argc, argv, "hci");
    ros::NodeHandle n; 
    sensorPublisher = n.advertise<hci::sensorValue>("sensorValue", 1);
    motorService = n.advertiseService("motorCommand", addMotorCallback);

    string port = "0";
    while(port == "0"){
        port = findHardwareControllerPort();
        ros::Duration(1).sleep();
    } 

    while(ros::ok()){
        try
        {
            hcSerial.setPort(port);
            hcSerial.setBaud(baud);
            hcSerial.setTimeout(serial::Timeout::simpleTimeout(1000));
            hcSerial.open();
        }
        catch (serial::IOException& e)
        {
            ROS_ERROR("Unable to open port ");
        }

        if(hcSerial.isOpen()){
            ROS_INFO("Serial port opened");
            break;
        }
    }


    

    enumeratePorts();
    ros::spin();

    return 0;
}
