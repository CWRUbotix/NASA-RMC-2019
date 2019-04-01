#include <HardwareControlInterface.h>

map<uint8_t, float> motorValues;
ros::Publisher sensorPublisher;
ros::Subscriber motorSubscriber;

serial::Serial hcSerial;
unsigned long baud = 115200; 
string hcDescription = "Teensyduino USB Serial 4822650"; //description of the HCb

uint8_t setOutputsByte = 0x51;
uint8_t readValuesByte = 0x52;
uint8_t testByte = 0x53;
uint8_t syncTimeByte = 0x54;

bool addMotorValue(int ID, float value){
    //we don't care if this overwrites an existing pair
    uint8_t smallerID = ID;
    motorValues.erase(smallerID);
    motorValues.insert(std::pair<uint8_t,float>(smallerID,value));
    return true;
}

void addMotorCallback(const hci::motorCommand& msg){
    addMotorValue(msg.motorID, msg.value);
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


vector<uint8_t> generateMotorCommandMessage(void){
    vector<uint8_t> commandMessage;
    commandMessage.push_back(setOutputsByte);
    uint16_t checksum = 0;
    for(map<uint8_t,float>::iterator it = motorValues.begin(); it != motorValues.end(); it++) {

        commandMessage.push_back(it->first);

        char value[sizeof(float)];
        float f = it->second;
        memcpy(value, &f, sizeof f);    //transfer the float value into a char array

        commandMessage.push_back(value[0]);
        commandMessage.push_back(value[1]);
        commandMessage.push_back(value[2]);
        commandMessage.push_back(value[3]);

        checksum += (uint16_t)(0x00ff & it->first);
        checksum += (uint16_t)(0x00ff & value[0]);
        checksum += (uint16_t)(0x00ff & value[1]);
        checksum += (uint16_t)(0x00ff & value[2]);
        checksum += (uint16_t)(0x00ff & value[3]);
    }   

    vector<uint8_t>::iterator it = commandMessage.begin();
    it++; //now points to position 1
    uint16_t length = commandMessage.size() - 1;
    it = commandMessage.insert(it, checksum);
    it = commandMessage.insert(it, checksum >> 8);
    it = commandMessage.insert(it, length);
    it = commandMessage.insert(it, length >> 8);
    
    return commandMessage;

}

int main(int argc, char** argv) {

    ros::init(argc, argv, "hci");
    ros::NodeHandle n; 
    sensorPublisher = n.advertise<hci::sensorValue>("sensorValue", 1);
    motorSubscriber = n.subscribe("motorCommand",1,addMotorCallback); 

    string port = "0";
    while(port == "0"){
        port = findHardwareControllerPort();
        ros::Duration(1).sleep();
    } 
    ROS_INFO("%s\n ", port.c_str());

    while(ros::ok()){
        if(!hcSerial.isOpen()){
            while(!hcSerial.isOpen()){
                try
                {
                    hcSerial.setPort(port);
                    hcSerial.setBaudrate(baud);
                    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
                    hcSerial.setTimeout(to);
                    hcSerial.open();
                }
                catch (serial::IOException& e)
                {
                    ROS_ERROR("Unable to open port ");
                    ROS_ERROR("%s", e.what());
                }
                ros::Duration(0.25).sleep();
                ros::spinOnce();
            }
            ROS_INFO("Serial port opened");
        }
        vector<uint8_t> motorCommandMessage = generateMotorCommandMessage();
        ROS_INFO("NEW MESSAGE");
        for (std::vector<uint8_t>::const_iterator i = motorCommandMessage.begin(); i != motorCommandMessage.end(); ++i){
            ROS_INFO("%u", *i);
        }

        hcSerial.write(motorCommandMessage);
        ros::Duration(1).sleep();
        ros::spinOnce();

    }
    
    ros::spin();

    return 0;
}
