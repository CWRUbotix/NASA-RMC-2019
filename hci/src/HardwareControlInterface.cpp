#include <HardwareControlInterface.h>

map<int, float> motorValues;
ros::Publisher sensorPublisher;
ros::ServiceServer motorService;

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

void enumerate_ports(void){
    vector<serial::PortInfo> devices_found = serial::list_ports();
    vector<serial::PortInfo>::iterator iter = devices_found.begin();
    while( iter != devices_found.end() )
    {
        serial::PortInfo device = *iter++;
        ROS_INFO( "(%s, %s, %s)\n", device.port.c_str(), device.description.c_str(),
            device.hardware_id.c_str() );
    }
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "hci");
    ros::NodeHandle n; 
    sensorPublisher = n.advertise<hci::sensorValue>("sensorValue", 1);
    motorService = n.advertiseService("motorCommand", addMotorCallback);

    enumerate_ports();
    ros::spin();

    return 0;
}
