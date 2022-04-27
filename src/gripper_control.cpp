#include "dynamixel_sdk.h"
#include "gripper_control.h"
#include "utils.h"
#include <vector>
#include <math.h>
#include <unistd.h>

std::vector<float> linPot = {0,0};


// *******************************************************************************************//
// **************************** Gripper dynamixel functions **********************************//
// *******************************************************************************************//
GripperControl::GripperControl() {}

GripperControl::GripperControl(int baudrate, const char *device_name, float protocol_version, uint16_t addr_mx_torque_enable, uint16_t addr_mx_goal_position, uint16_t addr_mx_present_position)
{
    this->baudrate = baudrate;
    // this->device_name = device_name;
    this->addr_mx_torque_enable = addr_mx_torque_enable;
    this->addr_mx_goal_position = addr_mx_goal_position;
    this->addr_mx_present_position = addr_mx_present_position;
    this->dxl_resolution = 300 * M_PI/180 * 25.46e-3/2 /1024;       // multiply this value with the dynamixel encode value to get displacement
    
    // Initialize the PortHandler instance
    portHandler = dynamixel::PortHandler::getPortHandler(device_name);

    // Initialize PacketHandler instance
    packetHandler = dynamixel::PacketHandler::getPacketHandler(protocol_version);
    
}

GripperControl::~GripperControl()
{
  dxl_disable();
  portHandler->closePort();
}

// Get the number of connected dynamixels
int GripperControl::get_connected_dxl_count(std::vector<int> *dxl_id_list)
{
    int comm_success = 0;
    int dxl_count = 0;
    uint8_t dxl_error;
    uint16_t dxl_model_num;

    for (int id=1; id < 10; id++)
    {
      if (packetHandler->ping(portHandler, id, &dxl_model_num, &dxl_error)==COMM_SUCCESS)
      {
        dxl_count += 1;
        dxl_id_list->push_back(id);
      }
    }
    return dxl_count;
}

// Check the communication (packet transfer and receiving) status of the dynamixel
void GripperControl::dxl_comm_status(int dxl_comm_result, uint8_t dxl_error)
{
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf(RED "%s \n" DEFAULT, packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf(RED "%s \n" DEFAULT, packetHandler->getRxPacketError(dxl_error));
    }
}

// Connect to dynamixel, set the baudrate and enable torque
bool GripperControl::dxl_enable()
{
    // Open port
    if (!portHandler->openPort())
    {
        printf(RED "Failed to open the port! \n" DEFAULT);
        return false;
    }

    // Set port baudrate
    if (!portHandler->setBaudRate(this->baudrate))
    {
        printf(RED "Failed to change the baudrate! \n" DEFAULT);
        return false;
    }

    dxl_count = get_connected_dxl_count(&dxl_id_list);
    // this->dxl_id_list = dxl_id_list;

    uint8_t torque_enable = 1;
    int dxl_comm_result;
    uint8_t dxl_error;
    for (int dxl_id : dxl_id_list)
    {
        // Enable Dynamixel Torque
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id, addr_mx_torque_enable, torque_enable, &dxl_error);
        dxl_comm_status(dxl_comm_result, dxl_error);
    }

    printf(GREEN "Successfully connected to dynamixel! \n" DEFAULT);
    return true;
}

// disable the dynamixel
void GripperControl::dxl_disable()
{
    uint8_t torque_disable = 0;
    uint8_t dxl_error = 0;
    int dxl_comm_result;

    for (int dxl_id : dxl_id_list)
    {
      // Disable Dynamixel Torque
      dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id, addr_mx_torque_enable, torque_disable, &dxl_error);
      dxl_comm_status(dxl_comm_result, dxl_error);
    }
}

// set min and max position of a dynamixel motor in the gripper frame
void GripperControl::set_dxl_min_max(int dxl_id, uint16_t dxl_min, uint16_t dxl_max){
    
    // dxl id 1 is inner magnet
    if (dxl_id == 1)
    {
        dxl1_min = dxl_min;
        dxl1_max = dxl_max;

    }else if (dxl_id == 2){
        dxl2_min = dxl_min;
        dxl2_max = dxl_max;
    }
}

// get min and max position of a dynamixel motor in the gripper frame
void GripperControl::get_dxl_min_max(int dxl_id, std::vector<uint16_t> *dxl_limits){
    
    std::vector<uint16_t> limits{0,0};

    // dxl id 1 is inner magnet
    if (dxl_id == 1)
    {
        limits[0] = dxl1_min;
        limits[1] = dxl1_max;

    }else if (dxl_id == 2){
        limits[0] = dxl2_min;
        limits[1] = dxl2_max;
    }

    *dxl_limits = limits;
}

// convert dynamixel encoder value to position in gripper frame
float GripperControl::convert_dxl2pos(uint16_t dxl_value, uint16_t dxl_max, float offset)
{   
    return offset - (dxl_max - dxl_value) * dxl_resolution;
}

// convert position in gripper frame to dynamixel encoder value
uint16_t GripperControl::convert_pos2dxl(float pos, uint16_t dxl_max, float offset)
{   
    int temp_pos =  (offset - pos)/dxl_resolution;
    if (temp_pos > (int) dxl_max){
        temp_pos = (int) dxl_max;
    }
    return (uint16_t) ((int) dxl_max - temp_pos) ;
}

// calculate the mag to finger separation for each finger
std::vector<float> GripperControl::calculate_magsep(uint16_t dxl1_value, uint16_t dxl2_value, float finger_position, float offset1, float offset2)//,  std::vector<float>*mag_sep) std::vector<float> *pos, std::vector<float> *sep)
{
    float pos1 = convert_dxl2pos(dxl1_value, dxl1_max, offset1); // inner magnet position
    float pos2 = convert_dxl2pos(dxl2_value, dxl2_max, offset2); // outer magnet position
    
    std::vector<float> sep{0, 0};
    sep[0] = pos1 - finger_position; // inner magnet separation
    sep[1] = finger_position - pos2; // outer magnet separation 
    
    return sep;
}

// calculate dxl position from the provided magnet separation 
std::vector<uint16_t> GripperControl::calculate_dxlvalue(std::vector<float> sep, float finger_position, float offset1, float offset2){
    
    float pos1 = finger_position + sep[0]; // inner magnet position
    float pos2 = finger_position - sep[1]; // outer magnet position 
    
    std::vector<uint16_t> dxl_vec{0, 0};
    dxl_vec[0] = convert_pos2dxl(pos1, dxl1_max, offset1);
    dxl_vec[1] = convert_pos2dxl(pos2, dxl2_max, offset2);

    // std::cout << dxl_vec[1] << ", " << pos2 << ", " << dxl2_max << ", " << offset2 << ", " << std::endl;
    return dxl_vec;
}

// bool GripperControl::checkdxlpos(){

// }

// get position for the rf series
uint16_t GripperControl::get_dxl_pos(int dxl_id)
{
    uint8_t dxl_error;
    int dxl_comm_result;
    uint16_t dxl_pos;

    // Set dynamixel position
    dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, dxl_id, addr_mx_present_position, &dxl_pos, &dxl_error);
    
    // check if the dxl_pos is within safe operating limits
    if ((dxl_pos < 0) || (dxl_pos > 1023))
    {
        printf(RED "Error reading dynamixel value ! \n" DEFAULT);
        return 0;
    }

    return dxl_pos;
}

// set position for the rf series
void GripperControl::set_dxl_pos(int dxl_id, uint16_t dxl_value)
{
    uint8_t dxl_error;
    int dxl_comm_result;

    std::vector<uint16_t> dxl_limits{0, 0};
    
    get_dxl_min_max(dxl_id, &dxl_limits);

    // check if the dxl_value is within safe operating limits
    if (dxl_value < dxl_limits[0])
    {
        dxl_value = dxl_limits[0];
    }else if (dxl_value > dxl_limits[1])
    {
        dxl_value = dxl_limits[1];
    }

    // Set dynamixel position
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id, addr_mx_goal_position, dxl_value, &dxl_error);
    dxl_comm_status(dxl_comm_result, dxl_error);
}


// *******************************************************************************************//
// **************************** Phidget functions ********************************************//
// *******************************************************************************************//
// Store the value of the potentiometer into linPot
int CCONV SensorChangeHandler(CPhidgetInterfaceKitHandle IFK, void *usrptr, int Index, int Value)
{
    std::vector<float> potCenter = {455, 451}; // get the center position of the handle during calibration
    // std::vector<float> potCenter = {448, 448}; // get the center position of the handle during calibration
    
    if (Index < 2)
    {
        linPot[Index] = (Value - potCenter[Index])*200/1000/1e3; // dxl softpot strips are 20 cms long and their bit values are in the range [1, 1000]
        // printf("Index: %d, Pot value: %d, distance from center: %f \n", Index, Value, linPot[Index]);
    }
    return 0;
}

// error handler for phidget
int CCONV ErrorHandler(CPhidgetHandle IFK, void *userptr, int ErrorCode, const char *unknown)
{
    printf("Error handled. %d - %s", ErrorCode, unknown);
    return 0;
}

// Connect to phidget kit and detect if the sensor value is changed
bool GripperControl::phidget_enable()
{   
    int result, numSensors;
    const char *err;
    CPhidgetInterfaceKitHandle ifKit = 0;
    CPhidgetInterfaceKit_create(&ifKit);
    CPhidget_set_OnError_Handler((CPhidgetHandle)ifKit, ErrorHandler, NULL);
    CPhidgetInterfaceKit_set_OnSensorChange_Handler (ifKit, SensorChangeHandler, NULL);
    CPhidget_open((CPhidgetHandle)ifKit, -1);
    if((result = CPhidget_waitForAttachment((CPhidgetHandle)ifKit, 10000)))
    {
        CPhidget_getErrorDescription(result, &err);
        printf("Problem connecting to phidget: %s \n", err);
        return 0;
    }

    CPhidgetInterfaceKit_setRatiometric(ifKit, 0);
    CPhidgetInterfaceKit_getSensorCount(ifKit, &numSensors);
    
    // change the sensor change trigger for the connected potentiometers
    for(int Index = 0; Index < 8; Index++)
    {   
        CPhidgetInterfaceKit_setSensorChangeTrigger(ifKit, Index, 0);  //0 represents higher sensitivity
    }
    

    printf(GREEN "Successfully connected to Phidget with %d sensors \n" DEFAULT, numSensors);
    return 1;
}    

// *******************************************************************************************//
// **************************** GRipper utilities ********************************************//
// *******************************************************************************************//
// Magnet force model
float GripperControl::magForce(float s)
{
    float C1 = 27.34; // small magnet model
    float C2 = 168.15;
    return C1 * exp(-C2 * s);
}

// resultant force on the finger
float GripperControl::resFingerForce(std::vector<float> sep)
{
    if (sep[0] < 0) sep[0] = 0.0;
    if (sep[1] < 0) sep[1] = 0.0;

    return magForce(sep[1]) - magForce(sep[0]);
}

// resultant force on the finger
float GripperControl::finger_dynamic_force(float m, float b, float fs, float acc, float vel, std::vector<float> sep)
{
    float extern_force = m * acc + b * vel + fs * sign_fun(vel) - resFingerForce(sep);

    return extern_force;
}

// resultant force on the object
float GripperControl::resObjectForce(std::vector<float> fingersep1, std::vector<float> fingersep2)
{

}

// close the gripper with a user defined magnet separation aka finger stiffness
void GripperControl::closeGripper(std::vector<int> dxl_id_list, uint16_t pos1, uint16_t pos2){

}


// function to open or close the gripper
// opening gripper pos1: 200, pos2:10
// closing gripper pos1: 1000, pos2:400
// Please calibrate the dynamixel position in gripper frame to prevent any damage
void GripperControl::opencloseGripper(std::vector<int> dxl_id_list, uint16_t dxl1_value, uint16_t dxl2_value){
    set_dxl_pos(dxl_id_list[0], dxl1_value);
    usleep(0.1*1e6);

    set_dxl_pos(dxl_id_list[1], dxl2_value);
    usleep(0.5*1e6);

    // printf("finger1 pos: %f, finger2 pos: %f \n", linPot[0], linPot[1]);
}

// ------------------------------------------------------------------ //
// Impact adaptation strategies
// ------------------------------------------------------------------ //
// Set the active magnets to maximum posssible position to avoid impact transfer
// uint16_t impact_adaptation(uint16_t dxl_pos, uint16_t new_dxl_pos, float handle_position, float force_threshold)
// {
//     float res_force = abs(resForce(dxl_pos, handle_position));
//     if (res_force > force_threshold){
//             new_dxl_pos = dxl_max_pos - 25; // set the dynamixel to the fathest position minus 25 for the safety
//         }
//     return new_dxl_pos;
// }

// // detect if the force exceeds the threshold and change the disableTorque flag to true
// void detect_impact(uint16_t dxl_pos, float handle_position, float force_threshold, bool *disableTorque)
// {
//     float res_force = resForce(dxl_pos, handle_position);
//     printf("Handle position: %f, Resultant force: %f\n", linPot, res_force);

//     if (abs(res_force) > force_threshold){
//             *disableTorque = true;
//         }
// }

// // ------------------------------------------------------------------ //
// // Adaptation strategies
// // ------------------------------------------------------------------ //
// // The adaptation strategy will always try to move the magnet to the dxl_initial_pos once the external force is removed
// // ((1)) |F1 - F2|/K_virtual = x
// uint16_t spring_adaptation(uint16_t dxl_pos_t1, uint16_t dxl_pos_t, uint16_t new_dxl_pos, float handle_position, float K_virtual, float force_threshold)
// {
//     if (K_virtual < 1) K_virtual = 1; // prevent division by zero error

//     float res_force = abs(resForce(dxl_pos_t, handle_position));

//     if (res_force > force_threshold){
//         float estimated_pos = res_force / K_virtual ;
//         new_dxl_pos = dxl_pos_t1 +  estimated_pos / dxl_resolution;
//     }else{
//         new_dxl_pos = dxl_pos_t;
//     }

//     new_dxl_pos = set_safe_dxl_pos(new_dxl_pos);
//     return new_dxl_pos;
// }

// // Admittance adaptation strategy
// // ((2)) (|F1 - F2| - C_virtual * Vt)/M_virtual  * dt = Vt+1
// uint16_t admittance_adaptation(uint16_t dxl_pos_t1, uint16_t dxl_pos_t, float handle_position, float M_virtual, float C_virtual, float force_threshold)
// {
//     uint16_t new_dxl_pos;
//     float res_force;

//     res_force = abs(resForce(dxl_pos_t, handle_position));

//     if (res_force > force_threshold){
//         // float estimated_pos = (2 * res_force * dt * dt + 4*M_virtual * dxl_pos_t + (2*M_virtual - C_virtual*dt) * dxl_pos_t1) / (2*M_virtual + C_virtual * dt);
//         float estimated_pos = (2 * res_force * dt * dt) / (2*M_virtual + C_virtual * dt);

//         new_dxl_pos = dxl_pos_t + estimated_pos / dxl_resolution;
//     }else{
//         new_dxl_pos = dxl_pos_t;
//     }
//     printf("force:%f, current_pos:%d, new_pos:%d, handle_pos:%f\n", res_force, dxl_pos_t, new_dxl_pos, handle_position);
//     new_dxl_pos = set_safe_dxl_pos(new_dxl_pos);
//     return new_dxl_pos;
// }
