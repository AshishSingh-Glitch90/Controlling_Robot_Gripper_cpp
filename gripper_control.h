/*******************************************************************************
* Dynamixel control functions for gripper using DynamixelSDK
* Created on: 01/28/2022
* Author : Sri Sadhan
*******************************************************************************/

#include "dynamixel_sdk.h"
#include <phidget21.h>

extern std::vector<float> linPot;

class GripperControl{
    public:
        GripperControl();
        GripperControl(int baudrate, const char *device_name, float protocol_version, uint16_t addr_mx_torque_enable, uint16_t addr_mx_goal_position, uint16_t addr_mx_present_position);
        
        ~GripperControl();
        
        void dxl_comm_status(int dxl_comm_result, uint8_t dxl_error);
        bool dxl_enable();
        void dxl_disable();
        int get_connected_dxl_count(std::vector<int> *dxl_id_list);

        float convert_dxl2pos(uint16_t dxl_value, uint16_t dxl_max, float offset);
        uint16_t convert_pos2dxl(float pos, uint16_t dxl_max, float offset);
        std::vector<float> calculate_magsep(uint16_t dxl1_value, uint16_t dxl2_value, float finger_position, float offset1, float offset2); //, std::vector<float> *mag_sep); 
        std::vector<uint16_t>  calculate_dxlvalue(std::vector<float> sep, float finger_position, float offset1, float offset2);

        uint16_t get_dxl_pos(int dxl_id);
        void set_dxl_pos(int dxl_id, uint16_t dxl_value);
        void set_dxl_min_max(int dxl_id, uint16_t dxl_min, uint16_t dxl_max);
        void get_dxl_min_max(int dxl_id, std::vector<uint16_t> *dxl_limits);

        bool phidget_enable();
        float magForce(float s);
        float resFingerForce(std::vector<float> sep);
        float resObjectForce(std::vector<float> fingersep1, std::vector<float> fingersep2);
        float finger_dynamic_force(float m, float b, float fs, float acc, float vel, std::vector<float> sep);

        void opencloseGripper(std::vector<int> dxl_id_list, uint16_t pos1, uint16_t pos2);
        void closeGripper(std::vector<int> dxl_id_list, uint16_t pos1, uint16_t pos2);


    private:
        int baudrate;
        int dxl_count;
        double dxl_resolution;
        std::vector<int> dxl_id_list; 
        // const char *device_name;


        uint16_t addr_mx_torque_enable;
        uint16_t addr_mx_goal_position;
        uint16_t addr_mx_present_position;

        uint16_t dxl1_min;
        uint16_t dxl1_max;
        uint16_t dxl2_min;
        uint16_t dxl2_max;
        
        dynamixel::PortHandler *portHandler;
        dynamixel::PacketHandler *packetHandler;

        float pos; // magnet position 
        std::vector<float> sep;
};

int CCONV SensorChangeHandler(CPhidgetInterfaceKitHandle IFK, void *usrptr, int Index, int Value);
int CCONV ErrorHandler(CPhidgetHandle IFK, void *userptr, int ErrorCode, const char *unknown);
