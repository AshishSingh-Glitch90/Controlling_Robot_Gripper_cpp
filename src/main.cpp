#include <iostream>
#include <fstream>
#include "gripper_control.h"
#include "utils.h"
#include <unistd.h>
#include <chrono>

// boost headers
#include <boost/thread.hpp>

// Replace the control table parameters according to the dynamixel model
// These parameters are for the new dynamixel motors (XM430-W210R) used in the VSM
#define ADDR_RX_TORQUE_ENABLE           24                  // Control table address is different in Dynamixel model
#define ADDR_RX_GOAL_POSITION           30
#define ADDR_RX_PRESENT_POSITION        36

// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL_ID1                         1                   // Dynamixel ID: 1
#define DXL_ID2                         2                   // Dynamixel ID: 2
#define BAUDRATE                        1000000
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
#define DXL_MOVING_STATUS_THRESHOLD     20


uint16_t DXL_MIN_POS = 0;       // Min position of the dynamixel in the gripper
uint16_t DXL_MAX_POS = 1023;    // Max position of the dynamixel in the gripper

std::vector<uint32_t> dxl_pos(2, 0);        // encoder reading of each dynamixel 
// std::vector<float> linPot{0, 0};            // potentiometer reading in m from the equilibrium postion
std::vector<float> vsm_force(2, 0);         // resultant force on each degree of freedom (vsm_force[i] - ith DoF)
std::vector<std::vector<float>> mag_sep(2, std::vector<float> (2, 0));            // magnet separation matrix

// stops the program
void stop(bool* flag){
    char in;
    std::cin.get(in);
    *flag = true;
}

// calculate the velocity of both the fingers using finite difference 
std::vector<float> calculate_velocity(float dt, std::vector<float> finger_pos_t, std::vector<float> finger_pos_t1){
    std::vector<float> vel{0.0f,0.0f};

    vel[0] = (finger_pos_t[0] - finger_pos_t1[1]) / dt;
    vel[1] = (finger_pos_t[1] - finger_pos_t1[1]) / dt;

    return vel;
}

// calculate the acceleration of both the fingers using finite difference 
std::vector<float> calculate_acceleration(float dt, std::vector<float> finger_vel_t, std::vector<float> finger_vel_t1){
    std::vector<float> acc{0.0f,0.0f};

    acc[0] = (finger_vel_t[0] - finger_vel_t1[1]) / dt;
    acc[1] = (finger_vel_t[1] - finger_vel_t1[1]) / dt;

    return acc;
}

int main(int argc, char** argv){

    std::stringstream strValue, strValue1;
    float s = 0.010; // default value is 2 cm
    int sponge_id = 1;

    if (argc > 1){
        if(strcmp(argv[1], "-h") == 0) printf(GREEN "the additional arguments are \n -s : separation values \n -f : filename index \n examples: \n sudo ./gripper_test (default sep = 0.01 cm) \n sudo ./gripper_test -s 0.05 (change the separation value per convenience) \n" DEFAULT);
        if(strcmp(argv[1], "-s") == 0)
        {
            strValue << argv[2];
            strValue >> s;
        }
        if(argc > 3){
            if(strcmp(argv[3], "-f") == 0)
            {
            strValue1 << argv[4];
            strValue1 >> sponge_id;
        }

        }
    }
    std::vector<float> req_sep{s, s};
    float req_finger_pos = -0.03f;

    std::ofstream datafile;
    datafile.open("../data/sponge_stiffness/sponge_" + std::to_string(sponge_id) + ".csv");
    datafile << "finger1, finger2, force1, force2\n";

    std::cout << "Set the position to dynamixel" << std::endl;
    GripperControl gripper(BAUDRATE, DEVICENAME, PROTOCOL_VERSION, ADDR_RX_TORQUE_ENABLE, ADDR_RX_GOAL_POSITION, ADDR_RX_PRESENT_POSITION);
    std::vector<int> dxl_id_list;

    int dxl_comm_result = COMM_TX_FAIL;                             // Communication result
    uint8_t dxl_error = 0;                                          // Dynamixel error

    // connect to the available dynamixel
    if (!gripper.dxl_enable()) return 0;
    printf("Number of dyn connected: %d \n", gripper.get_connected_dxl_count(&dxl_id_list));

    if (!gripper.phidget_enable()) return 0;
    printf("Successfully connected to phidget interface \n");

    // define the min and max of each dynamixel in the gripper frame
    gripper.set_dxl_min_max(dxl_id_list[0], 100, 1023);
    gripper.set_dxl_min_max(dxl_id_list[1], 0, 580);

    bool terminate_code = false;
    boost::thread stop_thread(stop, &terminate_code);
    ///////////////////////////////////////////////
    // dxl-id1: Inner magnets of the gripper; 
    //          position limits: [100, 1023] 
    //          offset: -23 mm at 1023 between finger inner magnet to inner active magnet surface
    // dxl-id2: Outer magnets of the gripper; position limits: [0, 600] 
    //          position limits: [0, 600] 
    //          offset: 0 mm at 600 dxl between finger outer magnet to outer active magnet surface
    // 21.6 mm is the finger width
    //////////////////////////////////////////////
    float offset1 = 0.023;
    float offset2 = 0.000;

    // Close Gripper
    // gripper.opencloseGripper(dxl_id_list, 900, 500);
    // usleep(5e6);
    // Open Gripper
    // gripper.opencloseGripper(dxl_id_list, 200, 10);
    // std::cout << dxl_id_list[0] << std::endl;
    // uint16_t dxl1_value = gripper.get_dxl_pos(dxl_id_list[0]);
    // uint16_t dxl2_value = gripper.get_dxl_pos(dxl_id_list[1]);
    
    // printf("dxl1: %d, dxl2: %d , pos1: %f, finger1: %f, finger2: %f, pos2: %f \n", dxl1_value, dxl2_value, gripper.convert_dxl2pos(dxl1_value, 1023, offset1), linPot[0], linPot[1], gripper.convert_dxl2pos(dxl2_value, 600, offset2));

    // std::vector<float> sep = gripper.calculate_magsep(dxl1_value, dxl2_value, linPot[0], offset1, offset2);
    // printf("Separation between OuterMag and finger1: %f, InnerMag and finger1: %f \n", sep[1], sep[0]);
    
    // sep = gripper.calculate_magsep(dxl1_value, dxl2_value, linPot[1], offset1, offset2);
    // printf("Separation between OuterMag and finger2: %f, InnerMag and finger2: %f \n", sep[1], sep[0]);

    gripper.set_dxl_pos(dxl_id_list[0], gripper.calculate_dxlvalue(req_sep, req_finger_pos, offset1, offset2)[0]);
    gripper.set_dxl_pos(dxl_id_list[1], gripper.calculate_dxlvalue(req_sep, req_finger_pos, offset1, offset2)[1]);

    usleep(1e6);
    // // Close Gripper
    // gripper.opencloseGripper(dxl_id_list, 1023, 550);
    

    float dt = 0.005f;
    std::chrono::time_point<std::chrono::system_clock> timeLoop;
    std::chrono::duration<float> elaps_loop;


    std::vector<float> act_sep1{0.02f, 0.02f};
    std::vector<float> act_sep2{0.02f, 0.02f};
    std::vector<uint16_t> req_dxl_value{200, 10};
    std::vector<uint16_t> act_dxl_value{200, 10};
    float finger1_force = 0.0f;
    float finger2_force = 0.0f;
    float finger1_dyn_force = 0.0f;
    float finger2_dyn_force = 0.0f;
    
    std::vector<float> acc{0.0f, 0.0f};
    std::vector<float> vel{0.0f, 0.0f};

    std::vector<float> finger_pos_t1{0.0f, 0.0f};
    std::vector<float> finger_pos_t{0.0f, 0.0f};    
    std::vector<float> finger_vel_t1{0.0f, 0.0f};
    std::vector<float> finger_vel_t{0.0f, 0.0f};    
    
    // force threshold on each finger to detect object
    float force_threshold = 5.0;

    // ######################################################## //
    // Determines if an object is placed in between the fingers
    // ######################################################## //
    while((!terminate_code) && (abs(finger1_force) < force_threshold) && (abs(finger2_force) < force_threshold)){
        timeLoop = std::chrono::system_clock::now();
        // std::cout << req_sep[0] << ", " << req_sep[1] << std::endl;
        // only run the code if the desired finger position is higher than zero
        req_dxl_value = gripper.calculate_dxlvalue(req_sep, req_finger_pos, offset1, offset2);
        // printf("dxl1 pos: %u, dxl2 pos: %u \n", req_dxl_value[0], req_dxl_value[1]);

        req_finger_pos += 0.0005;

        gripper.set_dxl_pos(dxl_id_list[0], req_dxl_value[0]);
        gripper.set_dxl_pos(dxl_id_list[1], req_dxl_value[1]);

        act_dxl_value[0] = gripper.get_dxl_pos(dxl_id_list[0]);
        act_dxl_value[1] = gripper.get_dxl_pos(dxl_id_list[1]);
        // printf("Read dynamixel pos 1: %u, 2:%u \n", act_dxl_value[0], act_dxl_value[1]);

        // update the finger position 
        for(int id=0; id < 2; id++){finger_pos_t[id]  = linPot[id];}
        // calculate the velocity of the fingers
        vel = calculate_velocity(dt, finger_pos_t, finger_pos_t1);
        // printf("Finger pos at t-1: %f, t: %f \n", finger_pos_t1[0], finger_pos_t[0]);

        for(int id=0; id < 2; id++){finger_vel_t[id]  = vel[id];}
        acc = calculate_acceleration(dt, finger_vel_t, finger_vel_t1);
        
        // printf("Velocity [0]: %f, [1]: %f;   Acceleration: [0]: %f, [1]: %f \n", vel[0], vel[1], acc[0], acc[1]);
        
        act_sep1 = gripper.calculate_magsep(act_dxl_value[0], act_dxl_value[1], linPot[0], offset1, offset2);
        finger1_force = gripper.resFingerForce(act_sep1);
        finger1_dyn_force = gripper.finger_dynamic_force(0.04f, 0.5, 0.5, acc[0], vel[0], act_sep1);
        // printf("Static force of finger 1: %f, dynamic force: %f \n",finger1_force, finger1_dyn_force);
        // std::cout << "sep:" << act_sep1[0] << ", " << act_sep1[1] << ", finger1: " << linPot[0] <<" at dxl: " << act_dxl_value[0] << ", " << act_dxl_value[1] << std::endl;

        act_sep2 = gripper.calculate_magsep(act_dxl_value[0], act_dxl_value[1], linPot[1], offset1, offset2);
        finger2_force = gripper.resFingerForce(act_sep2);
        finger2_dyn_force = gripper.finger_dynamic_force(0.04f, 0.5, 0.5, acc[1], vel[1], act_sep2);
        // printf("Static force of finger 2: %f, dynamic force: %f \n",finger2_force, finger2_dyn_force);
        // std::cout << "sep:" << act_sep2[0] << ", " << act_sep2[1] << ", finger2: " << linPot[1] << " at dxl: " << act_dxl_value[0] << ", " << act_dxl_value[1] << std::endl;

        
        for(int id=0; id < 2; id++){
            // update the positions at time t-1 for both the fingers 
            finger_pos_t1[id] = finger_pos_t[id];
            // update the velocities at time t-1 for both the fingers
            finger_vel_t1[id] = finger_vel_t[id];
        }

        printf("force on finger1: %f, finger2:%f \n", finger1_force, finger2_force);

        elaps_loop = std::chrono::system_clock::now() - timeLoop;
        if ((dt-elaps_loop.count()) > 0){
            usleep( (dt-elaps_loop.count())*1000*1000);
        }
    }
    printf(GREEN "######Object detected###### \n" DEFAULT);

    usleep(2e6);
    // ######################################################## //
    // Compresses the object to detect stiffness
    // ######################################################## //

    // force threshold to compress 
    force_threshold = 12.0;

    while((!terminate_code) && (abs(finger1_force) < force_threshold) && (abs(finger2_force) < force_threshold)){
        timeLoop = std::chrono::system_clock::now();
        
        // only run the code if the desired finger position is higher than zero
        req_dxl_value = gripper.calculate_dxlvalue(req_sep, req_finger_pos, offset1, offset2);
        
        req_finger_pos += 0.0001;

        gripper.set_dxl_pos(dxl_id_list[0], req_dxl_value[0]);
        gripper.set_dxl_pos(dxl_id_list[1], req_dxl_value[1]);

        act_dxl_value[0] = gripper.get_dxl_pos(dxl_id_list[0]);
        act_dxl_value[1] = gripper.get_dxl_pos(dxl_id_list[1]);
        
        act_sep1 = gripper.calculate_magsep(act_dxl_value[0], act_dxl_value[1], linPot[0], offset1, offset2);
        finger1_force = gripper.resFingerForce(act_sep1);
        
        act_sep2 = gripper.calculate_magsep(act_dxl_value[0], act_dxl_value[1], linPot[1], offset1, offset2);
        finger2_force = gripper.resFingerForce(act_sep2);
    
        printf("force on finger1: %f, finger2:%f \n", finger1_force, finger2_force);

        datafile << abs(linPot[0]) << "," << abs(linPot[1]) << "," << abs(finger1_force) << "," << abs(finger2_force) << "\n";
        elaps_loop = std::chrono::system_clock::now() - timeLoop;
        if ((dt-elaps_loop.count()) > 0){
            usleep( (dt-elaps_loop.count())*1000*1000);
        }
    }

    while(!terminate_code){

    }

    gripper.dxl_disable();
    stop_thread.interrupt();
    datafile.close();

    return 1;
}

/*
*------- Read file ----------*
cout<< "Reading file...."<<endl;
std::vector< std::vector<double> > matrix;
inputFile("hammer_traj/"+filename + ".txt", &matrix);

int traj_col = 0;
int  ncols, nrows=0;
for (std::vector< std::vector<double> >::const_iterator it = matrix.begin(); it != matrix.end(); ++ it)
{
    nrows++;
    ncols = 0;
    for (std::vector<double>::const_iterator itit = it->begin(); itit != it->end(); ++ itit)
    {
        ncols++;
    }
}
cout << "size of imported matrix = " << nrows << "*" << ncols << endl;
TooN::Matrix<Dynamic,Dynamic,double> imported_traj(nrows, ncols);

// first 6 cols for the joint angles and the 7th col for the magnet position
if (ncols == 7){

    // Put the matrix into TooN matrix
    nrows = 0;  ncols = 0;
    for (std::vector< std::vector<double> >::const_iterator it = matrix.begin(); it != matrix.end(); ++ it)
    {
        ncols = 0;
        for (std::vector<double>::const_iterator itit = it->begin(); itit != it->end(); ++ itit)
        {
            imported_traj(nrows,ncols) = *itit;
            ncols++;
        }
        nrows++;
        traj_col = ncols;
    }
}else{
    cout<< RED <<"Inconsistent matrix size for trajectory generation" << DEFAULT << endl;
    return 0;
}
cout << "File read succesful" <<endl;
*/