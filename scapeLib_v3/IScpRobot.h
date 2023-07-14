/* interface for scape robot 2022-05-10-1*/
#ifndef _IScpRobot__H_

#define _IScpRobot__H_

// YASKAWA ROBOT specific head file "motoPlus.h"
#include "motoPlus.h"

// IMPORTANT NOTE ABOUT TOOL FRAME AND BASE FRAME:
// This libary always refer a consistent Tool frame and Base frame !!!
// It is up to the user to keep the activated Tool frame and Base is same while execute scape tasks!!!

/*
_dbg_SCAPE_ = 0 silence the debug output message
_dbg_SCAPE_ = 1 TCP/IP communication 
_dbg_SCAPE_ = 2    scape_os_core
_dbg_SCAPE_ = 3    scape_bin_picking
_dbg_SCAPE_ = 9 all
*/
#define _dbg_SCAPE_ 0
#define BUFF_LEN 255
#define MAX_GRIPPER_NUM 3
#define MAX_PRODUCT_NUM_IN_GROUP 4
#define MAX_SCAPE_TASKS_LEN 15
#define SCAPE_ROBOT_AXIS_NUM 6
// 'JOB_' user prepared small program, for example:
// a customized gripper control
// a teached path move robot from one end to another
// a specific code to check if gripper has part in hand or not
// usually these part of code reside on teach pendant, so user can edit it on TP.
// JOB ID 0 ~ 99: SCAPE STATUS, see SCAPE CALIBRATION MANAGER for more.
// JOB ID 100 ~ 199: SCAPE external routines, see SCAPE CALIBRATION MANAGER for more.
// JOB ID >= 200 user teached path, or SCAPE UPDATE IO routines.
#define JOB_HOMING 100
#define JOB_TEACH 101
#define JOB_GET_TCP_POSE 111
#define JOB_GET_JOINTS 112
#define JOB_MOVE_BIN_TO_HS 221
#define JOB_MOVE_HS_TO_BIN 222
#define JOB_MOV_CLEAR_HS 223
#define JOB_LIGHT_ON 211
#define JOB_PATTERN_ON 212
#define JOB_EMPTY_HS 213
#define JOB_CHECK_GRIP 210
#define JOB_EXIT 255
// 'RC_' scape pick return value
#define RC_BIN_FINISH -1
#define RC_LAYER_FINISH -2
#define RC_UNPICKABLE_PART_IN_BIN -3
#define RC_UNPICKABLE_PART_IN_LAYER -4
#define RC_SOMETHING_IN_BIN -5
#define RC_SOMETHING_IN_LAYER -6
#define RC_OC_FAILED -7



typedef enum {NotMov = 0, MOVEJ = 1,MOVEL = 2} MotionType;

typedef enum {BP_AND_OC = 0,BP_ONLY = 1,OC_ONLY = 2,BP_AND_PLACE_ON_HS = 3}PickCfg;

typedef enum {Info,Warning,Error}MsgLevel;

typedef enum {Started,AllFinished,SliderColision,FlangeCollision}TaskRunStatus;

typedef struct _Bin
{
    unsigned int product_id;

    unsigned int product_group_id;

    unsigned int start_height_mm;

    unsigned int remain_parts_height_mm;

    unsigned int picked_parts_count;

    int bin_status;
}Bin;

// scape tasks
typedef struct _ScapeTask
{
    int taskIndex;
    
    int taskNumInTotal;
    
    int motionValid;
    
    MotionType motionType;
    
    double target[6];
    
    double joints[SCAPE_ROBOT_AXIS_NUM];
    
    int speed;
    
    int acc;
    
    int blend;

    int logicValid;
    
    int job_id;
    
    int par0;
    
    int par1;
    
    int par2;
    
    int stateWaitMotionStart;
}ScapeTask;

// a robot must support below methods to go with SCAPE system
typedef struct _Robot
{
    // hold program execution for seconds
    void (*fnWaitSec)(float seconds);

    // connect to scape server
    void (*fnConnectToScape)(void);

    // disconnect from scape server
    void (*fnDisconnectToScape)(void);

    // send a string to scape server
    void (*fnSendToScape)(const char[BUFF_LEN]);

    // receive a string from scape server
    void (*fnReceiveFromScape)(char[BUFF_LEN]);

    // popup a message from scape
    void (*fnShowMsg)(char[BUFF_LEN],MsgLevel);

    // clean message screen
    void (*fnCleanMsg)(void);

    // execution scape task
    void (*fnRunScapeTask)(ScapeTask*);
    
    // hold until all scape sent task were executed, or aborted
    void (*fnWaitScapeTaskComplete)(TaskRunStatus*);

    // robot best axis configuration durning bin-picking
    double jGlobalBestConfig[SCAPE_ROBOT_AXIS_NUM];

}Robot;

// scape output methods
// these methods can be used to configure bin-picking loop, depend on the needs
// details see example code.
typedef struct Scp
{
    // scape calibration
    int (*scp_calibration)(void);

    // initialize
    int (*scp_initialize)(unsigned int product_group_id, unsigned int product_number_in_group);
    
    // pick one part from specificed bin or handling station, rc >= 0 picking succeed, rc == grip family id
    int (*scp_pick)(Bin* bin,short rescan,PickCfg pickCfg);
    
    // start acquisition of specificed bin
    int (*scp_start_scan)(Bin* bin);

    // start handling station recognition, valid only stationary camera was used.
    int (*scp_start_handling_station_recog)(Bin* bin);

    int (*scp_place_on_handling_station)(Bin* bin);

    int (*scp_regrip_at_handling_station)(Bin* bin);

    int (*scp_check_oc_result)(Bin* bin);

    int (*scp_pick_3D)(Bin* bin);

    int (*scp_scan_3D)(Bin* bin);

    int (*scp_scan_2D)(Bin* bin);
}IScp;

int init_robot(Robot* robot,IScp* scp);

#endif