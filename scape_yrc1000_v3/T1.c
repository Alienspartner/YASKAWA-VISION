
#include "T.h"
#include "IScpRobot.h"
// #include "stdbool.h"


#define DBG_YRC 0
#define ROBOT_TASK_BUFF 2 /* 2 at most */


#define PICK_CFG_B_59 59
#define EXECUTION_STATUS_B_60 60
#define CHOICE_B_61 61
#define GROUP_ID_B_62 62
#define PRODUCT_ID_B_63 63
#define PRODUCTS_NUM_B_64 64
#define NEED_RESCAN_B_65 65
#define GRIPPER1_STATUS_B_66 66
#define GRIPPER2_STATUS_B_67 67
#define GRIPPER3_STATUS_B_68 68
#define ALL_TASK_DONE_B_69 69
#define ROBOT_IN_POSE_B_70 70
#define WAIT_USER_CHOICE_B_71 71
#define PICK_RETURN_I_65 65
#define BIN_HEIGHT_RETURN_I_66 66
#define T1_RUN_STATUS_S_51 51
#define SCP_MSG_S_54 54
#define T1_ERROR_S_55 55

#define USER_PORT_I_91 91
#define USER_TOOL_I_92 92
#define USER_BASE_I_93 93
#define USER_IP_S_91 91
#define ROBOT_IN_CALIB 0
#define ROBOT_IN_BIN_PICKING 1

typedef enum {CALIBRATION_10 = 10,BP_INITIALIZE_20 = 20, SCAPE_PICK_30 = 30, START_BIN_SCAN_40 = 40,START_HS_RECOG_50 = 50}Choice;
typedef struct _YRC_TASK
{
    // general
    unsigned int b_task_flag_idx;
    unsigned int b_task_index_idx;
    unsigned int b_task_num_idx;
    // motion
    unsigned int b_motionType_idx;
    unsigned int p_pose_idx;
    unsigned int i_speed_idx;
    unsigned int i_acc_idx;
    unsigned int i_blend_idx;
    // logic
    unsigned int i_job_id_idx;
    unsigned int i_par0_idx;
    unsigned int i_par1_idx;
    unsigned int i_par2_idx;
}YRC_TASK;

const YRC_TASK yrc_task[2] = {
{51,53,55,57,51,51,53,55,57,59,61,63},
{52,54,56,58,52,52,54,56,58,60,62,64}
};

static short robot_current_working = ROBOT_IN_CALIB;
static IScp* scp = NULL;
static Robot* yrc = NULL;
static char scp_ip[16] = "192.168.80.124";
static int scp_port = 14001;
static int SCAPE_TOOL_INDEX = 1;
static int SCAPE_BASE_INDEX = 0;
static double jGlobalCfg[SCAPE_ROBOT_AXIS_NUM] = {0,0,0,0,-90,0};
static int sockHandle = -9999;
static void t1_log(char*);
static void put_s_val(char* msg,unsigned short idx);
Bin bins[MAX_PRODUCT_NUM_IN_GROUP] = {};
extern TASK_STATUS T1_STATUS;
extern short T2_RESET_T1;
extern int T1_ID;
extern int T2_ID;
int current_task_idx = 0, current_tasks_num = 0;
int last_pick = 0;
int last_result = 1;

static void waitSec(float sec)
{
    mpTaskDelay((int)(1000 * sec/mpGetRtc()));
}

static void put_b_val(unsigned short var,unsigned char idx)
{
    MP_USR_VAR_INFO varInfo;
    
    memset(&varInfo, 0, sizeof(varInfo));
    varInfo.var_type = MP_VAR_B;
    varInfo.var_no = idx;
    varInfo.val.b = var;
    while(!T2_RESET_T1)
    {
        if(mpPutUserVars(&varInfo) == 0) break;
        waitSec(0.01);
    }
}

static void end_task1(void)
{
    put_s_val("T1 deleted itself.",T1_RUN_STATUS_S_51);
    T1_STATUS = T1_DELETED;
    mpDeleteTask(TID_SELF);
}

static void set_io(unsigned int address,short value)
{
    MP_IO_DATA sData;
    LONG num = 1;

    sData.ulAddr = address;
    sData.ulValue = value;
    
    mpWriteIO(&sData,num);
}

static short get_io(unsigned int address)
{
    MP_IO_INFO sData;
    USHORT rData;
    LONG num = 1;

    sData.ulAddr = address;

    mpReadIO(&sData,&rData,num);
    return rData;
}

static void put_s_val(char* var,unsigned short idx)
{
    MP_USR_VAR_INFO varInfo;
    
    memset(&varInfo, 0, sizeof(varInfo));
    varInfo.var_type = MP_VAR_S;
    varInfo.var_no = idx;
    if (strlen(var)>32)
    {
        memcpy(varInfo.val.s,var,32);
    }
    else
    {
        memcpy(varInfo.val.s,var,strlen(var));
    }
    while(1)
    {
        if(mpPutUserVars(&varInfo) == 0) break;
        waitSec(0.01);
    }
}

static void hold_T1_if_needed()
{
    MP_SERVO_POWER_RSP_DATA powerOn;
    

    mpGetServoPower(&powerOn);
    if((powerOn.sServoPower == 0 || T2_RESET_T1 == 1) && (T1_STATUS == T1_RUNNING)) 
    {
        // T1_STATUS = T1_SUSPENDED;
        // t1_log("T1 received suspend cmd.");
        // mpTaskSuspend(TID_SELF);
        waitSec(0.3);
    }
    
    //waitSec(0.001);
    // WAIT T2_RESET_T1 = 0
    while(T2_RESET_T1 != 0)
    {
        T1_STATUS = T1_WAIT_RESET_CANCEL;
        //t1_log("t1 resumed, wait t2 reset = 0");
        waitSec(0.01);
    }
    T1_STATUS = T1_RUNNING;
    //t1_log("T1 is running.");
}

static void stopAndShowErrMsg(char* err_msg)
{
    // [alm_code] 8000 ～ 8999, scp.error_code
    //int alm_code = 8100;
    //char alm_msg[32];
    
    if(err_msg)
    {
        put_s_val(err_msg,T1_ERROR_S_55);
    }
    else
    {
        put_s_val(" ",T1_ERROR_S_55);
    }
    end_task1();
}

static void put_r_val(float var,unsigned short idx)
{
    MP_USR_VAR_INFO varInfo;
    
    memset(&varInfo, 0, sizeof(varInfo));
    varInfo.var_type = MP_VAR_R;
    varInfo.var_no = idx;
    varInfo.val.r = var;
    while(1)
    {
        if(mpPutUserVars(&varInfo) == 0) break;
        waitSec(0.01);
    }    
}

static void put_i_val(int var,unsigned short idx)
{
    MP_USR_VAR_INFO varInfo;

    memset(&varInfo, 0, sizeof(varInfo));
    varInfo.var_type = MP_VAR_I;
    varInfo.var_no = idx;
    varInfo.val.i = var;
    while(!T2_RESET_T1)
    {
        if(mpPutUserVars(&varInfo) == 0) break;
        waitSec(0.01);
    }
}

static void put_p_val(long* var,unsigned int config, unsigned short idx)
{
    MP_USR_VAR_INFO varInfo;
    
    memset(&varInfo, 0, sizeof(varInfo));
    varInfo.var_type = MP_VAR_P;
    varInfo.var_no = idx;
    varInfo.val.p.dtype = MP_BASE_DTYPE;
    varInfo.val.p.tool_no = SCAPE_TOOL_INDEX;
    varInfo.val.p.uf_no = 0;
    varInfo.val.p.fig_ctrl = config; //MP_FIG_SIDE;
    varInfo.val.p.data[0] = var[0];
    varInfo.val.p.data[1] = var[1];
    varInfo.val.p.data[2] = var[2];
    varInfo.val.p.data[3] = var[3];
    varInfo.val.p.data[4] = var[4];
    varInfo.val.p.data[5] = var[5];
    while(1)
    {
        if(mpPutUserVars(&varInfo) == 0) break;
        waitSec(0.01);
    }
}

static void put_j_val(long* var, unsigned short idx)
{
    MP_USR_VAR_INFO varInfo;
    
    memset(&varInfo, 0, sizeof(varInfo));
    varInfo.var_type = MP_VAR_P;
    varInfo.var_no = idx;
    varInfo.val.p.dtype = MP_PULSE_DTYPE;
    varInfo.val.p.tool_no = SCAPE_TOOL_INDEX;
    varInfo.val.p.uf_no = 0;
    varInfo.val.p.fig_ctrl = MP_FIG_SIDE;
    varInfo.val.p.data[0] = var[0];
    varInfo.val.p.data[1] = var[1];
    varInfo.val.p.data[2] = var[2];
    varInfo.val.p.data[3] = var[3];
    varInfo.val.p.data[4] = var[4];
    varInfo.val.p.data[5] = var[5];
    while(1)
    {
        if(mpPutUserVars(&varInfo) == 0) break;
        waitSec(0.01);
    }
}

static char get_b_val(unsigned short idx)
{
    MP_USR_VAR_INFO varInfo1, varInfo2;
    unsigned short rc1 = 0, rc2 = 0;
    
    memset(&varInfo1, 0, sizeof(varInfo1));
    varInfo1.var_type = MP_VAR_B;
    varInfo1.var_no = idx;

    memset(&varInfo2, 0, sizeof(varInfo2));
    varInfo2.var_type = MP_VAR_B;
    varInfo2.var_no = idx;
    while(1)
    {
        rc1 = mpGetUserVars(&varInfo1);
        waitSec(0.01);
        rc2 = mpGetUserVars(&varInfo2);
        if (rc1 == 0 && rc2 == 0 && varInfo1.val.b == varInfo2.val.b)
        {
            break;
        }
    }
    return varInfo1.val.b;
}

static int get_i_val(unsigned short idx)
{
    MP_USR_VAR_INFO varInfo;
    
    memset(&varInfo, 0, sizeof(varInfo));
    varInfo.var_type = MP_VAR_I;
    varInfo.var_no = idx;
    
    while(1)
    {
        if(mpGetUserVars(&varInfo) == 0) break;
        waitSec(0.01);
    }
    
    return varInfo.val.i;
}

static void get_s_val(unsigned short idx, char str[16])
{
    MP_USR_VAR_INFO varInfo;
    
    memset(&varInfo, 0, sizeof(varInfo));
    varInfo.var_type = MP_VAR_S;
    varInfo.var_no = idx;
    
    while(1)
    {
        if(mpGetUserVars(&varInfo) == 0) break;
        waitSec(0.01);
    }
    memcpy(str,varInfo.val.s,16);
}

static void wait_b_val(unsigned int idx,char val)
{
    while(1)
    {
        if (get_b_val(idx) == val) break;
        
        put_b_val(idx,1);
        put_b_val(val,2);
        t1_log("t1 wait bite value");
        hold_T1_if_needed();
    }
}

static void wait_i_val(unsigned int idx,char val)
{
    while(1)
    {
        if (get_i_val(idx) == val)
        break;
        t1_log("t1 wait int value");
        hold_T1_if_needed();
    }
}

static void get_ip()
{
    char ip_temp[16] = {0};
    
    get_s_val(USER_IP_S_91,ip_temp);
    
    // check if ip_temp is legal form
    if(1)
    {
        memcpy(scp_ip,ip_temp,16);
    }
}

static void get_port()
{
    int port_temp = 14001;
    
    port_temp = get_i_val(USER_PORT_I_91);
    // check if port is legal
    if(port_temp > 0)
    {
        scp_port = port_temp;
    }
}

static void get_scape_frame()
{
    int tool_index,base_index;
    
    tool_index = get_i_val(USER_TOOL_I_92);
    base_index = get_i_val(USER_BASE_I_93);
    // check if frame index is leagual
    if(tool_index >= 0 && tool_index <= 64)
    {
        SCAPE_TOOL_INDEX = tool_index;
    }
    if(base_index >= 0 && base_index <= 64)
    {
        SCAPE_BASE_INDEX = base_index;
    }

}

static int getProgramExeMode(){
    return 0;
    MP_MODE_RSP_DATA rData;
    const int PLAY = 2;
	memset(&rData, 0x00, sizeof(rData));
	const int rc = mpGetMode(&rData);
    if (rData.sMode == PLAY && robot_current_working == 555) return 1;
    return 0;
}

static int update_speed(double speed,MotionType mov_type)
{
    if (mov_type == MOVEJ)
    {
        if (speed > 100) speed = 100;
    }
    else
    {
        if (speed > 1500) speed = 1500;
    }

    if (speed <= 0) speed = 10;
    
    if (mov_type == MOVEJ)
    {
        return (int)(speed*100);
    }
    else
    {
        return (int)(speed*10);
    }

}

static int update_blend(double blend)
{
    if (blend >= 200) return 8;
    if (blend >= 150) return 6;
    if (blend >= 100) return 4;
    if (blend >= 50)  return 3;
    if (blend >= 0)   return 2;
    return -1;
}

static int update_acc(double acc)
{
    if (acc > 100) return 100;
    if (acc < 20 ) return 20;
    return (int)acc;
}

static long dtl(double a)
{
    if (a >= 0) return (long)(a+0.5);
    return (long)(a-0.5);
}

static void WaitInPos(unsigned short pose_id)
{
    while (1)
    {
        t1_log("wait robot in pose.");
        wait_b_val(ROBOT_IN_POSE_B_70,pose_id);
        if (get_b_val(ALL_TASK_DONE_B_69) > 1)
        {
            break;
        }
        
    }
}

static void waitPosStarted(unsigned short pose_id)
{
    while (1)
    {
        t1_log("wait pose started.");
        //return;
        if (get_b_val(ROBOT_IN_POSE_B_70) >= pose_id)
        {
            break;
        }
    }
    
}

static void pulseToAngle(long* pulse, double* angle)
{
    long angleOut[MP_GRP_AXES_NUM];
    int result = 0;

    result = mpConvPulseToAngle(0,pulse,angleOut);
    if (result == E_KINEMA_FAILURE)
    {
        waitSec(0.01);
        result = mpConvPulseToAngle(0,pulse,angleOut);
        if (result != 0)
        {
            stopAndShowErrMsg("transform pulse to angle failed!");
        }
    }
    angle[0] = angleOut[0]/10000.0;
    angle[1] = angleOut[1]/10000.0;
    angle[2] = angleOut[2]/10000.0;
    angle[3] = angleOut[3]/10000.0;
    angle[4] = angleOut[4]/10000.0;
    angle[5] = angleOut[5]/10000.0;
}

static void GetJoints(double* cur_joints)
{
    MP_PULSE_POS_RSP_DATA rsp_data;
    MP_CTRL_GRP_SEND_DATA ctl_grp;

    ctl_grp.sCtrlGrp = 0;
    mpGetPulsePos(&ctl_grp,&rsp_data);
    pulseToAngle(rsp_data.lPos,cur_joints);
}

static void GetPose(double* cur_pose)
{
    if(DBG_YRC) printf("\n rob GetPose start\n");
    MP_CARTPOS_EX_SEND_DATA sData;
    MP_CART_POS_RSP_DATA_EX rData;
    
    sData.sRobotNo = 0;
    sData.sFrame = SCAPE_BASE_INDEX;
    sData.sToolNo = SCAPE_TOOL_INDEX;
    
    mpGetCartPosEx(&sData,&rData);
    
    cur_pose[0] = rData.lPos[0]/1000.0;
    cur_pose[1] = rData.lPos[1]/1000.0;
    cur_pose[2] = rData.lPos[2]/1000.0;
    cur_pose[5] = rData.lPos[3]/10000.0;
    cur_pose[4] = rData.lPos[4]/10000.0;
    cur_pose[3] = rData.lPos[5]/10000.0;

    if(DBG_YRC) printf("\n rob GetPose end %lf %lf %lf %lf %lf %lf\n",cur_pose[0],cur_pose[1],cur_pose[2],cur_pose[3],cur_pose[4],cur_pose[5]);
}

static void calculate_best_config(const double* pTarget,const double* jConfig,BITSTRING* result_config)
{
    MP_COORD coord;
    long start_angle[MP_GRP_AXES_NUM];
    long angle1[MP_GRP_AXES_NUM],angle2[MP_GRP_AXES_NUM];
    int status = 0;
    BITSTRING config1,config2,cfg = 32;
    
    start_angle[0] = dtl(jConfig[0] * 10000);
    start_angle[1] = dtl(jConfig[1] * 10000);
    start_angle[2] = dtl(jConfig[2] * 10000);
    start_angle[3] = dtl(jConfig[3] * 10000);
    start_angle[4] = dtl(jConfig[4] * 10000);
    start_angle[5] = dtl(jConfig[4] * 10000);
    
    // calculate configuration
    status = mpConvAxesToCartPos(0,start_angle,SCAPE_TOOL_INDEX,&config1,&coord);
    if (status == E_KINEMA_FAILURE)
    {
        waitSec(0.01);
        status = mpConvAxesToCartPos(0,start_angle,SCAPE_TOOL_INDEX,&config1,&coord);
        if (status != 0) stopAndShowErrMsg("cal config error!");
    }
    if (status != 0) stopAndShowErrMsg("cal config error!");

    // tilt D05
    if(config1 & cfg)
    {
        cfg = 31;
        config2 = config1 & cfg;
    }
    else
    {
        cfg = 63;
        config2 = config1 & cfg;
    }
    
    coord.x = (long)(pTarget[0] * 1000);
    coord.y = (long)(pTarget[1] * 1000);
    coord.z = (long)(pTarget[2] * 1000);
    coord.rz = (long)(pTarget[3] * 10000);
    coord.ry = (long)(pTarget[4] * 10000);
    coord.rx = (long)(pTarget[5] * 10000);
    
    mpConvCartPosToAxes(0,&coord,SCAPE_TOOL_INDEX,config1,NULL,MP_KINEMA_FIG,angle1);
    mpConvCartPosToAxes(0,&coord,SCAPE_TOOL_INDEX,config2,NULL,MP_KINEMA_FIG,angle2);
    
    if(abs(angle1[5]) <= abs(angle2[5]))
    {
        *result_config = config1;
    }
    else
    {
        *result_config = config2;
    }
}

static void connect_to_scape()
{
    int result = 0;
    struct sockaddr_in scapeAddress;
    
    if(sockHandle > 0) stopAndShowErrMsg("sockHandle > 0");
    // create socket
    sockHandle = mpSocket(AF_INET,SOCK_STREAM,0);
    if(sockHandle <= 0) stopAndShowErrMsg("create socket failed!");
    memset(&scapeAddress, 0, sizeof(scapeAddress));
    scapeAddress.sin_family = AF_INET;
    scapeAddress.sin_addr.s_addr = mpInetAddr(scp_ip);
    scapeAddress.sin_port = mpHtons(scp_port);

    // connect to scape server
    result = mpConnect(sockHandle,(struct sockaddr *)&scapeAddress,sizeof(scapeAddress));
    if (result !=0)
    {
        stopAndShowErrMsg("connect to scape server failed!");
    }    
}

static void disconnect_to_scape()
{
    if (sockHandle == -9999) return;
    mpClose(sockHandle);
    sockHandle = -9999;
}

static void send_to_scape(const char* str)
{
    int result = 0;
    
    t1_log("t1 send scape string");
    hold_T1_if_needed();
    if (sockHandle < 0)
    {
        stopAndShowErrMsg("socket illegal when send string to server!");
    }
    result = mpSend(sockHandle,str,strlen(str),0);
    if (result < 0 )
    {
        stopAndShowErrMsg("error happend when send string to server!");
    }
    waitSec(0.01);
}

static void receive_from_scape(char* str)
{
    int result = 0;
    
    if (sockHandle < 0)
    {
        stopAndShowErrMsg("socket illegal when recive string from server!");
    }
    memset(str,'\0',strlen(str));
    result = mpRecv(sockHandle,str,BUFF_LEN,0);
    if (result <0 )
    {
        stopAndShowErrMsg("error happend when receive string from server!");
    }
    waitSec(0.01);
}

static void showMsgOnTP(char* msg,MsgLevel level)
{
    if (level == Error)
    {
        while (1)
        {
            waitSec(3);    
            if(msg)
            {
                put_s_val(msg,SCP_MSG_S_54);
            }
            else
            {
                put_s_val(" ",SCP_MSG_S_54);
            }
        }
    }
    else
    {
        if(msg)
        {
            put_s_val(msg,SCP_MSG_S_54);
        }
        else
        {
            put_s_val(" ",SCP_MSG_S_54);
        }
    }
}

static void cleanTPMsg()
{
    put_s_val(" ",SCP_MSG_S_54);
}

static void poseToMotman(const double* pose,long* p)
{
    memset(p,0,sizeof(p));
    p[0] = dtl(pose[0] * 1000);
    p[1] = dtl(pose[1] * 1000);
    p[2] = dtl(pose[2] * 1000);
    p[3] = dtl(pose[5] * 10000);
    p[4] = dtl(pose[4] * 10000);
    p[5] = dtl(pose[3] * 10000);
}

static void angleToPulse(const double* angle, long*pulse)
{
    long pulseIn[8];
    int result = 0;
    
    pulseIn[0] = dtl(angle[0] * 10000);
    pulseIn[1] = dtl(angle[1] * 10000);
    pulseIn[2] = dtl(angle[2] * 10000);
    pulseIn[3] = dtl(angle[3] * 10000);
    pulseIn[4] = dtl(angle[4] * 10000);
    pulseIn[5] = dtl(angle[5] * 10000);
    pulseIn[6] = 0;
    pulseIn[7] = 0;
    //memset(pulseIn,0,sizeof(pulseIn));
    result = mpConvAngleToPulse(0,pulseIn,pulse);
    if (result == E_KINEMA_FAILURE)
    {
        if(DBG_YRC) printf("\n\n angleToPulse failed first time");
        waitSec(0.01);
        result = mpConvAngleToPulse(0,pulseIn,pulse);
        if (result != 0)
        {
            stopAndShowErrMsg("transform angle to pulse failed!");
        }
    }
}

static void MovJ(unsigned int idx,ScapeTask task)
{
    long pulse[SCAPE_ROBOT_AXIS_NUM + 2]= { };
    long p[SCAPE_ROBOT_AXIS_NUM + 2] = {};
    //double newJoints[MAX_ROBOT_AXIS+2] = {0};
    int i;
    unsigned int config = MP_FIG_SIDE;
    double pose[SCAPE_ROBOT_AXIS_NUM] = { 0 };

    if(DBG_YRC) printf("\nrob: joints j1 %lf,j2 %lf,j3 %lf,j4 %lf,j5 %lf,j6 %lf\n", task.joints[0], task.joints[1], task.joints[2], task.joints[3], task.joints[4], task.joints[5]);
    // time to put motion data to registor
    // if (task.target == NULL)
    // {
    //     if (task.joints == NULL) stopAndShowErrMsg("MovJ pose and joints both are NULL!!");
    //     // move to joints directly
    //     if(DBG_YRC) puts("\n rob: MovJ move to joints directly\n");
    //     angleToPulse(task.joints, pulse);
    //     put_j_val(pulse, yrc_task[idx].p_pose_idx);
    //     stopAndShowErrMsg("task.target == NULL");
    // }
    // else
    // {
    //     for (i = 0; i < 6; i++) pose[i] = task.target[i];
    //     if(DBG_YRC) printf("\nrob: poseTarget x %lf,y %lf,z %lf,a %lf,b %lf,c %lf\n", task.target[0], task.target[1], task.target[2], task.target[3], task.target[4], task.target[5]);
    //     if (task.joints == NULL) stopAndShowErrMsg("MovJ joints is NULL!!");;
    //     // calculate the configuration
    //     calculate_best_config(task.target,task.joints, &config);
    //     poseToMotman(pose, p);
    //     put_p_val(p, config, yrc_task[idx].p_pose_idx);
    // }
    if (robot_current_working == 555 || get_i_val(22) == 555)
    {
        angleToPulse(task.joints, pulse);
        put_j_val(pulse, yrc_task[idx].p_pose_idx);
    }
    else
    {
        for (i = 0; i < 6; i++) pose[i] = task.target[i];
        if(DBG_YRC) printf("\nrob: poseTarget x %lf,y %lf,z %lf,a %lf,b %lf,c %lf\n", task.target[0], task.target[1], task.target[2], task.target[3], task.target[4], task.target[5]);
        if (task.joints == NULL) stopAndShowErrMsg("MovJ joints is NULL!!");;
        // calculate the configuration
        calculate_best_config(task.target,task.joints, &config);
        poseToMotman(pose, p);
        put_p_val(p, config, yrc_task[idx].p_pose_idx);
    }

    // set move type is J
    put_b_val(MOVEJ, yrc_task[idx].b_motionType_idx);

    put_i_val(update_speed(task.speed, MOVEJ), yrc_task[idx].i_speed_idx);

    put_i_val(update_acc(task.acc), yrc_task[idx].i_acc_idx);

    put_i_val(update_blend(task.blend), yrc_task[idx].i_blend_idx);
}

static void MovL(unsigned int idx,ScapeTask task)
{
    long p[6] = {0};
    double pose[6] = {};
    int i = 0;
    long pulse[SCAPE_ROBOT_AXIS_NUM + 2]= {};
    unsigned int config = MP_FIG_SIDE;
    long pluse[SCAPE_ROBOT_AXIS_NUM+3]={};
    
    // time to put motion data to registor
    for (i = 0; i < 6; i++) pose[i] = task.target[i];
    poseToMotman(pose, p);

    if (robot_current_working == 555 || get_i_val(22) == 555)
    {
        angleToPulse(task.joints, pulse);
        put_j_val(pulse, yrc_task[idx].p_pose_idx);
    }
    else {
        calculate_best_config(task.target,task.joints, &config);
        //calculate_best_config1(task.target,task.joints, &config,pluse);
        put_p_val(p, config, yrc_task[idx].p_pose_idx);  
    }
    
    // set move type is L
    put_b_val(MOVEL, yrc_task[idx].b_motionType_idx);
    put_i_val(update_speed(task.speed, MOVEL), yrc_task[idx].i_speed_idx);
    put_i_val(update_acc(task.acc), yrc_task[idx].i_acc_idx);
    put_i_val(update_blend(task.blend), yrc_task[idx].i_blend_idx);
}

static void waitSentTaskDone()
{
    int rc, i;

    while(1)
    {
        rc = 0;
        for (i = 0; i < ROBOT_TASK_BUFF; i++)
        {
            rc += get_b_val(yrc_task[i].b_task_flag_idx);
        }
        
        if (rc == 0)
        {
            break;
        }
        t1_log("t1 wait all sent tasks done.");
        hold_T1_if_needed();
    }
}

static void wait_scape_task_complete(TaskRunStatus* status)
{
    waitSentTaskDone();
    if (status) *status = get_b_val(EXECUTION_STATUS_B_60);
    put_b_val(0,EXECUTION_STATUS_B_60);
}

static void runScapeTask(ScapeTask* task)
{
    int free_yrc_task_idx = 0,i = 0;
    
    if(task->taskIndex < 1) stopAndShowErrMsg("task index < 1!");
    
    // find a free buff
    while (1)
    {
        for (i = 0; i < ROBOT_TASK_BUFF; i++)
        {
            if (get_b_val(yrc_task[i].b_task_flag_idx) == 0)
            {
                free_yrc_task_idx = i;
                goto SEND_TASK;
            }
        }
    }
    SEND_TASK:
    // step 1 write task data to TP
    current_task_idx = task->taskIndex;
    put_b_val(task->taskIndex, yrc_task[free_yrc_task_idx].b_task_index_idx);
    current_tasks_num = task->taskNumInTotal;
    put_b_val(task->taskNumInTotal, yrc_task[free_yrc_task_idx].b_task_num_idx);
    put_b_val(task->motionType,yrc_task[free_yrc_task_idx].b_motionType_idx);
    put_i_val(task->job_id,yrc_task[free_yrc_task_idx].i_job_id_idx);
    put_b_val(0,ROBOT_IN_POSE_B_70);
    // write motion data if needed
    if (task->motionValid)
    {           
        if (task->motionType == MOVEL)
        {
            MovL(free_yrc_task_idx, *task);
        }
        else
        {
            MovJ(free_yrc_task_idx, *task);
        }
    }
    // write logical data if needed
    if (task->logicValid)
    {
        put_i_val(task->par0, yrc_task[free_yrc_task_idx].i_par0_idx);
        put_i_val(task->par1, yrc_task[free_yrc_task_idx].i_par1_idx);
        put_i_val(task->par2, yrc_task[free_yrc_task_idx].i_par2_idx);
    }

    //step 2 set the current task flag is ready, let TP execute task
    if(task->job_id == JOB_EXIT)
    {
        put_b_val(JOB_EXIT, yrc_task[free_yrc_task_idx].b_task_flag_idx);
    }
    else
    {
        put_b_val(1, yrc_task[free_yrc_task_idx].b_task_flag_idx);
    }
    
    if (task->stateWaitMotionStart && robot_current_working == ROBOT_IN_CALIB)
    {
        waitPosStarted(task->taskIndex);
    }
    
    if ((task->blend == -1 && robot_current_working == ROBOT_IN_CALIB) || task->job_id == JOB_TEACH || task->job_id == JOB_GET_TCP_POSE || task->job_id == JOB_GET_JOINTS || task->job_id == JOB_CHECK_GRIP)
    {
        WaitInPos(task->taskIndex);
    }
    
    // read gripper status
    if (task->job_id == JOB_CHECK_GRIP)
    {
        task->par0 = get_b_val(GRIPPER1_STATUS_B_66);
        task->par1 = get_b_val(GRIPPER2_STATUS_B_67);
        task->par2 = get_b_val(GRIPPER3_STATUS_B_68);
        put_b_val(0,GRIPPER1_STATUS_B_66);
        put_b_val(0,GRIPPER2_STATUS_B_67);
        put_b_val(0,GRIPPER3_STATUS_B_68);
    }

    if(task->job_id == JOB_GET_TCP_POSE || task->job_id == JOB_TEACH)
    {
        GetPose(task->target);
    }

    if(task->job_id == JOB_GET_JOINTS)
    {
        GetJoints(task->joints);
    }
    return;
}

// static void StopMotion(const int grpNo){
// 	if (mpMotStop(0) < 0)
// 	{
// 		stopAndShowErrMsg("mpMotStop(0)\n");
// 	}
// 	if (mpMotTargetClear(0x0f, 0) < 0)
// 	{
// 		stopAndShowErrMsg("mpMotTargetClear(0)\n");
// 	}
// }

/* static void StartMotion(const int grpNo){
    // motion start!
	int rc = 0;
	LONG nRet;
	MP_PLAY_STATUS_RSP_DATA dxRet;
    MP_TASK_SEND_DATA sData;
    MP_CUR_JOB_RSP_DATA rData;
    MP_HOLD_SEND_DATA sdHold;
    MP_STD_RSP_DATA rHold;
    MP_SERVO_POWER_SEND_DATA sPower;
    MP_STD_RSP_DATA rPower;
    MP_SERVO_POWER_RSP_DATA serverPower;
    memset(&sdHold,0,sizeof(sdHold));
    memset(&sPower,0,sizeof(sPower));
    memset(&serverPower,0,sizeof(serverPower));
	while (1)
	{   
        rc = mpGetPlayStatus(&dxRet);
        if (rc == 0 && dxRet.sStart == 1){ // start on
            sdHold.sHold = 1; // hold on
            rc = mpHold(&sdHold,&rHold);
            waitSec(0.01);
            sdHold.sHold = 0; // hold off
            rc = mpHold(&sdHold,&rHold);
        }
        // else{
        //     sdHold.sHold = 0; // hold off
        //     rc = mpHold(&sdHold,&rHold);
        // }
        rc = mpGetServoPower(&serverPower);
        if (serverPower.sServoPower == 0){ // server power off
            sPower.sServoPower = 1;
            rc = mpSetServoPower(&sPower,&rPower);
        }
        
        t1_log("start motion");
		if ((rc = mpMotStart(0)) < 0)
		{
			if ((E_MP_MOT_SVOFF != rc) && (E_MP_MOT_HOLD != rc))
			{
				StopMotion(grpNo);
				stopAndShowErrMsg("mpMotStart(0) falied");
			}
		}
		else if (0 == rc)
		{
			return;
		}
		waitSec(0.2);
	}
} */

// static void calculate_best_config1(const double* pTarget, const double* jConfig, BITSTRING* result_config, long* ResultPuls)
// {
// 	MP_COORD coord;
// 	BITSTRING config1, config2;
// 	long start_angle[MP_GRP_AXES_NUM];

// 	start_angle[0] = dtl(jConfig[0] * 10000);
// 	start_angle[1] = dtl(jConfig[1] * 10000);
// 	start_angle[2] = dtl(jConfig[2] * 10000);
// 	start_angle[3] = dtl(jConfig[3] * 10000);
// 	start_angle[4] = dtl(jConfig[4] * 10000);
// 	start_angle[5] = dtl(jConfig[5] * 10000);

// 	// calculate configuration
// 	int status = mpConvAxesToCartPos(0, start_angle, SCAPE_TOOL_INDEX, &config1, &coord);
// 	if (status == E_KINEMA_FAILURE)
// 	{
// 		waitSec(0.01);
// 		status = mpConvAxesToCartPos(0, start_angle, SCAPE_TOOL_INDEX, &config1, &coord);
// 		if (status != 0)
// 		{
// 			stopAndShowErrMsg("cal config error!");
// 		}
// 	}
// 	if (status != 0)
// 	{
// 		stopAndShowErrMsg("cal config error!");
// 	}

// 	//
// 	// TODO Add checks for flip
// 	//

// 	// D00: Front
// 	// D01: Upper arm
// 	// D02: 0: No flip
// 	// D03: 0: R < 180 1: R >= 180
// 	// D04: 0: T < 180 1: T >= 180
// 	// D05: 0: S < 180 1: S >= 180
// 	// D06 - D15: Reserved by manufacturer
// 	config1 = config1 | 0x4;

// 	// 6 axis roation D04
// 	if (config1 & 0x10)
// 	{
// 		//printf("Forcing D04 = 0 \n");
// 		config2 = config1 & 0xFFFFFFEF; // Set D04 = 0
// 	}
// 	else
// 	{
// 		//printf("Forcing D04 = 1 \n");
// 		config2 = config1 | 0x10; // Set D05 = 1
// 	}

// 	// Use the coordinatates from the SCAPE PC
// 	coord.x = (long)(pTarget[0] * 1000);
// 	coord.y = (long)(pTarget[1] * 1000);
// 	coord.z = (long)(pTarget[2] * 1000);
// 	coord.rz = (long)(pTarget[3] * 10000);
// 	coord.ry = (long)(pTarget[4] * 10000);
// 	coord.rx = (long)(pTarget[5] * 10000);

// 	long angle1Output[MP_GRP_AXES_NUM], angle2Output[MP_GRP_AXES_NUM];
// 	if (OK != mpConvCartPosToAxes(0, &coord, SCAPE_TOOL_INDEX, config1, NULL, MP_KINEMA_FIG, angle1Output))
// 	{
// 		stopAndShowErrMsg("cmpConvCartPosToAxes angle1 faild");
// 	}
// 	if (OK != mpConvCartPosToAxes(0, &coord, SCAPE_TOOL_INDEX, config2, NULL, MP_KINEMA_FIG, angle2Output))
// 	{
// 		stopAndShowErrMsg("cmpConvCartPosToAxes angle2 faild");
// 	}

// 	const long config1_delta = abs(start_angle[5] - angle1Output[5]);
// 	const long config2_delta = abs(start_angle[5] - angle2Output[5]);
// 	if (config1_delta <= config2_delta)
// 	{
// 		//printf("Chosing angle 1 \n");
// 		*result_config = config1;
// 		if (0 != mpConvAngleToPulse(0, angle1Output, ResultPuls))
// 		{
// 			stopAndShowErrMsg("mpConvAngleToPulse angle 1");
// 		}
// 	}
// 	else
// 	{
// 		//printf("Chosing angle 2 \n");
// 		*result_config = config2;
// 		if (0 != mpConvAngleToPulse(0, angle2Output, ResultPuls))
// 		{
// 			stopAndShowErrMsg("mpConvAngleToPulse angle 2");
// 		}
// 	}
// }

// static int AddedTaskToBuffer(ScapeTask* task, int grpNo, long pulse[])
// {
// 	// if ((task->joints == NULL) && (task->motionType == MOVEJ))
// 	// {
// 	// 	return 0;
// 	// }
// 	int rc = 0;

// 	MP_SPEED spd;
// 	MP_TARGET target;
//     MP_USR_VAR_INFO varInfo;
// 	/* long p[SCAPE_ROBOT_AXIS_NUM + 2] = {};
// 	//double newJoints[MAX_ROBOT_AXIS+2] = {0};
// 	unsigned int config = MP_FIG_SIDE;
// 	double pose[SCAPE_ROBOT_AXIS_NUM] = { 0 };

// 	// Copy over the Target data
// 	int jointIndex;
// 	for (jointIndex = 0; jointIndex < 6; jointIndex++)
// 	{
// 		pose[jointIndex] = task->target[jointIndex];
// 	}

// 	// calculate the configuration
// 	calculate_best_config1(task->target, task->joints, &config, pulse);
// 	// Convert pose to motoman pos
// 	poseToMotman(pose, p); */
    
//     // yrc_task[0].p_pose_idx
// 	// Set reset the target
// 	memset(&target, 0, sizeof(target));
//     memset(&varInfo, 0, sizeof(varInfo));
//     varInfo.var_type = MP_VAR_P;
// 	varInfo.var_no = 51;
// 	// Set Index
// 	target.id = task->taskIndex;
//     angleToPulse(task->joints, pulse);
//     put_j_val(pulse, 51);
//     if (task->motionType == MOVEL){
//         target.intp = MP_MOVL_TYPE;
//         // MovL(0, *task);
//         // memcpy(&target.dst.coord, varInfo.val.p.data, sizeof(target.dst.coord));
//         memcpy(target.dst.joint, varInfo.val.p.data, sizeof(target.dst.joint));
// 	}
// 	else
// 	{
// 		target.intp = MP_MOVJ_TYPE;
//         // MovJ(0, *task);
//         // memcpy(&target.dst.coord, varInfo.val.p.data, sizeof(target.dst.coord));
//         memcpy(target.dst.joint, varInfo.val.p.data, sizeof(target.dst.joint));
// 	}
// 	// Copy the data to the target
// 	//memcpy(&target.dst.coord, p, sizeof(target.dst.coord));
// /* 	target.dst.joint[0] = pulse[0];
// 	target.dst.joint[1] = pulse[1];
// 	target.dst.joint[2] = pulse[2];
// 	target.dst.joint[3] = pulse[3];
// 	target.dst.joint[4] = pulse[4];
// 	target.dst.joint[5] = pulse[5];
// 	target.dst.joint[6] = 0;
// 	target.dst.joint[7] = 0; */

// 	// Setup the interpolation type
// 	// if (task->motionType == MOVEL)
// 	// {
// 	// 	target.intp = MP_MOVL_TYPE;
// 	// }
// 	// else
// 	// {
// 	// 	target.intp = MP_MOVJ_TYPE;
// 	// }
// 	// Convert blend to um from mm
// 	// Note task->Blend = -1 is used as "no blend", therefore this distinction.
// 	const long Blend_um = task->blend > 0 ? task->blend * 1000 : 0;
// 	// Speed for movement
// 	const int Speed = update_speed(task->speed, task->motionType);
// 	memset(&spd, 0, sizeof(spd));
// 	if (task->motionType == MOVEL)
// 	{
// 		spd.v = Speed;
// 		spd.vj = 100 * 100; //Max joint speed
// 	}
// 	else
// 	{
// 		// It is unclear whether the v values is used during MOVEJ
// 		// We may have to change this later.
// 		spd.v = 1500; //Max linar speed
// 		spd.vj = Speed;
// 	}

// 	// initialize motion control with the first task.
// 	if (task->taskIndex == 1)
// 	{
// 		if (mpMotStop(0) < 0)
// 		{
// 			return 0;
// 		}
// 		if (mpMotTargetClear(0x0f, 0) < 0)
// 		{
// 			return 0;
// 		}
// 	}

// 	// Set Coordinate base system.
// 	// if ((rc = mpMotSetCoord(grpNo, MP_PULSE_TYPE, 0)) < 0)
// 	if ((rc = mpMotSetCoord(grpNo, MP_BASE_TYPE, 0)) < 0)
// 	{
// 		return 0;
// 	}

// 	if ((rc = mpMotSetOrigin(grpNo, MP_ABSO_VAL)) < 0)
// 	{
// 		return 0;
// 	}

// 	// Set the speed for the movement
// 	if ((rc = mpMotSetSpeed(grpNo, &spd)) < 0)
// 	{
// 		return 0;
// 	}

// 	// // Set the Acceleration for the movement
// 	// const long Acceleration = update_acc(task->acc) * 100;
// 	// if ((rc = mpMotSetAccel(grpNo, Acceleration)) < 0)
// 	// {
// 	// 	return 0;
// 	// }
// 	// if ((rc = mpMotSetDecel(grpNo, Acceleration)) < 0)
// 	// {
// 	// 	return 0;
// 	// }

// 	// Set the joint Config for the movement
// 	// if ((rc = mpMotSetConfig(grpNo, config)) < 0)
// 	// {
// 	// 	return 0;
// 	// }

// 	// Sets the value for blends
// 	if ((rc = mpMotSetAccuracy(grpNo, Blend_um)) < 0)
// 	{
// 		return 0;
// 	}

// 	// Put the target in the queue
// 	const int timeout = 10 / mpGetRtc();
// 	int TargetSendRC = mpMotTargetSend((1 << grpNo), &target, timeout);
// 	if (TargetSendRC < 0)
// 	{
// 		return 0;
// 	}
// 	return 1;
// }

// static int IsRobotPositionDifferenceLessThen(const long DifferenceAng, long pulse[])
// {
// 	MP_CTRL_GRP_SEND_DATA sPulsePosData;
// 	MP_PULSE_POS_RSP_DATA rPulsePosData;
// 	memset(&sPulsePosData, 0x00, sizeof(sPulsePosData));
// 	memset(&rPulsePosData, 0x00, sizeof(rPulsePosData));
// 	long rc = mpGetPulsePos(&sPulsePosData, &rPulsePosData);
// 	if (rc < 0)
// 	{
// 		return 0;
// 	}
// 	long a0Diff = abs(rPulsePosData.lPos[0] - pulse[0]);
// 	long a1Diff = abs(rPulsePosData.lPos[1] - pulse[1]);
// 	long a2Diff = abs(rPulsePosData.lPos[2] - pulse[2]);
// 	long a3Diff = abs(rPulsePosData.lPos[3] - pulse[3]);
// 	long a4Diff = abs(rPulsePosData.lPos[4] - pulse[4]);
// 	long a5Diff = abs(rPulsePosData.lPos[5] - pulse[5]);

// 	if (DifferenceAng > a0Diff &&
// 		DifferenceAng > a1Diff &&
// 		DifferenceAng > a2Diff &&
// 		DifferenceAng > a3Diff &&
// 		DifferenceAng > a4Diff &&
// 		DifferenceAng > a5Diff)
// 	{
// 		return 1;
// 	}
// 	else
// 	{
// 		return 0;
// 	}
// }

/* static int MPRunScapeTask(ScapeTask tasks[],short nTotalNumberOfTasks){
    const int Completed = 0;
	if (tasks[0].job_id == JOB_GET_TCP_POSE || tasks[0].job_id == JOB_TEACH)
	{
		GetPose(tasks[0].target);
		return Completed;
	}
	if (tasks[0].job_id == JOB_GET_JOINTS)
	{
		GetJoints(tasks[0].joints);
		return Completed;
	}

    long pulse[SCAPE_ROBOT_AXIS_NUM + 2] = { };
    int bStartMotionWasCalled = 0;
	int grpNo = 0;
	int rc = 0;
    grpNo = mpCtrlGrpId2GrpNo(MP_R1_GID);
    const int nTool = SCAPE_TOOL_INDEX;
    rc = mpMotSetTool(grpNo, nTool);
    int ntaskIndex = 0;
	const int PollingTimeout = 5 / mpGetRtc();
	int LastPathSegment = 0;
    int StopCompletely = 0;
    int motionAdded = 0;
    int targetID = 0;
	LONG nPlayStatusRet;
	MP_PLAY_STATUS_RSP_DATA dxRet;
	nPlayStatusRet = mpGetPlayStatus(&dxRet);
    SHORT sHoldOld = dxRet.sHold;

    while(1){
        put_b_val(0,20);
        if (get_b_val(20)==1) break;
        t1_log("wait scp core loop");
        waitSec(0.1);
    }
    for (ntaskIndex = 0; ntaskIndex < nTotalNumberOfTasks; ntaskIndex++){
        ScapeTask* pCurrentTask = &(tasks[ntaskIndex]);
        StopCompletely = 0;
        if (pCurrentTask->blend <= 0) StopCompletely = 1;
        if (pCurrentTask->motionValid){
            targetID = pCurrentTask->taskIndex;
            if (AddedTaskToBuffer(pCurrentTask, grpNo, pulse) == 0)
            {
                StopMotion(grpNo);
            }
            motionAdded = 1;
        }
        if (motionAdded && !bStartMotionWasCalled){
            StartMotion(grpNo);
            bStartMotionWasCalled = 1;
        }
        if ((StopCompletely || pCurrentTask->job_id > 0 || pCurrentTask->taskIndex == pCurrentTask->taskNumInTotal) && motionAdded){
            const int TargetIDForPolling = targetID;
            while(1){
                const int ReturnCodeForPolling = mpMotTargetReceive(grpNo, TargetIDForPolling, NULL, PollingTimeout, 0);
                if (ReturnCodeForPolling >= 0){
                    break;
                }
                hold_T1_if_needed();
                if (IsRobotPositionDifferenceLessThen(100,pulse)){
                    break;
                }
            }
            if (pCurrentTask->job_id == 9){
                set_io(30244,1);
                set_io(20245,0);
            }
            if (pCurrentTask->job_id == 10){
                set_io(30244,0);
                set_io(20245,1);
            }
            if (pCurrentTask->job_id == 11){
                set_io(30244,1);
                set_io(20245,0);
            }
            if (pCurrentTask->job_id == 12){
                set_io(30244,0);
                set_io(20245,1);
            }
        }
    }
    StopMotion(grpNo);
    return Completed;
} */

static void create_robot(Robot* robot)
{
    int i;
    if (robot)
    {
        robot->fnWaitSec = waitSec;
        robot->fnConnectToScape = connect_to_scape;
        robot->fnDisconnectToScape = disconnect_to_scape;
        robot->fnSendToScape = send_to_scape;
        robot->fnReceiveFromScape = receive_from_scape;
        robot->fnShowMsg = showMsgOnTP;
        robot->fnCleanMsg = cleanTPMsg;
        robot->fnGetExeMode = getProgramExeMode;
        robot->fnRunScapeTask = runScapeTask;
        // robot->fnMPRunScapeTask = MPRunScapeTask;
        robot->fnWaitScapeTaskComplete = wait_scape_task_complete;
        for(i = 0; i< 6; i++) robot->jGlobalBestConfig[i] = jGlobalCfg[i];
    }
    else
    {
        end_task1();
    }
}

static void T1_wakeUp()
{
    //程序刚启动后，因系统访问变量区域，在mpPutVarData()、
    //mpPutSVarInfo()、mpPutPosVarData() 等的变量区域进行写入
    //的函数可能会返回错误。因此，需要任务自动启动的程序，请
    //设计一段重复读取变量的结构，直到没有错误返回。
   t1_log("t1 wake up...");
    while(1)
    {
        put_b_val(1,1);
        if(get_b_val(1) == 1) break;
    }
    while(1)
    {
        put_b_val(0,1);
        if(get_b_val(1) == 0) break;
    }
}

static void mpTask1Reset(void)
{
    int i = 0;
    
    while(1)
    {
        t1_log("t1 wait t2 not reset t1");
        if(!T2_RESET_T1) break;
        waitSec(0.01);
    }
    t1_log("t1 wait bite vars reset");
    for(i = 51; i<=70; i++)
    {
        wait_b_val(i,0);
    }
    t1_log("t1 wait int vars reset");
    for(i = 51; i<=65; i++)
    {
        wait_i_val(i,0);
    }
    t1_log("t1 wait vars reset complete");
}

static unsigned int getGroupId()
{
    return get_b_val(GROUP_ID_B_62);
}

static unsigned int getProductId()
{
    return (get_b_val(PRODUCT_ID_B_63) - 1);
}

static unsigned int getNumOfProducts()
{
    return get_b_val(PRODUCTS_NUM_B_64);
}

static PickCfg getPickCfg()
{
    return get_b_val(PICK_CFG_B_59);
}

static unsigned int getUserChoice()
{
    unsigned int choice;
    choice = get_b_val(CHOICE_B_61);
    if (choice != 0)
    {
    	put_b_val(0,CHOICE_B_61);
    }
    
    return choice;
}

static short isRescanNeed()
{
    if (get_b_val(NEED_RESCAN_B_65)) return 1;
    return 0;
}

static void set_return_val(int ret)
{
    put_i_val(ret,PICK_RETURN_I_65);
}

static void set_bin_height(int hight_in_mm){
    put_i_val(hight_in_mm,BIN_HEIGHT_RETURN_I_66);
}

static void t1_log(char* msg)
{
   put_s_val(msg,T1_RUN_STATUS_S_51);
}

void reset_all_bins()
{
    int i;
    for (i = 0; i < MAX_PRODUCT_NUM_IN_GROUP; i++)
    {   
        bins[i].bin_status = 0;
        bins[i].picked_parts_count = 0;
        bins[i].remain_parts_height_mm = 0;
        bins[i].product_group_id = 1;
        bins[i].product_id = i + 1;
        bins[i].start_height_mm = get_i_val(i+1);
    }
}

int get_next_bin(int current){
    if (current == 0) return 3;
    if (current == 3) return 1;
    if (current == 1) return 2;
    if (current == 2) return 0;
    return 0;
}

void get_io_test()
{
    MP_IO_INFO nfo;
    nfo.ulAddr = 161;
    USHORT signal;
    static int f = 1;
    put_b_val(mpReadIO(&nfo, &signal, 1),16);
    put_b_val(signal,17);
    put_b_val(++f,18);
}

void get_bin_id(int* current, int* next)
{
    int rc = 0;
    static int current_bin = 0;
    static int next_bin = 0;
    int i = 0;
    int b16 = 0;
    MP_IO_INFO pick_bin[4];
    MP_IO_INFO bReady[4];
    pick_bin[0].ulAddr = 160;
    pick_bin[1].ulAddr = 161;
    pick_bin[2].ulAddr = 162;
    pick_bin[3].ulAddr = 163;
    bReady[0].ulAddr = 140;
    bReady[1].ulAddr = 141;
    bReady[2].ulAddr = 142;
    bReady[3].ulAddr = 143;

    USHORT bin_enable[4] = {0,0,0,0};
    USHORT bin_ready[4] = {0,0,0,0};

    rc = mpReadIO(pick_bin,bin_enable,4);
    rc = mpReadIO(bReady,bin_ready,4);
    // b16 : current robot is picking product idx
    b16 = get_b_val(16) - 1;

    // if (!(bin_ready[0] && bin_ready[1] && bin_ready[2] && bin_ready[3]))
    // {
    //     *current = 255;
    //     *next = 255;
    //     return;
    // }
    
    if (next_bin > 3 || next_bin < 0)
    {
        next_bin = 0;
    }
    
    if (bin_enable[next_bin] != 0 && next_bin != b16 && bin_ready[next_bin] != 0)
    {
        current_bin = next_bin;
    }
    else
    {
        current_bin = 255;
        for (i = 0; i < 4; i++)
        {
            if (bin_enable[i] != 0 && b16 != i && bin_ready[i] != 0)
            {
                current_bin = i;
                break;
            }           
        }
    }
    *current = current_bin;
    next_bin = 255;
    if (current_bin >=0 && current_bin <= 3)
    {
       next_bin = get_next_bin(current_bin);
       if (bin_enable[next_bin] == 0 || next_bin == b16 || bin_ready[next_bin] == 0)
       {           
            for (i = 0; i < 4; i++)
            {
                if (bin_enable[i] != 0 && i != current_bin && i != b16 && bin_ready[i] != 0)
                {
                    next_bin = i;
                    break;
                }
            }
       }        
    }
    *next = next_bin;
}

int is_conveyor_ready(const int id){
    if (id == 1){
        return get_io(20225);
    }
    if (id == 2){
        return get_io(20226);
    }
    return 0;
}

int get_count(int fixture){
    if(fixture == 1){
        if(get_io(20221)){
            int count1 = 0;
            if(get_io(20227)) count1++;
            if(get_io(20230)) count1++;
            return count1;
        }
        
    }
    if(fixture == 2){
        if(get_io(20222)){
            int count2 = 0;
            if(get_io(20231)) count2++;
            if(get_io(20232)) count2++;
            return count2;
        }
        
    }
    return 0;
}

int get_tp_bin_to_pick(){
    if(get_io(20236)&&is_conveyor_ready(2)){
        last_pick = 2;
        return 2;
    }
    if(get_io(20237)&&is_conveyor_ready(1)){
        last_pick = 1;
        return 1;
    }
    if(last_result){//grip succeed
        if(get_b_val(16)==0){//nothing in hand
            if(get_count(1)||get_count(2)){//has fixture
                if(get_count(1)==1&&is_conveyor_ready(1)){
                    last_pick=1;
                    return 1;
                }
                if(get_count(2)==1&&is_conveyor_ready(2)){
                    last_pick=2;
                    return 2;
                }
                if(get_count(1)==2&&is_conveyor_ready(1)){
                    last_pick=1;
                    return 1;
                }
                if(get_count(2)==2&&is_conveyor_ready(2)){
                    last_pick=2;
                    return 2;
                }
            }else{
                if(last_pick==1&&is_conveyor_ready(2)){
                    last_pick=2;
                    return 2;
                }
                if(last_pick==2&&is_conveyor_ready(1)){
                    last_pick=1;
                    return 1;
                }
            }
        }
        if(get_b_val(16)==1){//part1 in hand
            if(get_count(1)==1&&is_conveyor_ready(2)){
                last_pick = 2;
                return 2;
            }
            if(get_count(1)==2&&is_conveyor_ready(1)){
                last_pick = 1;
                return 1;
            }
            if(get_count(1)==0&&is_conveyor_ready(1)){
                last_pick = 1;
                return 1;
            }
        }
        if(get_b_val(16) == 2){//part2 in hand
            if(get_count(2)==1&&is_conveyor_ready(1)){
                last_pick = 1;
                return 1;
            }
            if(get_count(2)==2&&is_conveyor_ready(2)){
                last_pick = 2;
                return 2;
            if(get_count(2)==0&&is_conveyor_ready(2)){
                last_pick = 2;
                return 2;
            }
            }
        }
    }else{//grip failed
        if(last_pick==1){//last cycle pick part1
            if(is_conveyor_ready(1)){
                return 1;
            }
            if(get_count(2)&&is_conveyor_ready(2)){
                last_pick=2;
                return 2;
            }
        }
        if(last_pick==2){//last cycle pick part2
            if(is_conveyor_ready(2)){
                return 2;
            }
            if(get_count(1)&&is_conveyor_ready(1)){
                last_pick=1;
                return 1;
            }
        }
    }
    return 255;
}

int conveyor_moved(int product){
    if(product == 1){
        return get_io(20233);
   }
    if(product == 2){
        return get_io(20234);
   }
   return 0;
}

void reset_moved(int product){
    if(product == 1){
        set_io(10251, 1);
        waitSec(0.5);
        set_io(10251, 0);
   }
    if(product == 2){
        set_io(10252, 1);
        waitSec(0.5);
        set_io(10252, 0);
   }
}

void move_conveyor(int product){
    if(product == 1){
        set_io(10244, 1);
        waitSec(0.5);
        set_io(10244, 0);
   }
    if(product == 2){
        set_io(10247, 1);
        waitSec(0.5);
        set_io(10247, 0);
   }
}

void beforescan(int product){
    if(product == 1){
        set_io(10242, 1);
        waitSec(0.5);
        set_io(10242, 0);
   }
    if(product == 2){
        set_io(10245, 1);
        waitSec(0.5);
        set_io(10245, 0);
   }
}

void afterpick(int product){
    if(product == 1){
        set_io(10243, 1);
        waitSec(0.5);
        set_io(10243, 0);
   }
    if(product == 2){
        set_io(10246, 1);
        waitSec(0.5);
        set_io(10246, 0);
   }
}

void t1_main(int arg1, int arg2)
{    
    int product_id = 0;
    int result = 0;
    int bin_id = 0, next_bin = 0;
    T1_STATUS = T1_RUNNING;
    
    t1_log("T1 start up");
	T1_wakeUp();
    mpTask1Reset();
    
    if (!scp) {
        scp = mpMalloc(sizeof(IScp));
        }
    if (!yrc){
        yrc = mpMalloc(sizeof(Robot));
        }
    get_ip();
    get_port();
    get_scape_frame();
    create_robot(yrc);

    // get a scape interface
    if (init_robot(yrc,scp) != 0) end_task1();
    while(1)
    {   
        t1_log("t1 wait choice"); 
        switch(getUserChoice())
        {
            case CALIBRATION_10:
                robot_current_working = ROBOT_IN_CALIB;
                scp->scp_calibration();
                break;
            case BP_INITIALIZE_20:
                reset_all_bins();
                scp->scp_initialize(getGroupId(),getNumOfProducts());
                break;
            case SCAPE_PICK_30:
                robot_current_working = ROBOT_IN_BIN_PICKING;
                product_id = getProductId();
                set_return_val(scp->scp_pick(&bins[product_id],isRescanNeed(),getPickCfg()));
                set_bin_height(bins[product_id].remain_parts_height_mm);
                break;
            case START_BIN_SCAN_40:
                scp->scp_start_scan(&bins[getProductId()]);
                break;
            case START_HS_RECOG_50:
                scp->scp_start_handling_station_recog(&bins[getProductId()]);
                break;
            case 120: // TUOPU
                put_b_val(0, 15);
                put_b_val(0, 16);
                put_b_val(0, 25);
                put_b_val(0, 26);
                put_b_val(0, 51);
                put_b_val(0, 52);
                robot_current_working = 555;
                while(get_b_val(61) != 1){
                    product_id = get_tp_bin_to_pick();
                    put_b_val(product_id, 15);//tell inform next bin
                    if (product_id == 1 || product_id == 2){
                        put_b_val(1, 25);//tell inform start next cycle
                        short forcescan;
                        forcescan = conveyor_moved(product_id);//conveyor moved and force scan
                        if(forcescan) reset_moved(product_id);//reset conveyor moved signal
                        beforescan(product_id);//lock conveyor
                        if (get_b_val(16)){
                            waitSec(1);//scan delay
                        }
                        result = scp->TPPick(&bins[product_id-1], forcescan);//pick prepare
                        put_b_val(0, 15);//reset current bin
                        while (1)//wait tp core
                        {
                            if(get_b_val(26)){
                                put_b_val(0, 26);
                                break;
                            }
                            waitSec(0.01);
                        }
                        
                        if(result<0){//bin empty
                            last_result = 0;
                            move_conveyor(product_id);
                        }
                        else{//pick succeed
                            last_result = 1;
                            afterpick(product_id);
                        }
                    }
                    t1_log("120 loop...");
                    hold_T1_if_needed();
                }
                break;
            default:
                break;
        }
        
        put_b_val(1,WAIT_USER_CHOICE_B_71);
        waitSec(0.01);
        hold_T1_if_needed();
    }
}