/* 
2022-05-31
update bin left parts height - I66
2022-11-19
修正了一个bug,getUserChoice, 只有当读取到的选择不为0时候，在获取完用户选择时才会重置选择为0 否则有可能
在mp程序刚读完选择，而且此时选择为0，恰好在这时候，TP发送过来新的选择，然而此时的选择已经被mp程序重置为0（0即为空选择）
所以会导致mp程序无法收到用户选择，然而TP程序也会在core中循环等待任务，mp程序也会一直循环等待用户选择，导致程序无法正常工作。
2022-12-01
新松更新了立库，所以80站4个供料位置换箱操作均不影响剩余箱子的抓取，所以做出相应更新，提高换箱的效率
2023-02-09
scapeBase.c 更新，stopshowmsg 函数，如果输出的errorcode 余100 等于20 表示相机断开连结，将设置一个输出信号
2023-05-08
t1.c 更新，解决在切换一轮抓取时候等待的问题， 由于PLC无法及时给出下一轮需要抓取的零件，因此在out程序中主动的默认开始扫描下一轮的第一个零件
在住循环 case 110: // lear 80 GP25 improve cycle time中
当搜索不到需要提前扫描的箱子时候，默认开始扫描下一轮的第一个零件修改代码如下：
	// Set B015 255
	put_b_val(255, 15);
	scp->scp_start_scan(&bins[0]);
*/
// this task is the entrance of motoplus application
// this task manage T1 start, resume etc
#include "T.h"
#define T1_RESET_B_99 99
#define T2_RUN_STATUS_S_52 52
#define T2_LOOP_S_53 53

TASK_STATUS T1_STATUS;
short T2_RESET_T1 = 0;
int T1_ID = 0;
int T2_ID = 0;

extern void t1_main(int arg1, int arg2);

//Set application information.
static int SetApplicationInfo(void)
{
	MP_APPINFO_SEND_DATA    sData;
	MP_STD_RSP_DATA         rData;
	int                     rc;

	memset(&sData, 0x00, sizeof(sData));
	memset(&rData, 0x00, sizeof(rData));

	strncpy(sData.AppName,  "scape bin-picking",  MP_MAX_APP_NAME);
	strncpy(sData.Version,  "2023.07.08",         MP_MAX_APP_VERSION);
	strncpy(sData.Comment,  "SCAPE - YRC1000 V3", MP_MAX_APP_COMMENT);

	rc = mpApplicationInfoNotify(&sData, &rData);
	return rc;
}

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
	while(1)
	{
		if(mpPutUserVars(&varInfo) == 0) break;
		waitSec(0.001);
	}
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
		waitSec(0.001);
	}
}

static char get_b_val(unsigned short idx)
{
	MP_USR_VAR_INFO varInfo;
	
	memset(&varInfo, 0, sizeof(varInfo));
	varInfo.var_type = MP_VAR_B;
	varInfo.var_no = idx;
	while(1)
	{
		if(mpGetUserVars(&varInfo) == 0) break;
		waitSec(0.001);
	}
	return varInfo.val.b;
}

static void T2_wakeUp()
{
	//程序刚启动后，因系统访问变量区域，在mpPutVarData()、
	//mpPutSVarInfo()、mpPutPosVarData() 等的变量区域进行写入
	//的函数可能会返回错误。因此，需要任务自动启动的程序，请
	//设计一段重复读取变量的结构，直到没有错误返回。

	waitSec(10);

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

void t2_log(char* msg)
{
	put_s_val(msg,T2_RUN_STATUS_S_52);
}

void t2_main(int arg1, int arg2)
{
	// monitor task 1, and recreate task1 when necessary
	int counter = 0;
	char cmd = 0;
	MP_SERVO_POWER_RSP_DATA powerOn;

	long idx = 0;
	T2_wakeUp();
	t2_log("t2 start up.");
	while(1)
	{
		idx++;
		// restart task 1 procedure
		cmd = get_b_val(T1_RESET_B_99);
		if (cmd == 99) counter = 1;
		if (cmd == 4 && counter == 1) counter = 2;
		if (cmd == 3 && counter == 2) counter = 3;
		if (cmd == 2 && counter == 3) counter = 4;
		if (cmd == 1 && counter == 4) counter = 5;
		if (cmd == 0 && counter == 5)
		{
			counter = 0;
			t2_log("t2 restart t1.");
			if (T1_STATUS == T1_RUNNING) 
			{	
				t2_log("t2 wait t1 suspended.");
				T2_RESET_T1 = 1;
				waitSec(3);
				T2_RESET_T1 = 0;
				// wait T1_STATUS == T1_SUSPENDED?
			}
			if(T1_STATUS != T1_DELETED)
			{
				t2_log("t2 delete t1.");
				mpDeleteTask(T1_ID);
				T1_STATUS = T1_DELETED;
			}
			waitSec(0.1);
			if (T1_STATUS == T1_DELETED)
			{	
				t2_log("t2 create t1.");
				T1_STATUS = T1_CREATED;
				T1_ID = mpCreateTask(MP_PRI_TIME_NORMAL, MP_STACK_SIZE, (FUNCPTR)t1_main,
						0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
				waitSec(0.1);
				t2_log("t2 create t1 success");
				waitSec(0.2);
				if(T1_STATUS != T1_RUNNING)
				{
					T1_STATUS = T1_CREATE_FAILED;
				}
			}
		}

		// resume mpTask1 if t1 is suspended and motor on

		mpGetServoPower(&powerOn);

		if ((powerOn.sServoPower == 1) && (T1_STATUS == T1_SUSPENDED) && T2_RESET_T1 == 0)
		{
			t2_log("t2 resume t1.");
			mpTaskResume(T1_ID);
			waitSec(0.5);
			if(T1_STATUS != T1_RUNNING) T1_STATUS = T1_RESUME_FAILED;
		}

		// prove mpTask2 is alive...
		if (idx == 1)
		{
			put_s_val(".",T2_LOOP_S_53);
		}
		if (idx == 50)
		{
			put_s_val("...",T2_LOOP_S_53);
		}
		if (idx == 100)
		{
			idx = 0;
		}

		waitSec(0.01);
	}
	
}

void mpUsrRoot(int arg1, int arg2, int arg3, int arg4, int arg5, int arg6, int arg7, int arg8, int arg9, int arg10)
{
	int rc;

	T2_RESET_T1 = 0;
	T1_ID = 0;
	T2_ID = 0;
	T1_STATUS = T1_DELETED;
	
	T2_ID = mpCreateTask(MP_PRI_TIME_NORMAL, MP_STACK_SIZE, (FUNCPTR)t2_main,
						arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9, arg10);

	//Set application information.
	rc = SetApplicationInfo();
	
	//sem_id = mpSemBCreate(SEM_Q_FIFO, SEM_EMPTY);
	//Ends the initialization task.
	mpExitUsrRoot;
}
