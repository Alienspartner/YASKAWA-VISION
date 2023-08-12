
#include "IScpRobot.h"
#include "stdarg.h"
#include "stdlib.h"
#include "string.h"

#define SVAR_LEN 15
#define TEMP_DATA_LEN 50
#define true 1
#define false 0

typedef struct scpTask
{
    short targetType;

    short triggerCamera;

    short task_was_executed;

    int frameID;

    ScapeTask user_task;

} Scape_Task_Internal;

typedef struct scpProduct
{
    unsigned int product_group_id;

    unsigned int product_id;

    unsigned int cycle_index;

    unsigned int parts_bin_picked;

    unsigned int total_zones_on_hs;

    unsigned int parts_on_hs;

    int grip_family_id;

    short product_init_done;

    short bin_is_empty;

    short bp_use_tool_sensor;

    short bp_acq_needed;

    short bp_calib_needed;

    short use_oc;

    short use_bp;

    short layer_done;

    short oc_use_tool_cam;

    short oc_recog_started;

    short oc_recog_success;

    short oc_part_was_picked;

    short hs_is_full;

    short height_is_initialized;

    short part_placed_on_hs;

} Scp_Product;

static Scp_Product product_data[MAX_PRODUCT_NUM_IN_GROUP];
Robot *scapeRobot = NULL;
static int BIN_GIVE_UP_TIMES = 1;

//*******************************************
//             SCAPE UTILS
//*******************************************
static void stopAndShowErr(char *);

static void setDO_CameraLost()
{
    MP_IO_DATA* sData;
    LONG num = 1;

    sData->ulAddr = 164;
    sData->ulValue = 1;

    mpWriteIO(sData,num);
}

static long dtl(double a)
{
    if (a >= 0)
        return (long)(a + 0.5);
    return (long)(a - 0.5);
}

static int int_pow(int n)
{
    int temp = 1, i;
    for (i = 1; i <= n; i++)
        temp = temp * 10;
    return temp;
}

static char get_b_val(unsigned short idx)
{
    MP_USR_VAR_INFO varInfo1, varInfo2;
    unsigned short rc1 = 0;
    
    memset(&varInfo1, 0, sizeof(varInfo1));
    varInfo1.var_type = MP_VAR_B;
    varInfo1.var_no = idx;

    memset(&varInfo2, 0, sizeof(varInfo2));
    varInfo2.var_type = MP_VAR_B;
    varInfo2.var_no = idx;

    rc1 = mpGetUserVars(&varInfo1);

    return varInfo1.val.b;
}

static char *itos(int i, char *const sVar)
{
    short bits = 1;
    short counter = 1;
    int temp;

    temp = i;

    memset(sVar, '\0', SVAR_LEN);

    while (abs(temp) / 10 > 0)
    {
        temp = abs(temp) / 10;
        bits++;
    }

    if (bits >= SVAR_LEN)
        stopAndShowErr("itos svar too long");
    if (i < 0)
        sVar[0] = '-';

    for (counter = 1; counter <= bits; counter++)
    {
        if (i >= 0)
        {
            sVar[counter - 1] = (abs(i) % int_pow(bits - counter + 1)) / int_pow(bits - counter) + '0';
            sVar[counter] = '\0';
        }
        else
        {
            sVar[counter] = (abs(i) % int_pow(bits - counter + 1)) / int_pow(bits - counter) + '0';
            sVar[counter + 1] = '\0';
        }
    }

    return sVar;
}

static char *ltos(long i, char *const sVar)
{
    short bits = 1;
    short counter = 1;
    long temp;

    temp = i;

    memset(sVar, '\0', SVAR_LEN);

    while (abs(temp) / 10 > 0)
    {
        temp = abs(temp) / 10;
        bits++;
    }

    if (i < 0)
        sVar[0] = '-';
    if (bits >= SVAR_LEN)
        stopAndShowErr("ltos svar too long");
    for (counter = 1; counter <= bits; counter++)
    {
        if (i >= 0)
        {
            sVar[counter - 1] = (abs(i) % int_pow(bits - counter + 1)) / int_pow(bits - counter) + '0';
            sVar[counter] = '\0';
        }
        else
        {
            sVar[counter] = (abs(i) % int_pow(bits - counter + 1)) / int_pow(bits - counter) + '0';
            sVar[counter + 1] = '\0';
        }
    }

    return sVar;
}

static char *getStr(char *const raw, int n, ...)
{
    char sVar[SVAR_LEN + 1];
    char result[BUFF_LEN + 1];
    char temp[BUFF_LEN + 1];
    char *ret, *copy;

    va_list argptr;
    va_start(argptr, n);

    memset(result, '\0', BUFF_LEN + 1);
    copy = raw;
    while (1)
    {
        ret = strchr(copy, '%');
        if (!ret || strlen(ret) < 2)
        {
            if (strlen(copy) > 0)
                strcat(result, copy);
            break;
        }
        else
        {
            switch (*(ret + 1))
            {
            case 'd':
                memset(temp, '\0', BUFF_LEN + 1);
                memcpy(temp, copy, strlen(copy) - strlen(ret));
                strcat(result, temp);
                strcat(result, itos(va_arg(argptr, int), sVar));
                if (strlen(ret) > 2)
                    copy = ret + 2;
                break;
            case 's':
                memset(temp, '\0', BUFF_LEN + 1);
                memcpy(temp, copy, strlen(copy) - strlen(ret));
                strcat(result, temp);
                strcat(result, va_arg(argptr, char *));
                if (strlen(ret) > 2)
                    copy = ret + 2;
                break;
            case 'l':
                memset(temp, '\0', BUFF_LEN + 1);
                memcpy(temp, copy, strlen(copy) - strlen(ret));
                strcat(result, temp);
                strcat(result, ltos(va_arg(argptr, long), sVar));
                if (strlen(ret) > 2)
                    copy = ret + 2;
                break;
            default:
                strcat(result, ret);
                break;
            }
        }
    }
    va_end(argptr);
    memcpy(raw, result, strlen(result));
    *(raw + strlen(result)) = '\0';
    return raw;
}

void waitSeconds(float sec)
{
    scapeRobot->fnWaitSec(sec);
}

//*******************************************
//             scape messages
//*******************************************
static int getMsg(int iMsgCode, int iLanguage, char st_Msg[BUFF_LEN])
{
    memset(st_Msg, '\0', BUFF_LEN);

    if (_dbg_SCAPE_)
        printf("\n scp getMsg, msgCode: %d", iMsgCode);
    switch (iMsgCode)
    {
    case 0:
        strcpy(st_Msg, " ");
        break;
    case 101:
        strcpy(st_Msg, "SCS <ERROR> returned");
    case 901:
        strcpy(st_Msg, "SCAPE has been restarted. Please re-start robot program.");
        break;
    case 902:
        strcpy(st_Msg, "SCAPE has been stopped. Please re-start SBPM and robot program.");
        break;
    case 903:
        strcpy(st_Msg, "SCAPE doesnt answer. Make sure that SCAPE BP Manager is running");
        break;
    case 1000:
        strcpy(st_Msg, "StartOfCycle: Success");
        break;
    case 1999:
        strcpy(st_Msg, "StartOfCycle: Unsupported return code");
        break;
        // Codes from PickObject
    case 2000:
        strcpy(st_Msg, "PickObject: Success");
        break;
    case 2001:
        strcpy(st_Msg, "PickObject: Unknown product index.");
        break;
    case 2002:
        strcpy(st_Msg, "PickObject: SCAPE could not detect any remaining parts.");
        break;
    case 2003:
        strcpy(st_Msg, "PickObject: Height Initialization has not been performed.");
        break;
    case 2004:
        strcpy(st_Msg, "PickObject: Gripping failed 20 times in a row.");
        break;
    case 2010:
        strcpy(st_Msg, "PickObject: SCAPE Error. Check SCAPE Computer Monitor.");
        break;
    case 2020:
        strcpy(st_Msg, "PickObject: Camera connection error");
        break;
    case 2021:
        strcpy(st_Msg, "PickObject: Camera images too bright");
        break;
    case 2022:
        strcpy(st_Msg, "PickObject: Camera images too dark");
        break;
    case 2023:
        strcpy(st_Msg, "PickObject: Sliding scanner error");
        break;
    case 2024:
        strcpy(st_Msg, "PickObject: Empty point cloud 3 times in a row");
        break;
    case 2999:
        strcpy(st_Msg, "PickObject: Unsupported return code");
        break;

        // Codes from RegripAtHandlingStation
    case 3000:
        strcpy(st_Msg, "RegripAtHS: Success");
        break;
    case 3001:
        strcpy(st_Msg, "RegripAtHS: Unknown product index");
        break;
    case 3002:
        strcpy(st_Msg, "RegripAtHS: No part was recognized");
        break;
    case 3003:
        strcpy(st_Msg, "RegripAtHS: Gripping failed");
        break;
    case 3010:
        strcpy(st_Msg, "RegripAtHS: SCAPE Error. Check SCAPE Computer Monitor.");
        break;
    case 3999:
        strcpy(st_Msg, "RegripAtHS: Unsupported return code");
        break;

        // Codes from GetPlacePose
    case 4000:
        strcpy(st_Msg, "GetPlacePose: Success");
        break;
    case 4001:
        strcpy(st_Msg, "GetPlacePose: Unknown product");
        break;
    case 4002:
        strcpy(st_Msg, "GetPlacePose: No place pose is available");
        break;
    case 4003:
        strcpy(st_Msg, "GetPlacePose: Place pose is undefined in the Grip Manager.");
        break;
    case 4999:
        strcpy(st_Msg, "GetPlacePose: Unsupported return code");
        break;

        // Codes from PerformHeightInitialization
    case 5000:
        strcpy(st_Msg, "PerformHeightInit: Success");
        break;
    case 5001:
        strcpy(st_Msg, "PerformHeightInit: Unknown product");
        break;
    case 5002:
        strcpy(st_Msg, "PerformHeightInit: Could not determine height");
        break;
    case 5010:
        strcpy(st_Msg, "PerformHeightInit: SCAPE Error. Check SCAPE Computer Monitor.");
        break;
    case 5020:
        strcpy(st_Msg, "PerformHeightInit: Camera connection error");
        break;
    case 5021:
        strcpy(st_Msg, "PerformHeightInit: Images were too dark");
        break;
    case 5022:
        strcpy(st_Msg, "PerformHeightInit: Sliding Scanner Error");
        break;
    case 5023:
        strcpy(st_Msg, "PerformHeightInit: 3D Scanner found no 3D points");
        break;
    case 5999:
        strcpy(st_Msg, "PerformHeightInit: Unsupported return code");
        break;

        // Codes from ImageAcqRequired
    case 6000:
        strcpy(st_Msg, "ImageAcqRequired: Success");
        break;
    case 6001:
        strcpy(st_Msg, "ImageAcqRequired: Unknown product");
        break;
    case 6999:
        strcpy(st_Msg, "ImageAcqRequired: Unsupported return code");
        break;

        // Codes from AcquireImages
    case 7000:
        strcpy(st_Msg, "AcquireImages: Success");
        break;
    case 7001:
        strcpy(st_Msg, "AcquireImages: Unknown product index.");
        break;
    case 7010:
        strcpy(st_Msg, "AcquireImages: SCAPE Error. Check SCAPE Computer Monitor.");
        break;
    case 7999:
        strcpy(st_Msg, "AcquireImages: Unsupported return code");
        break;

        // Codes from GetReturnValue
    case 11011:
        strcpy(st_Msg, "GetReturnValue: Command Index not supported.");
        break;
    case 11012:
        strcpy(st_Msg, "GetReturnValue: No return value available");
        break;
    case 11999:
        strcpy(st_Msg, "GetReturnValue: Unsupported return code");
        break;

        // Codes from GetSensorType
    case 12000:
        strcpy(st_Msg, "GetSensorType: Success.");
        break;
    case 12001:
        strcpy(st_Msg, "GetSensorType: Unknown product index.");
        break;
    case 12002:
        strcpy(st_Msg, "GetSensorType: Product does not use OC");
        break;
    case 12010:
        strcpy(st_Msg, "GetSensorType: SCAPE Error. Check SCAPE Computer Monitor.");
        break;
    case 12999:
        strcpy(st_Msg, "GetSensorType: Unsupported return code");
        break;

        // Codes from RecognizeAtHandlingStation
    case 13000:
        strcpy(st_Msg, "RecognizeAtHS: Success");
        break;
    case 13001:
        strcpy(st_Msg, "RecognizeAtHS: Unknown product index");
        break;
    case 13002:
        strcpy(st_Msg, "RecognizeAtHS: No part was put on handling station");
        break;
    case 13003:
        strcpy(st_Msg, "RecognizeAtHS: No part was recognized");
        break;
    case 13004:
        strcpy(st_Msg, "RecognizeAtHS: Too much background noise");
        break;
    case 13008:
        strcpy(st_Msg, "RecognizeAtHS: Part was moving for too long");
        break;
    case 13010:
        strcpy(st_Msg, "RecognizeAtHS: SCAPE Error. Check SCAPE Computer Monitor.");
        break;
    case 13020:
        strcpy(st_Msg, "RecognizeAtHS: Camera connection error.");
        break;
    case 13021:
        strcpy(st_Msg, "RecognizeAtHS: Images too bright.");
        break;
    case 13022:
        strcpy(st_Msg, "RecognizeAtHS: Images too dark.");
        break;
    case 13999:
        strcpy(st_Msg, "RecognizeAtHS: Unsupported return code");
        break;

        // Codes from PlacePartOnHandlingStation
    case 14000:
        strcpy(st_Msg, "PlacePartOnHs: Success.");
        break;
    case 14001:
        strcpy(st_Msg, "PlacePartOnHs: Unknown product index.");
        break;
    case 14002:
        strcpy(st_Msg, "PlacePartOnHs: No part was bin-picked");
        break;
    case 14003:
        strcpy(st_Msg, "PlacePartOnHs: No part was in the gripper at HS");
        break;
    case 14004:
        strcpy(st_Msg, "PlacePartOnHs: Handling station is already full!");
        break;
    case 14005:
        strcpy(st_Msg, "PlacePartOnHs: Place pose is undefined in the Grip Manager!");
        break;
    case 14006:
        strcpy(st_Msg, "PlacePartOnHs: Place pose is not relative to Handling Station!");
        break;
    case 14010:
        strcpy(st_Msg, "PlacePartOnHs: SCAPE Error. Check SCAPE Computer Monitor.");
        break;
    case 14999:
        strcpy(st_Msg, "PlacePartOnHs: Unsupported return code");
        break;

        // Codes from SetObjectHeightInBin
    case 15000:
        strcpy(st_Msg, "SetObjHeightInBin: Success.");
        break;
    case 15001:
        strcpy(st_Msg, "SetObjHeightInBin: Unknown product index.");
        break;
    case 15999:
        strcpy(st_Msg, "SetObjHeightInBin: Unsupported return code");
        break;

        // Codes from GetObjectHeightInBin
    case 16000:
        strcpy(st_Msg, "GetObjHeightInBin: Success.");
        break;
    case 16001:
        strcpy(st_Msg, "GetObjHeightInBin: Unknown product index.");
        break;
    case 16002:
        strcpy(st_Msg, "GetObjHeightInBin: Height init has not been performed, height is unknown.");
        break;
    case 16009:
        strcpy(st_Msg, "GetObjHeightInBin: Das geladene Produkt hat keine Bin-Sessions");
        break;
    case 16010:
        strcpy(st_Msg, "GetObjHeightInBin: SCAPE Error. Check SCAPE Computer Monitor.");
        break;
    case 16999:
        strcpy(st_Msg, "GetObjHeightInBin: Unsupported return code");
        break;

        // Codes from HandlingStationWasCleared
    case 17000:
        strcpy(st_Msg, "HandlingStationWasCleared: Success.");
        break;
    case 17001:
        strcpy(st_Msg, "HandlingStationWasCleared: Unknown product index.");
        break;
    case 17002:
        strcpy(st_Msg, "HandlingStationWasCleared: Object get stuck in the gripper.");
        break;
    case 17010:
        strcpy(st_Msg, "HandlingStationWasCleared: SCAPE Error. Check SCAPE Computer Monitor.");
        break;
    case 17999:
        strcpy(st_Msg, "HandlingStationWasCleared: Unsupported return code");
        break;

        // Codes from PlaceObject
    case 18000:
        strcpy(st_Msg, "PlaceObject: Success.");
        break;
    case 18001:
        strcpy(st_Msg, "PlaceObject: Unknown product index.");
        break;
    case 18002:
        strcpy(st_Msg, "PlaceObject: No part was bin-picked");
        break;
    case 18004:
        strcpy(st_Msg, "PlaceObject: Handling station is already full!");
        break;
    case 18005:
        strcpy(st_Msg, "PlaceObject: Place pose is undefined in the Grip Manager!");
        break;
    case 18010:
        strcpy(st_Msg, "PlaceObject: SCAPE Error. Check SCAPE Computer Monitor.");
        break;
    case 18999:
        strcpy(st_Msg, "PlaceObject: Unsupported return code");
        break;

        // Codes from InitializeProductGroup
    case 22000:
        strcpy(st_Msg, "InitProductGroup: Success");
        break;
    case 22001:
        strcpy(st_Msg, "InitProductGroup: Unknown group index or failed loading group");
        break;
    case 22020:
        strcpy(st_Msg, "InitProductGroup: Camera connection error");
        break;
    case 22999:
        strcpy(st_Msg, "InitProductGroup: Unsupported return code");
        break;

        // Codes from LogValues
    case 23000:
        strcpy(st_Msg, "LogValues: Success");
        break;
    case 23999:
        strcpy(st_Msg, "LogValues: Unsupported return code");
        break;

        // Codes from GetGripperOffset
    case 24000:
        strcpy(st_Msg, "GetGripperOffset: Success");
        break;
    case 24003:
        strcpy(st_Msg, "GetGripperOffset: Invalid Toolunit ID");
        break;
    case 24004:
        strcpy(st_Msg, "GetGripperOffset: Invalid Gripper ID");
        break;
    case 24999:
        strcpy(st_Msg, "GetGripperOffset: Unsupported return code");
        break;

        // Codes from GetPickZoneInfo
    case 26000:
        strcpy(st_Msg, "GetPickZoneInfo: Success");
        break;
    case 26001:
        strcpy(st_Msg, "GetPickZoneInfo: Unknown product index.");
        break;
    case 26002:
        strcpy(st_Msg, "GetPickZoneInfo: Invalid pick zone or sub zone index.");
        break;
    case 26003:
        strcpy(st_Msg, "GetPickZoneInfo: Height initialization was not performed");
        break;
    case 26999:
        strcpy(st_Msg, "GetPickZoneInfo: Unsupported return code");
        break;

        // Codes from SetPickZoneInfo
    case 27000:
        strcpy(st_Msg, "SetPickZoneInfo: Success");
        break;
    case 27001:
        strcpy(st_Msg, "SetPickZoneInfo: Unknown product index.");
        break;
    case 27002:
        strcpy(st_Msg, "SetPickZoneInfo: Invalid pick zone or sub zone index.");
        break;
    case 27003:
        strcpy(st_Msg, "SetPickZoneInfo: Invalid property id");
        break;
    case 27004:
        strcpy(st_Msg, "SetPickZoneInfo: Invalid property value");
        break;
    case 27999:
        strcpy(st_Msg, "SetPickZoneInfo: Unsupported return code");
        break;

        // Codes from VerifyStationaryCameraCalibration
    case 30000:
        strcpy(st_Msg, "StatCamVerify: Success");
        break;
    case 30001:
        strcpy(st_Msg, "StatCamVerify: Error greater than 1.5 pixels. Calibrate Camera!");
        break;
    case 30002:
        strcpy(st_Msg, "StatCamVerify: Invalid Camera ID");
        break;
    case 30003:
        strcpy(st_Msg, "StatCamVerify: No calibration settings exist for this camera");
        break;
    case 30004:
        strcpy(st_Msg, "StatCamVerify: No self-test data was recorded for this camera");
        break;
    case 30005:
        strcpy(st_Msg, "StatCamVerify: The self-test mark could not be located");
        break;
    case 30010:
        strcpy(st_Msg, "StatCamVerify: SCAPE Error. Check SCAPE Computer Monitor.");
        break;
    case 30020:
        strcpy(st_Msg, "StatCamVerify: Camera connection error");
        break;
    case 30021:
        strcpy(st_Msg, "StatCamVerify: Camera images are too bright");
        break;
    case 30022:
        strcpy(st_Msg, "StatCamVerify: Camera images are too dark");
        break;
    case 30999:
        strcpy(st_Msg, "StatCamVerify: Unsupported return code");
        break;

        // Codes from VerifyToolCameraCalibration
    case 31000:
        strcpy(st_Msg, "ToolCamVerify: Success");
        break;
    case 31001:
        strcpy(st_Msg, "ToolCamVerify: Error > 3.0mm. Calibrate tool camera");
        break;
    case 31002:
        strcpy(st_Msg, "ToolCamVerify: Invalid CAST Station ID");
        break;
    case 31003:
        strcpy(st_Msg, "ToolCamVerify: Invalid Toolunit ID");
        break;
    case 31004:
        strcpy(st_Msg, "ToolCamVerify: Could not locate calibration mark");
        break;
    case 31010:
        strcpy(st_Msg, "ToolCamVerify: SCAPE Error. Check SCAPE Computer Monitor.");
        break;
    case 31020:
        strcpy(st_Msg, "ToolCamVerify: Camera connection error");
        break;
    case 31021:
        strcpy(st_Msg, "ToolCamVerify: Camera images are too bright");
        break;
    case 31022:
        strcpy(st_Msg, "ToolCamVerify: Camera images are too dark");
        break;
    case 31999:
        strcpy(st_Msg, "ToolCamVerify: Unsupported return code");
        break;

        // Codes from VerifyGridScannerCalibration
    case 35000:
        strcpy(st_Msg, "GridScannerVerify: Success");
        break;
    case 35001:
        strcpy(st_Msg, "GridScannerVerify: Error > 1.0 mm. Calibrate Grid Scanner");
        break;
    case 35002:
        strcpy(st_Msg, "GridScannerVerify: Error > 2.0 mm. Inspect and Calibrate Grid Scanner");
        break;
    case 35003:
        strcpy(st_Msg, "GridScannerVerify: Invalid CAST Station ID");
        break;
    case 35004:
        strcpy(st_Msg, "GridScannerVerify: Invalid Toolunit ID");
        break;
    case 35010:
        strcpy(st_Msg, "GridScannerVerify: SCAPE Error. Check SCAPE Computer Monitor.");
        break;
    case 35020:
        strcpy(st_Msg, "GridScannerVerify: Camera connection error");
        break;
    case 35021:
        strcpy(st_Msg, "GridScannerVerify: Camera images are too bright");
        break;
    case 35022:
        strcpy(st_Msg, "GridScannerVerify: Camera images are too dark");
        break;
    case 35023:
        strcpy(st_Msg, "GridScannerVerify: Few pattern points found. Projector may be broken.");
        break;
    case 35024:
        strcpy(st_Msg, "GridScannerVerify: Projector may be working sporadically. Check wires");
        break;
    case 35999:
        strcpy(st_Msg, "GridScannerVerify: Unsupported return code");
        break;

        // Codes from VerifySuctionGripperOffset
    case 36000:
        strcpy(st_Msg, "SucOffsetVerify: Success");
        break;
    case 36001:
        strcpy(st_Msg, "SucOffsetVerify: Gripper offset is no longer valid.");
        break;
    case 36002:
        strcpy(st_Msg, "SucOffsetVerify: Invalid CAST Station ID");
        break;
    case 36003:
        strcpy(st_Msg, "SucOffsetVerify: Invalid Toolunit ID");
        break;
    case 36004:
        strcpy(st_Msg, "SucOffsetVerify: Invalid suction gripper ID");
        break;
    case 36010:
        strcpy(st_Msg, "SucOffsetVerify: SCAPE Error. Check SCAPE Computer Monitor.");
        break;
    case 36999:
        strcpy(st_Msg, "SucOffsetVerify: Unsupported return code");
        break;

        // Codes from TestSuctionStrength
    case 37000:
        strcpy(st_Msg, "SucStrengthTest: Success");
        break;
    case 37001:
        strcpy(st_Msg, "SucStrengthTest: Suction is weak");
        break;
    case 37002:
        strcpy(st_Msg, "SucStrengthTest: Invalid CAST Station ID");
        break;
    case 37003:
        strcpy(st_Msg, "SucStrengthTest: Invalid Toolunit ID");
        break;
    case 37004:
        strcpy(st_Msg, "SucStrengthTest: Invalid suction gripper ID");
        break;
        ;
    case 37010:
        strcpy(st_Msg, "SucStrengthTest: SCAPE Error. Check SCAPE Computer Monitor.");
        break;
    case 37999:
        strcpy(st_Msg, "SucStrengthTest: Unsupported return code");
        break;

        // Codes from TestVacuumSensor
    case 38000:
        strcpy(st_Msg, "VacSensorTest: Success");
        break;
    case 38001:
        strcpy(st_Msg, "VacSensorTest: Sensor gives false high signal without suction");
        break;
    case 38002:
        strcpy(st_Msg, "VacSensorTest: Sensor gives false high signal when not at surface");
        break;
    case 38003:
        strcpy(st_Msg, "VacSensorTest: Sensor gives false low signal when at surface");
        break;
    case 38004:
        strcpy(st_Msg, "VacSensorTest: Invalid CAST Station ID");
        break;
    case 38005:
        strcpy(st_Msg, "VacSensorTest: Invalid Toolunit ID");
        break;
    case 38006:
        strcpy(st_Msg, "VacSensorTest: Invalid suction gripper ID");
        break;
    case 38010:
        strcpy(st_Msg, "VacSensorTest: SCAPE Error. Check SCAPE Computer Monitor.");
        break;
    case 38999:
        strcpy(st_Msg, "VacSensorTest: Unsupported return code");
        break;

        // Codes from TestFingerGripperSensor
    case 39000:
        strcpy(st_Msg, "FingerSensorTest: Success");
        break;
    case 39001:
        strcpy(st_Msg, "FingerSensorTest: Gripper did not respond when it was activated");
        break;
    case 39002:
        strcpy(st_Msg, "FingerSensorTest: Returned GRIP SUCCESSFUL when activated w/o part");
        break;
    case 39003:
        strcpy(st_Msg, "FingerSensorTest: Gripper did not respond when it was deactivated");
        break;
    case 39004:
        strcpy(st_Msg, "FingerSensorTest: Returned GRIP SUCCESSFUL when deactivated w/o part");
        break;
    case 39005:
        strcpy(st_Msg, "FingerSensorTest: Invalid Toolunit ID");
        break;
    case 39006:
        strcpy(st_Msg, "FingerSensorTest: Invalid finger gripper ID");
        break;
    case 39007:
        strcpy(st_Msg, "FingerSensorTest: Gripped with specified ID is not a finger gripper");
        break;
    case 39010:
        strcpy(st_Msg, "FingerSensorTest: SCAPE Error. Check SCAPE Computer Monitor.");
        break;
    case 39999:
        strcpy(st_Msg, "FingerSensorTest: Unsupported return code");
        break;

        // Codes from VerifyFingerGripperOffset
    case 40000:
        strcpy(st_Msg, "FingerOffsetVerify: Success");
        break;
    case 40001:
        strcpy(st_Msg, "FingerOffsetVerify: Measured offset is no longer correct");
        break;
    case 40002:
        strcpy(st_Msg, "FingerOffsetVerify: Invalid CAST Station ID");
        break;
    case 40003:
        strcpy(st_Msg, "FingerOffsetVerify: Invalid Toolunit ID");
        break;
    case 40004:
        strcpy(st_Msg, "FingerOffsetVerify: Invalid finger gripper ID");
        break;
    case 40005:
        strcpy(st_Msg, "FingerOffsetVerify: Gripped with specified ID is not a finger gripper");
        break;
    case 40010:
        strcpy(st_Msg, "FingerOffsetVerify: SCAPE Error. Check SCAPE Computer Monitor.");
        break;
    case 40999:
        strcpy(st_Msg, "FingerOffsetVerify: Unsupported return code");
        break;

        // Codes from VerifySlidingScannerCalibration
    case 41000:
        strcpy(st_Msg, "SlideScannerVerify: Success");
        break;
    case 41001:
        strcpy(st_Msg, "SlideScannerVerify: Calibration is no longer ok. Calibrate Scanner");
        break;
    case 41002:
        strcpy(st_Msg, "SlideScannerVerify: The specified Sliding Scanner ID is Invalid");
        break;
    case 41003:
        strcpy(st_Msg, "SlideScannerVerify: The specified Calibration Range ID is Invalid");
        break;
    case 41004:
        strcpy(st_Msg, "SlideScannerVerify: No calibration settings exist for this scanner");
        break;
    case 41005:
        strcpy(st_Msg, "SlideScannerVerify: Self-test data was not recorded during calibration");
        break;
    case 41006:
        strcpy(st_Msg, "SlideScannerVerify: Failed locating self-test mark during self-test");
        break;
    case 41010:
        strcpy(st_Msg, "SlideScannerVerify: SCAPE Error. Check SCAPE Computer Monitor.");
        break;
    case 41020:
        strcpy(st_Msg, "SlideScannerVerify: Scan failed. Check scanner");
        break;
    case 41021:
        strcpy(st_Msg, "SlideScannerVerify: No 3D points were found, laser may be broken");
        break;
    case 41999:
        strcpy(st_Msg, "SlideScannerVerify: Unsupported return code");
        break;

        // Codes from PlanAndMoveCartesian
    case 50000:
        strcpy(st_Msg, "PlanAndMoveCartesian: Success");
        break;
    case 50001:
        strcpy(st_Msg, "PlanAndMoveCartesian: Unknown product ID");
        break;
    case 50002:
        strcpy(st_Msg, "PlanAndMoveCartesian: Planning failed");
        break;
    case 50999:
        strcpy(st_Msg, "PlanAndMoveCartesian: Unsupported return code");
        break;

        // Codes from SetRobotStartPosition
    case 51000:
        strcpy(st_Msg, "SetRobotStartPos: Success");
        break;
    case 51001:
        strcpy(st_Msg, "SetRobotStartPos: Unknown product ID");
        break;
    case 51002:
        strcpy(st_Msg, "SetRobotStartPos: SCAPE Robot Kinematic model is not enabled/calibrated");
        break;
    case 51003:
        strcpy(st_Msg, "SetRobotStartPos: Position is outside SCAPE joint limits");
        break;
    case 51004:
        strcpy(st_Msg, "SetRobotStartPos: Position is in collision");
        break;
    case 51010:
        strcpy(st_Msg, "SetRobotStartPos: SCAPE Error. Check SCAPE Computer Monitor.");
        break;
    case 51999:
        strcpy(st_Msg, "SetRobotStartPos: Unsupported return code");
        break;

        // Codes from CalibrateStationaryCamera
    case 70000:
        strcpy(st_Msg, "StatCamCalib: Success");
        break;
    case 70001:
        strcpy(st_Msg, "StatCamCalib: Invalid camera ID");
        break;
    case 70002:
        strcpy(st_Msg, "StatCamCalib: Camera has never been calibrated. Calibrate with SCM.");
        break;
    case 70003:
        strcpy(st_Msg, "StatCamCalib: Detection of calibration mark failed too many times");
        break;
    case 70004:
        strcpy(st_Msg, "StatCamCalib: Calibration error was too big. Calibrate with SCM.");
        break;
    case 70005:
        strcpy(st_Msg, "StatCamCalib: Failed to adjust shutter values. Calibrate with SCM.");
        break;
    case 70010:
        strcpy(st_Msg, "StatCamCalib: SCAPE Error. Check SCAPE Computer Monitor.");
        break;
    case 70020:
        strcpy(st_Msg, "StatCamCalib: Camera connection error");
        break;
    case 70021:
        strcpy(st_Msg, "StatCamCalib: Camera images are too bright");
        break;
    case 70022:
        strcpy(st_Msg, "StatCamCalib: Camera images are too dark");
        break;
    case 70999:
        strcpy(st_Msg, "StatCamCalib: Unsupported return code");
        break;

        // Codes from CalibrateToolCamera
    case 71000:
        strcpy(st_Msg, "ToolCamCalib: Success");
        break;
    case 71001:
        strcpy(st_Msg, "ToolCamCalib: Invalid Toolunit ID");
        break;
    case 71002:
        strcpy(st_Msg, "ToolCamCalib: Camera has never been calibrated. Calibrate with SCM.");
        break;
    case 71003:
        strcpy(st_Msg, "ToolCamCalib: Detection of calibration mark failed too many times");
        break;
    case 71004:
        strcpy(st_Msg, "ToolCamCalib: Calibration error was too big. Calibrate with SCM.");
        break;
    case 71010:
        strcpy(st_Msg, "ToolCamCalib: SCAPE Error. Check SCAPE Computer Monitor.");
        break;
    case 71020:
        strcpy(st_Msg, "ToolCamCalib: Camera connection error");
        break;
    case 71021:
        strcpy(st_Msg, "ToolCamCalib: Camera images are too bright");
        break;
    case 71022:
        strcpy(st_Msg, "ToolCamCalib: Camera images are too dark");
        break;
    case 71999:
        strcpy(st_Msg, "ToolCamCalib: Unsupported return code");
        break;

        // Codes from CalibrateGridScanner
    case 74000:
        strcpy(st_Msg, "GridScannerCalib: Success");
        break;
    case 74001:
        strcpy(st_Msg, "GridScannerCalib: Invalid Toolunit ID");
        break;
    case 74002:
        strcpy(st_Msg, "GridScannerCalib: Grid Scanner was never calibrated. Calibrate with SCM.");
        break;
    case 74003:
        strcpy(st_Msg, "GridScannerCalib: Calibration error was too big. Calibrate with SCM.");
        break;
    case 74010:
        strcpy(st_Msg, "GridScannerCalib: SCAPE Error. Check SCAPE Computer Monitor.");
        break;
    case 74020:
        strcpy(st_Msg, "GridScannerCalib: Camera connection error");
        break;
    case 74021:
        strcpy(st_Msg, "GridScannerCalib: Camera images are too bright");
        break;
    case 74022:
        strcpy(st_Msg, "GridScannerCalib: Camera images are too dark");
        break;
    case 74023:
        strcpy(st_Msg, "GridScannerCalib: Few pattern points found. Projector may be broken.");
        break;
    case 74024:
        strcpy(st_Msg, "GridScannerCalib: Projector may be working sporadically. Check wires");
        break;
    case 74999:
        strcpy(st_Msg, "GridScannerCalib: Unsupported return code");
        break;

        // Codes from CalibrateSlidingScanner
    case 75000:
        strcpy(st_Msg, "SlideScannerCalib: Success");
        break;
    case 75001:
        strcpy(st_Msg, "SlideScannerCalib: The specified Sliding Scanner ID is Invalid");
        break;
    case 75002:
        strcpy(st_Msg, "SlideScannerCalib: The specified Calibration Range ID is Invalid");
        break;
    case 75003:
        strcpy(st_Msg, "SlideScannerCalib: Scanner was never calibrated. Calibrate with SCM.");
        break;
    case 75004:
        strcpy(st_Msg, "SlideScannerCalib: Detection of calibration mark failed too many times");
        break;
    case 75005:
        strcpy(st_Msg, "SlideScannerCalib: Calibration error was too big. Calibrate with SCM.");
        break;
    case 75010:
        strcpy(st_Msg, "SlideScannerCalib: SCAPE Error. Check SCAPE Computer Monitor.");
        break;
    case 75020:
        strcpy(st_Msg, "SlideScannerCalib: Camera connection error");
        break;
    case 75021:
        strcpy(st_Msg, "SlideScannerCalib: No 3D points were found, laser may be broken");
        break;
    case 75999:
        strcpy(st_Msg, "SlideScannerCalib: Unsupported return code");
        break;

        // Codes for AcquirePointCloud
    case 90000:
        strcpy(st_Msg, "Point cloud acquisition: Success");
        break;
    case 90001:
        strcpy(st_Msg, "Point cloud acquisition: Grid Scanner is not calibrated.");
        break;
    case 90002:
        strcpy(st_Msg, "Point cloud acquisition: Setting for raw acquisition not initialized.");
        break;
    case 90003:
        strcpy(st_Msg, "Point cloud acquisition: This SCAPE license does not support this program.");
        break;
    case 90010:
        strcpy(st_Msg, "Point cloud acquisition: SCAPE Error. Check SCAPE Computer Monitor.");
        break;
    case 90999:
        strcpy(st_Msg, "Point cloud acquisition: Unsupported return code");
        break;
    default:
        // not supported message code, -2 returned
        return -2;
    }
    return 0;
}

static void msgShow(char msg[BUFF_LEN], MsgLevel level)
{
    if (msg)
    {
        scapeRobot->fnShowMsg(msg, level);
    }
}

static void cleanMsg(void)
{
    scapeRobot->fnCleanMsg();
}

static void showCodeMsg(long msgCode, MsgLevel level)
{
    char msg[BUFF_LEN] = {0};

    // fetch message from scape message library
    if (getMsg(msgCode, 1, msg) != 0)
    {
        msgShow(getStr("err %d not found", 1, msgCode), Error);
    }
    // show message on robot
    msgShow(msg, level);
}

static void stopAndShowCodeMsg(long msgCode)
{
    int iEnglish = 1, i;
    char msg[BUFF_LEN] = {0};

    for (i = 0; i < MAX_PRODUCT_NUM_IN_GROUP; i++)
    {
        product_data[i].product_init_done = false;
    }

    // fetch message from scape message library
    while (1)
    {
        if (getMsg(msgCode, iEnglish, msg) != 0)
        {
            msgShow(getStr("err %d not found", 1, msgCode), Error);
        }
        else
        {
            msgShow(msg, Error);
        }
        if (/*ERROR CODE == ** 020 */(msgCode % 100 == 20)&&(msgCode > 1000))
        {
            setDO_CameraLost();
        }
        waitSeconds(10);
    }
}

static void stopAndShowErr(char error[BUFF_LEN])
{
    int i;
    for (i = 0; i < MAX_PRODUCT_NUM_IN_GROUP; i++)
    {
        product_data[i].product_init_done = false;
    }

    while (1)
    {
        msgShow(error, Error);
        waitSeconds(10);
    }
}

//*******************************************
//             scape communication
//*******************************************
static long tempData[TEMP_DATA_LEN] = {0};
static char recvBuff[BUFF_LEN + 1] = {0};
static int EC_IsReady(void);
static void setupGroups(void);
static void setVal(char *varToSet);

static void connectToScape(char *clientName)
{
    char cmd[BUFF_LEN + 1] = "SET_CLIENT_NAME ";

    strcat(cmd, clientName);
    // remove the old connection and reset scape
    scapeRobot->fnDisconnectToScape();
    scapeRobot->fnConnectToScape();
    scapeRobot->fnSendToScape(cmd);
    scapeRobot->fnReceiveFromScape(recvBuff);
    setupGroups();
    scapeRobot->fnSendToScape("SET ROBOT_STOP 1 ROBOT_TASK_READY 0 ROBOT_TASKS_LEFT 0 ROBOT_SIGNAL_READY 0;");
}

static void getVal(char *varToGet)
{
    char sVar[SVAR_LEN + 1] = {""};
    int counter = 0, var_counter = 0, sVar_counter = 0;

    // clean the buff
    // cleanBuff();
    setVal(varToGet);
    memset(tempData, 0, sizeof(tempData));
    memset(sVar, '\0', SVAR_LEN + 1);
    // parse received string
    if (_dbg_SCAPE_ == 1 || _dbg_SCAPE_ == 9)
        printf("\n getVal, recv string before parse: %s", recvBuff);
    while (1)
    {
        if (counter > strlen(recvBuff) && var_counter < TEMP_DATA_LEN)
        {
            return;
        }
        else
        {
            if (var_counter >= TEMP_DATA_LEN)
                stopAndShowErr("vars num too much!!");
        }
        if ((recvBuff[counter] == '-' && sVar_counter == 0) || (recvBuff[counter] >= '0' && recvBuff[counter] <= '9'))
        {
            if (sVar_counter >= SVAR_LEN)
                stopAndShowErr("recv svar too long");
            sVar[sVar_counter] = recvBuff[counter];
            sVar_counter++;
        }
        else
        {
            if (strlen(sVar) > 0)
            {
                tempData[var_counter] = atol(sVar);
                memset(sVar, '\0', SVAR_LEN + 1);
                sVar_counter = 0;
                var_counter++;
            }
        }
        counter++;
    }
}

static void setVal(char *varToSet)
{
    if (_dbg_SCAPE_ == 1 || _dbg_SCAPE_ == 9)
        printf("\n scp: setVal: %s\n", varToSet);

    // waitSeconds(0.01);

    scapeRobot->fnSendToScape(varToSet);

    memset(recvBuff, '\0', sizeof(recvBuff));

    // waitSeconds(0.001);

    scapeRobot->fnReceiveFromScape(recvBuff);

    if (_dbg_SCAPE_ == 1 || _dbg_SCAPE_ == 9)
        printf("\n scp: setVal received: %s\n", recvBuff);
    if (strlen(recvBuff) > 0)
    {
        if (strstr(recvBuff, "Error"))
        {
            // error returned from scape server, stop the program until reset
            stopAndShowCodeMsg(101);
        }
    }
}

static void waitForScapeReady(void)
{
    short isReady = false;
    // Make sure that the SCAPE system is active
    while (!isReady)
    {
        isReady = (EC_IsReady() == 0);
        if (!isReady)
        {
            showCodeMsg(903, Info);
        }
        else
        {
            cleanMsg();
            setVal("SET SCAPE_STATUS 0;");
            // Since SCAPE answered, we know that it is no longer busy. It is then safe to set ROBOT_STOP to 0
            setVal("SET ROBOT_STOP 0;");
            setVal("SET ROBOT_SUPPORTS_SCAPE_STATUS 1;");
            setVal("SET ROBOT_SUPPORTS_MOVING_GRIP_CHECK 1;");
            setVal("SET ROBOT_SUPPORTS_LAYER_PICKING_VERSION 3;");
            setVal("SET SCAPE_STATUS 0 ROBOT_TASK_READY 0 ROBOT_TASKS_LEFT 0;");
        }
    }
}

static void setupGroups(void)
{
    setVal("ADD_NAMES_TO_GROUP MTGROUP ROBOT_POSITION_X ROBOT_POSITION_Y ROBOT_POSITION_Z ROBOT_POSITION_A ROBOT_POSITION_B ROBOT_POSITION_C ROBOT_SPEED ROBOT_ACCELERATION;");
    setVal("ADD_NAMES_TO_GROUP MTGROUP ROBOT_USE_LINEAR_MOTION ROBOT_WORKCELL_STATE ROBOT_BLEND_DISTANCE ROBOT_TRIGGER_CAMERA ROBOT_STOP_COMPLETELY ROBOT_FRAME_ID;");
    setVal("ADD_NAMES_TO_GROUP MTGROUP ROBOT_TASKS_LEFT ROBOT_USE_GLOBAL_BEST_CONF ROBOT_WAIT_UNTIL_START ROBOT_USE_JOINT_VALUES;");
    setVal("ADD_NAMES_TO_GROUP MTGROUP ROBOT_JOINT_1_VALUE ROBOT_JOINT_2_VALUE ROBOT_JOINT_3_VALUE ROBOT_JOINT_4_VALUE ROBOT_JOINT_5_VALUE ROBOT_JOINT_6_VALUE;");
    setVal("ADD_NAMES_TO_GROUP MTGROUP ROBOT_JOINT_E1_VALUE ROBOT_JOINT_E2_VALUE ROBOT_TARGET_TYPE;");

    setVal("ADD_NAMES_TO_GROUP ETGROUP ROBOT_PROGRAM_ID ROBOT_PARAMETER_0 ROBOT_PARAMETER_1 ROBOT_PARAMETER_2 ROBOT_TASKS_LEFT;");
    setVal("ADD_NAMES_TO_GROUP STGROUP ROBOT_SLEEP_TIME ROBOT_TASKS_LEFT;");
    setVal("ADD_NAMES_TO_GROUP TCPPOSEGROUP ROBOT_POSITION_X ROBOT_POSITION_Y ROBOT_POSITION_Z ROBOT_POSITION_A ROBOT_POSITION_B ROBOT_POSITION_C;");
    setVal("ADD_NAMES_TO_GROUP PICKOBJSETGROUP ExtControlCommand ExtControlParamIn0 ExtControlParamIn1 ExtControlParamIn2;");
    setVal("ADD_NAMES_TO_GROUP PICKOBJGETGROUP ExtControlParamOut0 ExtControlParamOut1 ExtControlParamOut2 ExtControlParamOut3 ExtControlParamOut4 ExtControlParamOut5 ExtControlParamOut6 ExtControlParamOut7 ExtControlParamOut8 ExtControlParamOut9;");
    setVal("ADD_NAMES_TO_GROUP GETPLACEGROUP ExtControlParamOut0 ExtControlParamOut1 ExtControlParamOut2 ExtControlParamOut3 ExtControlParamOut4 ExtControlParamOut5 ExtControlParamOut6;");
    setVal("ADD_NAMES_TO_GROUP ISIOREADYGROUP ROBOT_SIGNAL_READY ROBOT_TASK_READY ROBOT_TASK_TYPE ExtControlCommand SCAPE_STATUS;");
    setVal("ADD_NAMES_TO_GROUP GRIPPEROFFSETGROUP ExtControlParamOut0 ExtControlParamOut1 ExtControlParamOut2 ExtControlParamOut3 ExtControlParamOut4 ExtControlParamOut5;");
    setVal("ADD_NAMES_TO_GROUP LOGVALUESGROUP ExtControlParamIn0 ExtControlParamIn1 ExtControlParamIn2 ExtControlParamIn3 ExtControlParamIn4 ExtControlParamIn5 ExtControlParamIn6 ExtControlParamIn7 ExtControlParamIn8 ExtControlParamOut9 ExtControlCommand;");
    setVal("ADD_NAMES_TO_GROUP REGRIPGROUP ExtControlCommand ExtControlParamIn0 ExtControlParamIn1 ExtControlParamIn2;");
    setVal("ADD_NAMES_TO_GROUP GETPICKZONEINFOGROUP ExtControlCommand ExtControlParamIn0 ExtControlParamIn1 ExtControlParamIn2;");
    setVal("ADD_NAMES_TO_GROUP SETPICKZONEINFOGROUP ExtControlCommand ExtControlParamIn0 ExtControlParamIn1 ExtControlParamIn2 ExtControlParamIn3 ExtControlParamIn4;");
    setVal("ADD_NAMES_TO_GROUP PLACEONHSSETGROUP ExtControlCommand ExtControlParamIn0 ExtControlParamIn1 ExtControlParamIn2;");
    setVal("ADD_NAMES_TO_GROUP TOOLCAMERAVERIFYGROUP ExtControlCommand ExtControlParamIn0 ExtControlParamIn1;");
    setVal("ADD_NAMES_TO_GROUP SUCTIONOFFSETGROUP ExtControlCommand ExtControlParamIn0 ExtControlParamIn1 ExtControlParamIn2;");
    setVal("ADD_NAMES_TO_GROUP SUCTIONSTRENGTHGROUP ExtControlCommand ExtControlParamIn0 ExtControlParamIn1 ExtControlParamIn2;");
    setVal("ADD_NAMES_TO_GROUP TESTVACUUMGROUP ExtControlCommand ExtControlParamIn0 ExtControlParamIn1 ExtControlParamIn2;");
    setVal("ADD_NAMES_TO_GROUP FINGEROFFSETGROUP ExtControlCommand ExtControlParamIn0 ExtControlParamIn1 ExtControlParamIn2;");
    setVal("ADD_NAMES_TO_GROUP ECAPI50 ExtControlCommand ExtControlParamIn0 ExtControlParamIn1 ExtControlParamIn2 ExtControlParamIn3 ExtControlParamIn4 ExtControlParamIn5 ExtControlParamIn6 ExtControlParamIn7 ExtControlParamIn8 ExtControlParamIn9;");
    setVal("ADD_NAMES_TO_GROUP ECAPI51 ExtControlCommand ExtControlParamIn0 ExtControlParamIn1 ExtControlParamIn2 ExtControlParamIn3 ExtControlParamIn4 ExtControlParamIn5 ExtControlParamIn6 ExtControlParamIn7 ExtControlParamIn8;");
}

//*******************************************
//             scpae EC_API
//*******************************************
static void os_core(short);
static int EC_WaitForCompletion(int iCommandID);

static int EC_AcquireImages(int product_id, short bForceAcquisition)
{
    char cmd[BUFF_LEN] = "SET ExtControlCommand 7 ExtControlParamIn0 %d ExtControlParamIn1 %d;";

    // Send the EC API command index, product index etc to SCAPE
    setVal(getStr(cmd, 2, product_id, bForceAcquisition));
    // Wait until SCAPE has carried out the EC API Command
    return EC_WaitForCompletion(7);
}

static int EC_AcquirePointCloud()
{
    // Send the EC API command index, product index etc to SCAPE
    setVal("SET ExtControlCommand 90;");
    os_core(true);
    // Wait until SCAPE has carried out the EC API Command
    return EC_WaitForCompletion(90);
}

static int EC_ClearProductSetup()
{
    // Send the EC API command index, product index etc to SCAPE
    setVal("SET ExtControlCommand 21;");
    os_core(true);
    // Wait until SCAPE has carried out the EC API Command
    return EC_WaitForCompletion(21);
}

static int EC_FingerOffsetVerify(int iSelfTestStationID, int iToolUnitID, int iGripperID)
{
    char cmd[BUFF_LEN] = "SET_GROUP_VALUES FINGEROFFSETGROUP 40 %d %d %d;";

    // Send the EC API command index, product index etc to SCAPE
    setVal(getStr(cmd, 3, iSelfTestStationID, iToolUnitID, iGripperID));

    os_core(true);
    // Wait until SCAPE has carried out the EC API Command
    return EC_WaitForCompletion(40);
}

static int EC_FingerSensorTest(int iToolUnitID, int iGripperID)
{
    char cmd[BUFF_LEN] = "SET ExtControlCommand 39 ExtControlParamIn0 %d ExtControlParamIn1 %d;";

    // Send the EC API command index, product index etc to SCAPE
    setVal(getStr(cmd, 2, iToolUnitID, iGripperID));

    os_core(true);
    // Wait until SCAPE has carried out the EC API Command
    return EC_WaitForCompletion(39);
}

static int EC_GetGripFamilyID(int *ID)
{
    int ret = 0;
    // Send the EC API command index, product index etc to SCAPE
    setVal("SET ExtControlCommand 10;");
    ret = EC_WaitForCompletion(10);
    getVal("GET ExtControlParamOut0;");
    *ID = (int)tempData[0];
    return ret;
}

static int EC_GetGripperOffset(int iToolUnitID, int iGripperID, double *gripperOffset)
{
    char cmd[BUFF_LEN] = "SET ExtControlCommand 24 ExtControlParamIn0 %d ExtControlParamIn1 %d;";
    int ret = 0;

    // Send the EC API command index, tool unit id and gripper id to SCAPE.
    setVal(getStr(cmd, 2, iToolUnitID, iGripperID));
    ret = EC_WaitForCompletion(24);
    getVal("GET_GROUP_VALUES GRIPPEROFFSETGROUP;");
    gripperOffset[0] = tempData[0] / 10.0;
    gripperOffset[1] = tempData[1] / 10.0;
    gripperOffset[2] = tempData[2] / 10.0;
    gripperOffset[3] = tempData[3] / 100.0;
    gripperOffset[4] = tempData[4] / 100.0;
    gripperOffset[5] = tempData[5] / 100.0;
    return ret;
}

static int EC_GetObjHeightInBin(int iproduct, unsigned int *objHeightInBin)
{
    char cmd[BUFF_LEN] = "SET ExtControlCommand 16 ExtControlParamIn0 %d;";
    int ret = 0;

    // Send the EC API command index AND product index TO SCAPE
    setVal(getStr(cmd, 1, iproduct));
    ret = EC_WaitForCompletion(16);
    getVal("GET ExtControlParamOut0;");
    *objHeightInBin = (int)tempData[0];
    return ret;
}

static int EC_GetPickZoneInfo(int iProduct, int iPickZoneIndex, int iSubZoneIndex, int *objHeightInZone, short *zoneIsOpen)
{
    char cmd[BUFF_LEN] = "SET_GROUP_VALUES GETPICKZONEINFOGROUP 26 %d %d %d;";
    int ret = 0;

    setVal(getStr(cmd, 3, iProduct, iPickZoneIndex, iSubZoneIndex));
    ret = EC_WaitForCompletion(26);
    getVal("GET ExtControlParamOut0 ExtControlParamOut1;");

    *objHeightInZone = (int)tempData[0];
    *zoneIsOpen = (short)tempData[1];
    return ret;
}

static int EC_GetPlacePose(int iProduct, double *po_PlacePose, int *placeFrame)
{
    char cmd[BUFF_LEN] = "SET ExtControlCommand 4 ExtControlParamIn0 %d;";
    int ret = 0;

    setVal(getStr(cmd, 1, iProduct));
    ret = EC_WaitForCompletion(4);
    getVal("GET_GROUP_VALUES GETPLACEGROUP;");
    po_PlacePose[0] = tempData[0] / 10.0;
    po_PlacePose[1] = tempData[1] / 10.0;
    po_PlacePose[2] = tempData[2] / 10.0;
    po_PlacePose[3] = tempData[3] / 100.0;
    po_PlacePose[4] = tempData[4] / 100.0;
    po_PlacePose[5] = tempData[5] / 100.0;
    *placeFrame = (int)tempData[6];

    return ret;
}

static int EC_GetReturnValue(int iProduct, int iCommandIndex)
{
    char cmd[BUFF_LEN] = "SET ExtControlCommand 11 ExtControlParamIn0 %d ExtControlParamIn1 %d;";

    setVal(getStr(cmd, 2, iProduct, iCommandIndex));

    return EC_WaitForCompletion(11);
}

static int EC_GetSensorType(int iProduct, short bReturnOCSensor, int *SensorType)
{
    char cmd[BUFF_LEN] = "SET ExtControlCommand 12 ExtControlParamIn0 %d ExtControlParamIn1 %d;";
    int ret = 0;

    setVal(getStr(cmd, 2, iProduct, bReturnOCSensor));

    ret = EC_WaitForCompletion(12);
    getVal("GET ExtControlParamOut0;");
    *SensorType = (int)tempData[0];
    return ret;
}

static int EC_GridScannerCalib(int iToolUnitID)
{
    char cmd[BUFF_LEN] = "SET ExtControlCommand 74 ExtControlParamIn0 %d;";

    setVal(getStr(cmd, 1, iToolUnitID));

    os_core(true);

    return EC_WaitForCompletion(74);
}

static int EC_GridScannerVerify(int iSelfTestStationID, int iToolUnitID)
{
    char cmd[BUFF_LEN] = "SET ExtControlCommand 35 ExtControlParamIn0 %d ExtControlParamIn1 %d;";

    setVal(getStr(cmd, 2, iSelfTestStationID, iToolUnitID));

    os_core(true);

    return EC_WaitForCompletion(35);
}

static int EC_HSWasCleared(int iProduct)
{
    char cmd[BUFF_LEN] = "SET ExtControlCommand 17 ExtControlParamIn0 %d;";

    setVal(getStr(cmd, 1, iProduct));

    return EC_WaitForCompletion(17);
}

static int EC_ImageAcqRequired(int iProduct, short *ImageAcqRequired)
{
    char cmd[BUFF_LEN] = "SET ExtControlCommand 6 ExtControlParamIn0 %d;";
    int ret = 0;

    setVal(getStr(cmd, 1, iProduct));

    ret = EC_WaitForCompletion(6);
    getVal("GET ExtControlParamOut0;");
    *ImageAcqRequired = (short)tempData[0];
    return ret;
}

static int EC_InitProductSetup(int iProductSetup)
{
    char cmd[BUFF_LEN] = "SET ExtControlCommand 22 ExtControlParamIn0 %d;";

    setVal(getStr(cmd, 1, iProductSetup));

    return EC_WaitForCompletion(22);
}

static int EC_IsReady()
{
    getVal("GET ExtControlCommand;");
    if (tempData[0] != 0)
        return 1;

    setVal("SET ExtControlCommand 20;");
    waitSeconds(0.5);
    getVal("GET ExtControlCommand;");
    if (tempData[0] == 0)
        return 0;
    else
        return 1;
}

static int EC_LogValues(int iValue1, int iValue2, int iValue3, int iValue4, int iValue5, int iValue6, int iValue7, int iValue8, int iValue9, int iValue10)
{
    char cmd1[BUFF_LEN] = "SET ExtControlParamIn0 %d ExtControlParamIn1 %d ExtControlParamIn2 %d ExtControlParamIn3 %d ExtControlParamIn4 %d ExtControlParamIn5 %d;";
    char cmd2[BUFF_LEN] = "SET ExtControlParamIn6 %d ExtControlParamIn7 %d ExtControlParamIn8 %d ExtControlParamIn9 %d  ExtControlCommand 23;";

    setVal(getStr(cmd1, 6, iValue1, iValue2, iValue3, iValue4, iValue5, iValue6));
    setVal(getStr(cmd2, 4, iValue7, iValue8, iValue9));
    return 0;
}

static int EC_PerformHeightInit(int iProduct)
{
    char cmd[BUFF_LEN] = "SET ExtControlCommand 5 ExtControlParamIn0 %d;";

    setVal(getStr(cmd, 1, iProduct));

    os_core(true);

    return EC_WaitForCompletion(5);
}

static int EC_PickObject(int iProduct, short bLeaveRobotAtDepart, short bWaitForImageAcqComplete, double *po_ViaLow, int *ViaLowFrame, int *GripFamilyID, short *CalibrationIsNeed, short *AcqIsNeed)
{
    char cmd[BUFF_LEN] = "SET_GROUP_VALUES PICKOBJSETGROUP 2 %d %d %d;";
    int ret = 0;
    if (_dbg_SCAPE_)
        printf(getStr(cmd, 3, iProduct, bLeaveRobotAtDepart, bWaitForImageAcqComplete));
    setVal(getStr(cmd, 3, iProduct, bLeaveRobotAtDepart, bWaitForImageAcqComplete));
    if (_dbg_SCAPE_)
        printf("\n pick object EC_PickObject is act\n");
    os_core(true);
    ret = EC_WaitForCompletion(2);
    getVal("GET_GROUP_VALUES PICKOBJGETGROUP;");
    po_ViaLow[0] = tempData[0] / 10.0;
    po_ViaLow[1] = tempData[1] / 10.0;
    po_ViaLow[2] = tempData[2] / 10.0;
    po_ViaLow[3] = tempData[3] / 100.0;
    po_ViaLow[4] = tempData[4] / 100.0;
    po_ViaLow[5] = tempData[5] / 100.0;
    *ViaLowFrame = (int)tempData[6];
    *GripFamilyID = (int)tempData[7];
    *CalibrationIsNeed = (int)tempData[8];
    *AcqIsNeed = (int)tempData[9];
    return ret;
}

static int EC_PlaceObject(int iProduct, short *ObjectIsPlacedOnHS)
{
    char cmd[BUFF_LEN] = "SET ExtControlCommand 18 ExtControlParamIn0 %d;";
    int ret = 0;

    setVal(getStr(cmd, 1, iProduct));

    os_core(true);

    ret = EC_WaitForCompletion(18);
    getVal("GET ExtControlParamOut0;");
    *ObjectIsPlacedOnHS = (short)tempData[0];
    return ret;
}

static int EC_PlaceOnHS(int iProduct, short bPlaceManually, short bCheckGripper, short *HSIsFull, int *PlaceZoneIndex)
{
    char cmd[BUFF_LEN] = "SET_GROUP_VALUES PLACEONHSSETGROUP 14 %d %d %d;";
    int ret = 0;

    setVal(getStr(cmd, 3, iProduct, bPlaceManually, bCheckGripper));

    os_core(true);

    ret = EC_WaitForCompletion(14);
    getVal("GET ExtControlParamOut0 ExtControlParamOut1;");
    *HSIsFull = (short)tempData[0];
    *PlaceZoneIndex = (int)tempData[1];
    return ret;
}

static int EC_PlanAndMoveCartesian(int iProduct, double *Destination)
{
    // ready soon...
    return 0;
}

static int EC_RecognizeAtHS(int iProduct, short bWaitAcqToComplete)
{
    char cmd[BUFF_LEN] = "SET ExtControlCommand 13 ExtControlParamIn0 %d ExtControlParamIn1 %d;";

    setVal(getStr(cmd, 2, iProduct, bWaitAcqToComplete));

    os_core(true);
    // Wait until SCAPE has carried out the EC API Command
    return EC_WaitForCompletion(13);
}

static int EC_RegripAtHS(int iProduct, short bLeaveRobotAtDepart, short bGripManually, int *pGripFamilyID)
{
    char cmd[BUFF_LEN] = "SET_GROUP_VALUES REGRIPGROUP 3 %d %d %d;";
    int ret = 0;
    setVal(getStr(cmd, 3, iProduct, bLeaveRobotAtDepart, bGripManually));
    // Run SCAPE_OS_CORE to let SCAPE control the robot while a part is being picked.
    os_core(true);
    // Wait until SCAPE has carried out the EC API Command
    ret = EC_WaitForCompletion(3);
    getVal("GET ExtControlParamOut0;");
    *pGripFamilyID = tempData[0];
    return ret;
}

static int EC_ResetValues()
{
    char cmd1[BUFF_LEN] = "SET ExtControlCommand 0 ExtControlParamIn0 0 ExtControlParamIn1 0 ExtControlParamIn2 0 ExtControlParamIn3 0 ExtControlParamIn4 0 ExtControlParamIn5 0 ExtControlParamIn6 0 ExtControlParamIn7 0;";
    char cmd2[BUFF_LEN] = "SET ExtControlParamIn8 0 ExtControlParamIn9 0 ExtControlParamOut0 0 ExtControlParamOut1 0 ExtControlParamOut2 0 ExtControlParamOut3 0  ExtControlParamOut4 0 ExtControlParamOut5 0 ExtControlParamOut6 0;";
    char cmd3[BUFF_LEN] = "SET ExtControlParamOut7 0 ExtControlParamOut8 0 ExtControlParamOut9 0 ExtControlCommandResult 0;";

    setVal(cmd1);
    setVal(cmd2);
    setVal(cmd3);
    return 0;
}

static int EC_SetObjectHeightInBin(int iProduct, int iObjHeightInMM)
{
    char cmd[BUFF_LEN] = "SET ExtControlCommand 15 ExtControlParamIn0 %d ExtControlParamIn1 %d;";
    // Send the EC API command index and product index to SCAPE
    setVal(getStr(cmd, 2, iProduct, iObjHeightInMM));

    return EC_WaitForCompletion(15);
}

static int EC_SetPickZoneInfo(int iProduct, int iPickZoneIndex, int iSubZoneIndex, int iPropertyID, int iPropertyValue)
{
    char cmd[BUFF_LEN] = "SET_GROUP_VALUES SETPICKZONEINFOGROUP 27 %d %d %d %d %d;";
    // Send the EC API command index, pick zone index, sub zone index, property id and property value to SCAPE
    setVal(getStr(cmd, 5, iProduct, iPickZoneIndex, iSubZoneIndex, iPropertyID, iPropertyValue));
    return EC_WaitForCompletion(27);
}

static int EC_SetRobotStartPose(int iProduct, double *StartPose)
{
    char cmd[BUFF_LEN] = "SET_GROUP_VALUES STARTPOSGROUP 51 %d %l %l %l %l %l %l ";
    char sVar[SVAR_LEN] = {"\0"};

    getStr(cmd, 7, iProduct, dtl(*(StartPose + 0) * 100), dtl(*(StartPose + 1) * 100), dtl(*(StartPose + 2) * 100), dtl(*(StartPose + 3) * 100), dtl(*(StartPose + 4) * 100), dtl(*(StartPose + 5) * 100));
    if (SCAPE_ROBOT_AXIS_NUM == 6)
        strcat(cmd, "0 0 ;");
    if (SCAPE_ROBOT_AXIS_NUM == 7)
    {
        strcat(cmd, ltos(dtl(*(StartPose + 6) * 100), sVar));
        strcat(cmd, " 0;");
    }
    if (SCAPE_ROBOT_AXIS_NUM == 8)
    {
        strcat(cmd, ltos(dtl(*(StartPose + 6) * 100), sVar));
        strcat(cmd, " ");
        strcat(cmd, ltos(dtl(*(StartPose + 7) * 100), sVar));
        strcat(cmd, " ;");
    }
    setVal(cmd);
    return EC_WaitForCompletion(51);
}

static int EC_SliderScannerVerify(int iScannerID, int iCalibrationRangeID)
{
    char cmd[BUFF_LEN] = "SET ExtControlCommand 41 ExtControlParamIn0 %d ExtControlParamIn1 %d;";
    // Send the EC API command index and SCANNER_ID to SCAPE
    setVal(getStr(cmd, 2, iScannerID, iCalibrationRangeID));
    // Run SCAPE_OS_CORE to let SCAPE control the robot while the images are being acquired
    os_core(true);
    // Wait until SCAPE has carried out the EC API Command

    return EC_WaitForCompletion(41);
}

static int EC_StartOfCycle(short bResetStatistics)
{
    char cmd[BUFF_LEN] = "SET ExtControlCommand 1 ExtControlParamIn0 %d;";
    setVal(getStr(cmd, 1, bResetStatistics));
    return EC_WaitForCompletion(1);
}

static int EC_StationaryCameraCalib(int iCameraID)
{
    char cmd[BUFF_LEN] = "SET ExtControlCommand 70 ExtControlParamIn0 %d;";
    // Send the EC API command index and camera ID to SCAPE
    setVal(getStr(cmd, 1, iCameraID));
    // Run SCAPE_OS_CORE to let SCAPE control the robot while the images are being acquired
    os_core(true);
    // Wait until SCAPE has carried out the EC API Command

    return EC_WaitForCompletion(70);
}

static int EC_StationaryCameraVerify(int iCameraID)
{
    char cmd[BUFF_LEN] = "SET ExtControlCommand 30 ExtControlParamIn0 %d;";
    // Send the EC API command index and CAMERA_ID to SCAPE
    setVal(getStr(cmd, 1, iCameraID));

    // Run SCAPE_OS_CORE to let SCAPE control the robot while the images are being acquired
    os_core(true);
    // Wait until SCAPE has carried out the EC API Command
    return EC_WaitForCompletion(30);
}

static int EC_SuctionOffsetVerify(int iSelfTestStationID, int iToolUnitID, int iGripperID)
{
    char cmd[BUFF_LEN] = "SET_GROUP_VALUES SUCTIONOFFSETGROUP 36 %d %d %d;";

    // Send the EC API command index and SELF_TEST_STATION_ID to SCAPE
    setVal(getStr(cmd, 3, iSelfTestStationID, iToolUnitID, iGripperID));

    // Run SCAPE_OS_CORE to let SCAPE control the robot while the images are being acquired
    os_core(true);
    // Wait until SCAPE has carried out the EC API Command
    return EC_WaitForCompletion(36);
}

static int EC_SuctionStrengthTest(int iSelfTestStationID, int iToolUnitID, int iGripperID)
{
    char cmd[BUFF_LEN] = "SET_GROUP_VALUES SUCTIONSTRENGTHGROUP 37 %d %d %d;";

    // Send the EC API command index and SELF_TEST_STATION_ID to SCAPE
    setVal(getStr(cmd, 3, iSelfTestStationID, iToolUnitID, iGripperID));

    // Run SCAPE_OS_CORE to let SCAPE control the robot while the images are being acquired
    os_core(true);
    // Wait until SCAPE has carried out the EC API Command
    return EC_WaitForCompletion(37);
}

static int EC_GridScannerCameraVerify(int iSelfTestStationID, int iToolUnitID)
{
    char cmd[BUFF_LEN] = "SET_GROUP_VALUES GRIDCAMERAVERIFYGROUP 31 %d %d;";

    // Send the EC API command index and SELF_TEST_STATION_ID to SCAPE
    setVal(getStr(cmd, 2, iSelfTestStationID, iToolUnitID));

    // Run SCAPE_OS_CORE to let SCAPE control the robot while the images are being acquired
    os_core(true);
    // Wait until SCAPE has carried out the EC API Command
    return EC_WaitForCompletion(31);
}

static int EC_VacumSensorTest(int iSelfTestStationID, int iToolUnitID, int iGripperID)
{
    char cmd[BUFF_LEN] = "SET_GROUP_VALUES TESTVACUUMGROUP 38 %d %d %d;";

    // Send the EC API command index and SELF_TEST_STATION_ID to SCAPE
    setVal(getStr(cmd, 3, iSelfTestStationID, iToolUnitID, iGripperID));

    // Run SCAPE_OS_CORE to let SCAPE control the robot while the images are being acquired
    os_core(true);
    // Wait until SCAPE has carried out the EC API Command
    return EC_WaitForCompletion(38);
}

static int EC_WaitForCompletion(int iCommandID)
{
    // wait until command completed
    int nCommandVal = 0, nReturnVal = 0, nSCAPEStatus = 0;

    // Wait until SCAPE has carried out the EC API Command
    nCommandVal = iCommandID;
    while (nCommandVal == iCommandID && nSCAPEStatus == 0)
    {
        getVal("GET ExtControlCommand ExtControlCommandResult SCAPE_STATUS;");
        nCommandVal = tempData[0];
        nReturnVal = tempData[1];
        nSCAPEStatus = tempData[2];
    }
    if (nCommandVal == 0 && nReturnVal != 0)
    {
        return nReturnVal;
    }
    else
    {
        // Return only 0 if SCAPE_STATUS is 0
        return -1 * nSCAPEStatus;
    }
}

//*******************************************
//             scape os core
//*******************************************
static void is_IO_or_task_ready(short *IO_ready, short *task_ready, short *task_type, short *stay_in_os_core);
static void update_io(void);
static Scape_Task_Internal get_task(short taskType, short *taskLeft, short read_task_counter);
static void run_job(int job_id, int p0, int p1, int p2);
static void teach_scape_pose(void);
static void send_scape_tcp_pose(Scape_Task_Internal task);
static void send_scape_joint_value(Scape_Task_Internal task);
static Scape_Task_Internal tasks[MAX_SCAPE_TASKS_LEN];

static void wait_all_tasks_done()
{
    TaskRunStatus taskRunStatus = Started;
    scapeRobot->fnWaitScapeTaskComplete(&taskRunStatus);
}

static void run_task(Scape_Task_Internal *task)
{
    if (!task)
    {
        stopAndShowErr("run NULL task!!");
    }

    if (task->task_was_executed == true)
    {
        return;
    }

    scapeRobot->fnRunScapeTask(&(task->user_task));

    task->task_was_executed = true;

    if (task->user_task.job_id == 101) // job 101, teach pose for scape
    {
        send_scape_tcp_pose(*task);
        teach_scape_pose();
    }
    if (task->user_task.job_id == 111) // job 111, get tcp pose for scape
    {
        send_scape_tcp_pose(*task);
    }
    if (task->user_task.job_id == 112) // job 112, get joints value for scape
    {
        send_scape_joint_value(*task);
    }
}

static void os_core(short run_in_ec_mode)
{
    short io_ready = false, task_ready = false, stay_in_os_core = false;
    short task_type = 0, task_left = -1, task_counter = 0, pre_execution_possible = true, i = 0;
    TaskRunStatus taskRunStatus = Started;

    while (1)
    {
        is_IO_or_task_ready(&io_ready, &task_ready, &task_type, &stay_in_os_core);

        if (stay_in_os_core == false && run_in_ec_mode)
            return;

        if (io_ready)
            update_io();

        if (task_ready)
        {
            tasks[task_counter] = get_task(task_type, &task_left, task_counter);
            task_counter++;
            if (task_counter > MAX_SCAPE_TASKS_LEN - 1)
                stopAndShowErr("scape tasks too much!");
        }

        if (task_left == 0 && task_counter > 0)
        {
            taskRunStatus = Started;
            /* if (scapeRobot->fnGetExeMode()){
                scapeRobot->fnMPRunScapeTask(tasks,task_counter);
            }
            else */{
                for (i = 0; i < task_counter; i++) run_task(&tasks[i]);
                scapeRobot->fnWaitScapeTaskComplete(&taskRunStatus);
                if (taskRunStatus == SliderColision) setVal("SET ROBOT_COLLISION_STATE 1;");
                if (taskRunStatus == FlangeCollision) setVal("SET ROBOT_COLLISION_STATE 2;");
            }
            setVal("SET ROBOT_TASKS_LEFT 0;");
            task_counter = 0;
            task_left = -1;
            pre_execution_possible = true;
        }
    }
}

static void is_IO_or_task_ready(short *IO_ready, short *task_ready, short *task_type, short *stay_in_os_core)
{
    int scape_status;

    getVal("GET_GROUP_VALUES ISIOREADYGROUP;");

    *IO_ready = tempData[0];
    *task_ready = tempData[1];
    *task_type = tempData[2];
    *stay_in_os_core = tempData[3];
    scape_status = tempData[4];

    if (scape_status != 0)
        *stay_in_os_core = false;
}

static Scape_Task_Internal get_task(short taskType, short *taskLeft, short read_task_counter)
{
    Scape_Task_Internal task;
    char cmd[BUFF_LEN] = "SET ROBOT_TASKS_LEFT %d;";
    int i;

    task.task_was_executed = false;
    task.user_task.taskIndex = read_task_counter + 1;
    switch (taskType)
    {
    case 0:
        task.user_task.motionValid = true;
        getVal("GET_GROUP_VALUES MTGROUP;");
        task.user_task.target[0] = tempData[0] / 10.0;
        task.user_task.target[1] = tempData[1] / 10.0;
        task.user_task.target[2] = tempData[2] / 10.0;
        task.user_task.target[3] = tempData[3] / 100.0;
        task.user_task.target[4] = tempData[4] / 100.0;
        task.user_task.target[5] = tempData[5] / 100.0;
        task.user_task.speed = (int)tempData[6] / 10;
        task.user_task.acc = (int)tempData[7] / 10;
        if (tempData[8] /* user liner motion*/)
        {
            task.user_task.motionType = MOVEL;
        }
        else
        {
            task.user_task.motionType = MOVEJ;
        }
        task.user_task.job_id = (int)tempData[9];
        task.user_task.par0 = 0;
        task.user_task.par1 = 0;
        task.user_task.par2 = 0;
        if (task.user_task.job_id > 0)
        {
            task.user_task.logicValid = true;
        }
        else
        {
            task.user_task.logicValid = false;
        }

        task.user_task.blend = (int)tempData[10] / 10;
        task.triggerCamera = (short)tempData[11];

        if (tempData[12] /* stop completely*/)
        {
            task.user_task.blend = -1;
        }

        task.frameID = (short)tempData[13];
        *taskLeft = (short)tempData[14];
        task.user_task.taskNumInTotal = read_task_counter + 1 + *taskLeft;

        task.user_task.stateWaitMotionStart = (short)tempData[16];
        if (tempData[15] /* use global config*/)
        {
            for (i = 0; i < 6; i++)
                task.user_task.joints[i] = scapeRobot->jGlobalBestConfig[i];
        }
        if (tempData[17] /* use joints value*/ && tempData[15] == 0 /* use global best config*/)
        {
            task.user_task.joints[0] = tempData[18] / 100.0;
            task.user_task.joints[1] = tempData[19] / 100.0;
            task.user_task.joints[2] = tempData[20] / 100.0;
            task.user_task.joints[3] = tempData[21] / 100.0;
            task.user_task.joints[4] = tempData[22] / 100.0;
            task.user_task.joints[5] = tempData[23] / 100.0;
        }
        task.targetType = tempData[26];
        break;
    case 1:
        task.user_task.motionValid = false;
        task.user_task.logicValid = true;
        getVal("GET_GROUP_VALUES ETGROUP;");
        if (tempData[0] != 4 /* prog ID update state*/)
        {
            task.user_task.job_id = tempData[0] + 100;
        }
        else
        {
            task.user_task.job_id = tempData[0];
        }
        task.user_task.par0 = (int)tempData[1];
        task.user_task.par1 = (int)tempData[2];
        task.user_task.par2 = (int)tempData[3];
        if (task.user_task.par1 /* state wait motion start*/)
        {
            task.user_task.stateWaitMotionStart = true;
        }
        else
        {
            task.user_task.stateWaitMotionStart = false;
        }

        *taskLeft = (short)tempData[4];
        task.user_task.taskNumInTotal = read_task_counter + 1 + *taskLeft;
        break;
    case 2:
        task.user_task.motionValid = false;
        task.user_task.logicValid = true;
        getVal("GET_GROUP_VALUES STGROUP;");
        task.user_task.job_id = 201;   // JOB_SLEEP = 201
        task.user_task.par0 = tempData[0]; // sleep time
        *taskLeft = (short)tempData[1];
        task.user_task.taskNumInTotal = read_task_counter + 1 + *taskLeft;
        break;
    default:
        stopAndShowErr("error task type!!");
        break;
    }

    if (*taskLeft == 0)
    {
        setVal(getStr(cmd, 1, read_task_counter + 1));
    }
    setVal("SET ROBOT_TASK_READY 0;");

    return task;
}

void run_job(int job_id, int p0, int p1, int p2)
{
    int i;

    tasks[0].task_was_executed = false;
    tasks[0].user_task.taskIndex = 1;
    tasks[0].user_task.taskNumInTotal = 1;
    tasks[0].user_task.motionValid = false;
    tasks[0].user_task.motionType = NotMov;
    for (i = 0; i < 6; i++)
        tasks[0].user_task.target[i] = 0;
    for (i = 0; i < SCAPE_ROBOT_AXIS_NUM; i++)
        tasks[0].user_task.joints[i] = 0;
    tasks[0].user_task.speed = 0;
    tasks[0].user_task.acc = 0;
    tasks[0].user_task.blend = 0;
    tasks[0].user_task.logicValid = true;
    tasks[0].user_task.job_id = job_id;
    tasks[0].user_task.par0 = p0;
    tasks[0].user_task.par1 = p1;
    tasks[0].user_task.par2 = p2;
    run_task(&(tasks[0]));
    wait_all_tasks_done();
}

static void teach_scape_pose(void)
{
    setVal("SET ROBOT_PARAMETER_0 0 ROBOT_PARAMETER_1 1;");
    //  Poll the ROBOT_PARAMETER_1 variable, until it is changed to 0.
    getVal("GET ROBOT_PARAMETER_1 ROBOT_PARAMETER_2;");
    while (tempData[0] != 0 && tempData[0] != 1)
    {
        getVal("GET ROBOT_PARAMETER_1 ROBOT_PARAMETER_2;");
    }
}

static void send_scape_tcp_pose(Scape_Task_Internal task)
{
    char cmd[BUFF_LEN + 1] = "SET_GROUP_VALUES TCPPOSEGROUP %l %l %l %l %l %l;";

    if (_dbg_SCAPE_ == 2 || _dbg_SCAPE_ == 9)
        printf("\n scp get_tcp_pose %lf %lf %lf %lf %lf %lf\n", task.user_task.target[0], task.user_task.target[1], task.user_task.target[2],
               task.user_task.target[3], task.user_task.target[4], task.user_task.target[5]);

    setVal(getStr(cmd, 6, dtl(task.user_task.target[0] * 10), dtl(task.user_task.target[1] * 10), dtl(task.user_task.target[2] * 10),
                  dtl(task.user_task.target[3] * 100), dtl(task.user_task.target[4] * 100), dtl(task.user_task.target[5] * 100)));
}

static void send_scape_joint_value(Scape_Task_Internal task)
{
    char cmd[BUFF_LEN + 1] = "SET ROBOT_JOINT_1_VALUE %l ROBOT_JOINT_2_VALUE %l ROBOT_JOINT_3_VALUE %l ROBOT_JOINT_4_VALUE %l ROBOT_JOINT_5_VALUE %l ROBOT_JOINT_6_VALUE %l;";

    if (_dbg_SCAPE_ == 2 || _dbg_SCAPE_ == 9)
        printf("\n scp get_joint_value %lf %lf %lf %lf %lf %lf\n", task.user_task.joints[0], task.user_task.joints[1], task.user_task.joints[2],
               task.user_task.joints[3], task.user_task.joints[4], task.user_task.joints[5]);

    setVal(getStr(cmd, 6, dtl(task.user_task.joints[0] * 100), dtl(task.user_task.joints[1] * 100), dtl(task.user_task.joints[2] * 100),
                  dtl(task.user_task.joints[3] * 100), dtl(task.user_task.joints[4] * 100), dtl(task.user_task.joints[5] * 100)));
}

static void update_io(void)
{
    char cmd[BUFF_LEN] = "SET GripSensor1_VALUE %d GripSensor2_VALUE %d GripSensor3_VALUE %d;";

    getVal("GET RingLight_LIGHT_RING_ON PatternProjector_LIGHT_PATTERN_PROJECTOR_ON;");
    run_job(JOB_LIGHT_ON, tempData[0], 0, 0);
    run_job(JOB_PATTERN_ON, tempData[1], 0, 0);
    run_job(JOB_CHECK_GRIP, 0, 0, 0);
    setVal(getStr(cmd, 3, tasks[0].user_task.par0, tasks[0].user_task.par1, tasks[0].user_task.par2));

    setVal("SET ROBOT_SIGNAL_READY 0;");
}

//*******************************************
//       scape robot path control
//*******************************************
typedef enum
{
    atUnknown,
    atBinEntry,
    atHSEntry,
    atBin,
    atClearBin,
    atHS,
    atClearHS,
    atBinExit,
    atHSExit
} RobotPose;
static RobotPose currentRobotPose = atUnknown;
static int products_num_of_current_group = 0;
static int current_selected_product_id = 0;
static int current_group_id = 1;

static void m_clear_of_hs(int group_id, int product_id)
{
    run_job(JOB_MOV_CLEAR_HS, group_id, product_id, 0);
}

static void m_bin_to_hs(int group_id, int product_id)
{
    run_job(JOB_MOVE_BIN_TO_HS, group_id, product_id, 0);
}

static void m_hs_to_bin(int group_id, int product_id)
{
    run_job(JOB_MOVE_HS_TO_BIN, group_id, product_id, 0);
}

static void request_robot_pose(RobotPose pose)
{
    return ;
    switch (currentRobotPose)
    {
    case atUnknown:
        if (pose == atHSEntry || pose == atBinEntry)
        {
            currentRobotPose = pose;
            return;
        }
        stopAndShowErr(getStr("path from %d to %d not exist!", 2, currentRobotPose, pose));
    case atBinEntry:
        if (pose == atBinEntry || pose == atClearBin || pose == atBin || pose == atBinExit)
        {
            currentRobotPose = pose;
            return;
        }
        if (pose == atHSEntry || pose == atHS)
        {
            m_bin_to_hs(current_group_id, current_selected_product_id);
            currentRobotPose = pose;
            return;
        }
        stopAndShowErr(getStr("path from %d to %d not exist!", 2, currentRobotPose, pose));
    case atHSEntry:
        if (pose == atHSEntry || pose == atHS || pose == atHSExit)
        {
            currentRobotPose = pose;
            return;
        }
        if (pose == atClearHS)
        {
            m_clear_of_hs(current_group_id, current_selected_product_id);
            currentRobotPose = pose;
            return;
        }
        if (pose == atBin || pose == atBinEntry)
        {
            m_hs_to_bin(current_group_id, current_selected_product_id);
            currentRobotPose = pose;
            return;
        }
        stopAndShowErr(getStr("path from %d to %d not exist!", 2, currentRobotPose, pose));
    case atBin:
        if (pose == atHS || pose == atHSEntry)
        {
            m_bin_to_hs(current_group_id, current_selected_product_id);
            currentRobotPose = pose;
            return;
        }
        if (pose == atClearBin || pose == atBinExit || pose == atBinEntry || pose == atBin)
        {
            currentRobotPose = pose;
            return;
        }
        stopAndShowErr(getStr("path from %d to %d not exist!", 2, currentRobotPose, pose));
    case atClearBin:
        if (pose == atHS || pose == atHSEntry)
        {
            m_bin_to_hs(current_group_id, current_selected_product_id);
            currentRobotPose = pose;
            return;
        }
        if (pose == atClearBin || pose == atBinExit || pose == atBinEntry || pose == atBin)
        {
            currentRobotPose = pose;
            return;
        }
        stopAndShowErr(getStr("path from %d to %d not exist!", 2, currentRobotPose, pose));
    case atHS:
        if (pose == atBin || pose == atBinEntry)
        {
            m_hs_to_bin(current_group_id, current_selected_product_id);
            currentRobotPose = pose;
            return;
        }
        if (pose == atClearHS)
        {
            m_clear_of_hs(current_group_id, current_selected_product_id);
            currentRobotPose = pose;
            return;
        }
        if (pose == atHSEntry || pose == atHSExit || pose == atHS)
        {
            currentRobotPose = pose;
            return;
        }
        stopAndShowErr(getStr("path from %d to %d not exist!", 2, currentRobotPose, pose));
    case atClearHS:
        if (pose == atHS || pose == atHSEntry || pose == atHSExit || pose == atClearHS)
        {
            currentRobotPose = pose;
            return;
        }
        if (pose == atBin || pose == atBinEntry)
        {
            // move from clear hs back to hs ????
            m_hs_to_bin(current_group_id, current_selected_product_id);
            currentRobotPose = pose;
            return;
        }
        stopAndShowErr(getStr("path from %d to %d not exist!", 2, currentRobotPose, pose));
    case atBinExit:
        if (pose == atBinEntry || pose == atBinExit || pose == atHSEntry)
        {
            currentRobotPose = pose;
            return;
        }
        stopAndShowErr(getStr("path from %d to %d not exist!", 2, currentRobotPose, pose));
    case atHSExit:
        if (pose == atHSExit || pose == atHSEntry || pose == atBinEntry)
        {
            currentRobotPose = pose;
            return;
        }
        stopAndShowErr(getStr("path from %d to %d not exist!", 2, currentRobotPose, pose));
    default:
        stopAndShowErr(getStr("path from %d to %d not exist!", 2, currentRobotPose, pose));
    }
}

//*******************************************
//             scape bin picking
//*******************************************
static int get_sensor_type(Scp_Product *, short);
static void oc_one_zone(Scp_Product *, short *);
static void oc_two_zones_stat_cam(Scp_Product *, short *);
static void oc_two_zones_tool_cam(Scp_Product *);
static short place_part_on_hs(Scp_Product *, short manully_put);
static void move_clear_and_empty_hs(Scp_Product *);
static void start_oc_recognition(Scp_Product *, short);
static void perform_regrip_at_hs(Scp_Product *);
static void check_oc_recognition_result(Scp_Product *);

static void reset_product_data(Scp_Product *product)
{
    product->product_group_id = 0;
    product->product_id = 0;
    product->parts_bin_picked = 0;
    product->cycle_index = 0;
    product->parts_on_hs = 0;
    product->product_init_done = false;
    product->bin_is_empty = false;
    product->hs_is_full = false;
    product->oc_recog_success = false;
    product->oc_recog_started = false;
    product->oc_part_was_picked = false;
    product->height_is_initialized = false;
    product->bp_acq_needed = true;
    product->bp_calib_needed = false;
    product->use_oc = false;
    product->use_bp = false;
    product->bp_use_tool_sensor = false;
    product->layer_done = false;
    product->oc_use_tool_cam = false;
    product->part_placed_on_hs = false;
}

static void perform_height_init(Scp_Product *product, unsigned int start_height)
{
    int result = 0;

    if (product->bp_use_tool_sensor)
        request_robot_pose(atBin);
    if (product->use_bp == false)
        return;
    product->bp_acq_needed = true;
    if (start_height <= 0)
    {
        // perform height initialization
        result = EC_PerformHeightInit(product->product_id);
        switch (result)
        {
        case -2: // SCAPE Bin-Picking Manager was stopped
            stopAndShowCodeMsg(902);
            break;
        case -1: // SCAPE Bin-Picking Manager was restarted
            stopAndShowCodeMsg(901);
            break;
        case 0: // all ok
            product->bin_is_empty = false;
            product->height_is_initialized = true;
            break;
        case 1: // Unknown product
            stopAndShowCodeMsg(5001);
            break;
        case 2: // Could not detect height
            stopAndShowCodeMsg(5002);
            break;
        case 10: // SCAPE ERROR
            stopAndShowCodeMsg(5010);
            break;
        case 20: // Failed connecting to camera
            stopAndShowCodeMsg(5020);
            break;
        case 21: // Camera images are too dark
            stopAndShowCodeMsg(5021);
            break;
        case 22: // Sliding Scanner Error
            stopAndShowCodeMsg(5022);
        case 23: // No 3D points were found
            stopAndShowCodeMsg(5023);
            break;
        default:
            // Unsupported return code
            stopAndShowCodeMsg(5999);
            break;
        }
    }
    else
    {
        // FIXED_STARTING_HEIGHT was specified so set height to this value
        result = EC_SetObjectHeightInBin(product->product_id, start_height);
        switch (result)
        {
        case -2: // SCAPE Bin-Picking Manager was stopped
            stopAndShowCodeMsg(902);
            break;
        case -1: // SCAPE Bin-Picking Manager was restarted
            stopAndShowCodeMsg(901);
            break;
        case 0: // all ok
            product->bin_is_empty = false;
            product->height_is_initialized = true;
            break;
        case 1: // Unknown product
            stopAndShowCodeMsg(15001);
            break;
        default:
            // Unsupported return code
            stopAndShowCodeMsg(15999);
            break;
        }
    }
}

static void height_init_if_needed(Scp_Product *product, unsigned int start_height)
{
    if (product->use_bp == false)
        return;
    if (product->height_is_initialized == false)
    {
        if (product->bp_use_tool_sensor == false)
        {
            // supporse to move robot clear of bin
            // do nothing here, user to make sure the last teach point
            // before the scp_pick() call is clear of the bin.
        }
        perform_height_init(product, start_height);
    }
}

static void init_product_group(int group_id)
{
    int result = 0;
    if (_dbg_SCAPE_)
        printf("\nscp: init_product_group: %d\n", group_id);
        result = EC_InitProductSetup(group_id);
        switch (result)
        {
        case -2: // SCAPE Bin-Picking Manager was stopped
            stopAndShowCodeMsg(902);
            break;
        case -1: // SCAPE Bin-Picking Manager has been re-started
            stopAndShowCodeMsg(901);
            break;
        case 0: // all ok
            break;
        case 1: // Unknown product or loading failed
            stopAndShowCodeMsg(22001);
            break;
        case 20: // Failed connecting to camera
            stopAndShowCodeMsg(22020);
            break;
        default:
            // Unsupported return code
            stopAndShowCodeMsg(22999);
            break;
        }
}

static void set_up_sensor_type(Scp_Product *product)
{
    int bp_sensor_type = 0, oc_sensor_type = 0;

    // Get sensor type for bin picking
    bp_sensor_type = get_sensor_type(product, false);

    product->use_bp = true;
    if (bp_sensor_type == -1)
        product->use_bp = false;

    if (bp_sensor_type == 1)
    {
        product->bp_use_tool_sensor = false;
    }
    else
    {
        product->bp_use_tool_sensor = true;
    }

    // Get sensor type for OC
    product->use_oc = true;
    oc_sensor_type = get_sensor_type(product, true);

    if (oc_sensor_type == -1)
    {
        product->use_oc = false;
        // Make sure that all variables have been initialized
        product->total_zones_on_hs = 1;
        product->oc_use_tool_cam = true;
    }
    else
    {
        if (oc_sensor_type == 1 || oc_sensor_type == 3)
        {
            product->oc_use_tool_cam = false;
        }
        else
        {
            product->oc_use_tool_cam = true;
        }
        if (oc_sensor_type == 1 || oc_sensor_type == 2)
        {
            product->total_zones_on_hs = 1;
        }
        else
        {
            product->total_zones_on_hs = 2;
        }
    }
}

static int get_sensor_type(Scp_Product *product, short return_oc_sensor_type)
{
    int result = 0, sensor_type = 0;

    result = EC_GetSensorType(product->product_id, return_oc_sensor_type, &sensor_type);
    switch (result)
    {
    case -2: // SCAPE Bin-Picking Manager was stopped
        stopAndShowCodeMsg(902);
        break;
    case -1: // SCAPE Bin-Picking Manager was restarted
        stopAndShowCodeMsg(901);
        break;
    case 0: // all ok
        break;
    case 1: // Unknown product
        stopAndShowCodeMsg(12001);
        break;
    case 2: // Product does not use OC
            // stopAndShowCodeMsg(12002)
        sensor_type = -1;
        break;
    case 3:
        // product not using bin-picking
        sensor_type = -1;
        break;
    case 10: // SCAPE Error
        stopAndShowCodeMsg(12010);
        break;
    default:
        // Unsupported return code
        stopAndShowCodeMsg(12999);
        break;
    }
    return sensor_type;
}

static void perform_oc(Scp_Product *product, short *back_to_bp)
{
    request_robot_pose(atHSEntry);

    if (product->total_zones_on_hs == 1)
    {
        oc_one_zone(product, back_to_bp);
    }
    else
    {
        if (product->oc_use_tool_cam)
        {
            oc_two_zones_tool_cam(product);
        }
        else
        {
            oc_two_zones_stat_cam(product, back_to_bp);
        }
    }
}

static void oc_one_zone(Scp_Product *product, short *back_to_bp)
{
    short part_was_placed_on_hs = false;
    // IF BIN_IS_EMPTY == FALSE then robot must have brought
    // a part. Place it!
    // If no part was brought to the HS, then there is also
    // nothing to recognize or grip.

    request_robot_pose(atHS);
    if (product->bin_is_empty == false)
    {
        part_was_placed_on_hs = place_part_on_hs(product, false);
        if (part_was_placed_on_hs)
        {
            if (product->oc_use_tool_cam == false)
            {
                request_robot_pose(atClearHS);
            }

            start_oc_recognition(product, true);
            check_oc_recognition_result(product);

            if (product->oc_recog_success)
            {
                perform_regrip_at_hs(product);
            }
        }
        if (product->oc_part_was_picked == false)
        {
            move_clear_and_empty_hs(product);

            if (product->use_bp)
                *back_to_bp = true;
        }
        else
        {
            *back_to_bp = false;
        }
    }
}

static void oc_two_zones_stat_cam(Scp_Product *product, short *back_to_bp)
{
    short oc_place_or_grip_failed = false;

    request_robot_pose(atHS);
    product->oc_part_was_picked = false;
    product->oc_recog_success = false;
    *back_to_bp = false;
    // If was placed but not recognized, remove it.
    if (product->parts_on_hs > 0)
    {
        check_oc_recognition_result(product);
        if (product->oc_recog_success == false)
        {
            move_clear_and_empty_hs(product);
        }
    }

    // IF BIN_IS_EMPTY == FALSE then robot must have brought
    // a part to the handling station. Place this part on the
    // handling station.
    if (product->bin_is_empty == false)
    {
        if (place_part_on_hs(product, false) == false)
        {
            oc_place_or_grip_failed = true;
        }
    }

    if (product->parts_on_hs < 2 && !product->bin_is_empty){
        *back_to_bp = true;
    }
    
    // PRODUCT_DATA.OC_RECOGNITION_SUCCESS will only be true
    // if there is a part on the handling station and it
    // has been recognized.
    if (product->oc_recog_success)
    {
        perform_regrip_at_hs(product);
        if (product->oc_part_was_picked == false)
        {
            oc_place_or_grip_failed = true;
        }
    }

    if (oc_place_or_grip_failed)
    {
        move_clear_and_empty_hs(product);

        if (product->use_bp)
            *back_to_bp = true;
    }

    // This way of starting recogntion requires that we
    // always have "motion detection" enabled.
    if (product->parts_on_hs > 0)
    {
        start_oc_recognition(product, false);
    }
}

static void move_clear_and_empty_hs(Scp_Product *product)
{
    request_robot_pose(atClearHS);
    run_job(JOB_EMPTY_HS, product->product_group_id, product->product_id, 0);
    if (product->use_oc)
    {
        product->oc_recog_success = false;
        product->oc_recog_started = false;
        product->parts_on_hs = 0;
        product->oc_part_was_picked = false;
        product->part_placed_on_hs = false;
    }
}

static void oc_two_zones_tool_cam(Scp_Product *product)
{
    stopAndShowErr("Not supported tool camera with two zones on HS!");
}

static void acquire_bin_data(Scp_Product *product, short force_scan)
{
    int result = 0;
    // move clear of bin user make sure when this called robot is out of scanner view

    if ((!product->use_bp || (product->bin_is_empty)))
        return;
    if (product->bp_acq_needed || force_scan)
    {
        product->bp_acq_needed = false;

        result = EC_AcquireImages(product->product_id, force_scan);
        switch (result)
        {
        case -2: // SCAPE Bin-Picking Manager was stopped
            stopAndShowCodeMsg(902);
            break;
        case -1: // SCAPE Bin-Picking Manager was restarted
            stopAndShowCodeMsg(901);
            break;
        case 0: // Success
            break;
        case 1: // Unknown acq product
            stopAndShowCodeMsg(7001);
            break;
        case 10: // SCAPE Error
            stopAndShowCodeMsg(7010);
            break;
        default: //
            // Unsupported return code
            stopAndShowCodeMsg(7999);
            break;
        }
    }
}

static void get_object_height_in_bin(Scp_Product *product, unsigned int* part_level_mm){
    int result = 0;

    result = EC_GetObjHeightInBin(product->product_id,part_level_mm);
    switch (result)
    {
    case -2: // SCAPE Bin-Picking Manager has been stopped
        stopAndShowCodeMsg(902);
        break;
    case -1: // SCAPE Bin-Picking Manager has been re-started
        stopAndShowCodeMsg(901);
        break;
    case 0:    // all ok
        break; //
    case 1: // Unknown product
        stopAndShowCodeMsg(16001);
        break;
    case 2: // Could not detect height
        stopAndShowCodeMsg(16002);
        break;
    case 9: // Loaded product has no bin Sessions.
        stopAndShowCodeMsg(16009);
        break;
    case 10: // SCAPE Error
        stopAndShowCodeMsg(16010);
        break;
    default:
        break;
    }
}

static void pick_object(Scp_Product *product, Bin *bin)
{
    short leave_robot_at_depart = false;
    short wait_for_img_acq_to_comp = true;
    int result = 0, viaLowFrame;
    double viaLow[6];

    request_robot_pose(atBin);
    bin->bin_status = 0;
    if (product->use_bp == false)
        return;
    result = EC_PickObject(product->product_id, leave_robot_at_depart, wait_for_img_acq_to_comp, viaLow, &viaLowFrame, &product->grip_family_id, &product->bp_calib_needed, &product->bp_acq_needed);
    switch (result)
    {
    case -2: // SCAPE Bin-Picking Manager was stopped
        stopAndShowCodeMsg(902);
        break;
    case -1: // SCAPE Bin-Picking Manager was restarted
        stopAndShowCodeMsg(901);
        break;
    case 0: // all ok
        bin->bin_status = product->grip_family_id;
        bin->picked_parts_count++;
        product->parts_bin_picked++;
        product->bin_is_empty = false;
        //get_object_height_in_bin(product,&bin->remain_parts_height_mm);
        break;
    case 1: // Unknown product
        stopAndShowCodeMsg(2001);
        break;
    case 2: // No objects found
        product->bin_is_empty = true;
        product->height_is_initialized = false;
        bin->bin_status = RC_BIN_FINISH;
        break;
    case 3: // Height initialization has not been performed
        stopAndShowCodeMsg(2003);
        break;
    case 4: // Gripping failed 20 times in a row
        stopAndShowCodeMsg(2004);
        break;
    case 5: // Layer is empty
        product->layer_done = true;
        bin->bin_status = RC_LAYER_FINISH;
        //product->height_is_initialized = false;
    case 9:
        // Loaded product does not contain a bin-picking Session.
        stopAndShowCodeMsg(2009);
        break;
    case 10: // SCAPE Error
        stopAndShowCodeMsg(2010);
        break;
    case 20: // Camera connection error
        stopAndShowCodeMsg(2020);
        break;
    case 21: // Camera images too bright
        stopAndShowCodeMsg(2021);
        break;
    case 22: // Camera images too dark
        stopAndShowCodeMsg(2022);
        break;
    case 23: // Sliding scanner error
        stopAndShowCodeMsg(2023);
        break;
    case 24: // Empty point cloud 3 times in a row
        stopAndShowCodeMsg(2024);
        break;
    case 30:  // No more PICKABLE parts in BIN, but at least one recognized part
    	product->bin_is_empty = true;
        bin->bin_status = RC_UNPICKABLE_PART_IN_BIN;
        product->height_is_initialized = false;
        break;
    case 31:  // No more PICKABLE parts in LAYER, but at least one recognized part
        bin->bin_status = RC_UNPICKABLE_PART_IN_LAYER;
        //product->height_is_initialized = false;
        break;
    case 32:  // No recognized parts in BIN, but SOMETHING was left in bin
    	product->bin_is_empty = true;
        bin->bin_status = RC_SOMETHING_IN_BIN;
        product->height_is_initialized = false;
        break;
    case 33:  // No recognized parts in LAYER, but SOMETHING was left in Layer
        bin->bin_status = RC_SOMETHING_IN_LAYER;
        //product->height_is_initialized = false;
        break;    
    default:
        // Unsupported return code
        stopAndShowCodeMsg(2999);
        break;
    }
    // Move the robot to via low, if EC_PICK_OBJECT
    // does not do it.
    if (result == 0 && leave_robot_at_depart)
    {
        // no meaning here
    }
}

static void start_of_cycle(short reset_statistics)
{
    int result = 0;

    result = EC_StartOfCycle(reset_statistics);
    switch (result)
    {
    case -2: // SCAPE Bin-Picking Manager was stopped
        stopAndShowCodeMsg(902);
        break;
    case -1: // SCAPE Bin-Picking Manager was restarted
        stopAndShowCodeMsg(901);
    case 0: // all ok
        break;
    default: // Unknown error code
        // Unsupported return code
        stopAndShowCodeMsg(1999);
        break;
    }
}

static short place_part_on_hs(Scp_Product *product, short manully_put)
{
    short check_gripper = false, place_part_manually = false;
    int result = 0, place_zone_index = 0;

    if (product->use_bp == false || manully_put)
        place_part_manually = true;
    if (place_part_manually == false)
        request_robot_pose(atHS);
    result = EC_PlaceOnHS(product->product_id, place_part_manually, check_gripper, &product->hs_is_full, &place_zone_index);
    switch (result)
    {
    case -2: // SCAPE Bin-Picking Manager was stopped
        stopAndShowCodeMsg(902);
        break;
    case -1: // SCAPE Bin-Picking Manager was restarted
        stopAndShowCodeMsg(901);
        break;
    case 0: // all ok
        product->parts_on_hs++;
        product->part_placed_on_hs = true;
        return true;
    case 1: // Unknown product
        stopAndShowCodeMsg(14001);
        break;
    case 2: // No part was bin-picked
        stopAndShowCodeMsg(14002);
        break;
    case 3: // Gripper was empty when robot reached HS
        // stopAndShowCodeMsg(14003)
        break;
    case 4: // Handling station is already full!
        stopAndShowCodeMsg(14004);
        break;
    case 5: // Place pose type is ROBOT POSE and PLACE_PART_MANUALLY is FALSE
        stopAndShowCodeMsg(14005);
        break;
    case 6: // Place pose is not relative to Handling Station
        stopAndShowCodeMsg(14006);
        break;
    case 10: // SCAPE Error
        stopAndShowCodeMsg(14010);
        break;
    default:
        // Unsupported return code
        stopAndShowCodeMsg(14999);
        break;
    }
    return false;
}

static void start_oc_recognition(Scp_Product *product, short bWaitForAcqComplete)
{
    int result = 0;

    result = EC_RecognizeAtHS(product->product_id, bWaitForAcqComplete);
    switch (result)
    {
    case -2: // SCAPE Bin-Picking Manager was stopped
        stopAndShowCodeMsg(902);
        break;
    case -1: // SCAPE Bin-Picking Manager was restarted
        stopAndShowCodeMsg(901);
        break;
    case 0: // all ok
        product->oc_recog_started = true;
        break;
    default:
        // Unsupported return code
        stopAndShowCodeMsg(13999);
        break;
    }
}

static void check_oc_recognition_result(Scp_Product *product)
{
    int result = 0;

    product->oc_recog_success = false;
    if (product->oc_recog_started == false)
    {
        return;
    }

    result = EC_GetReturnValue(product->product_id, 13);
    switch (result)
    {
    case -2: // SCAPE Bin-Picking Manager was stopped
        stopAndShowCodeMsg(902);
        break;
    case -1: // SCAPE Bin-Picking Manager was restarted
        stopAndShowCodeMsg(901);
    case 0: // Part was recognized
        product->oc_recog_success = true;
        product->oc_recog_started = false;
        break;
    case 1: // Unknown product
        stopAndShowCodeMsg(13001);
        break;
    case 2: // No part was put on the handling station
        stopAndShowCodeMsg(13002);
        break;
    case 3: // Part on handling station was not recognized
        product->oc_recog_success = false;
        break;
    case 4: // Too much noise on hs
        product->oc_recog_success = false;
        break;
    case 8: // Movement detection timeout
        product->oc_recog_success = false;
        break;
    case 10: // SCAPE Error
        stopAndShowCodeMsg(13010);
        break;
    case 11: // Command Index not supported
        stopAndShowCodeMsg(11011);
        break;
    case 12: // No return value available
        stopAndShowCodeMsg(11012);
        break;
    case 20: // Camera connection error
        stopAndShowCodeMsg(13020);
        break;
    case 21: // Camera images too bright
        stopAndShowCodeMsg(13021);
        break;
    case 22: // Camera images too dark
        stopAndShowCodeMsg(13022);
        break;
    default: // unknown return code
        stopAndShowCodeMsg(13999);
        break;
    }
}

static void perform_regrip_at_hs(Scp_Product *product)
{
    int result = 0;
    short leave_robot_at_depart = true, regrip_manually = false;

    request_robot_pose(atHS);
    product->oc_part_was_picked = false;
    result = EC_RegripAtHS(product->product_id, leave_robot_at_depart, regrip_manually, &product->grip_family_id);
    switch (result)
    {
    case -2: // SCAPE Bin-Picking Manager has been stopped
        stopAndShowCodeMsg(902);
        break;
    case -1: // SCAPE Bin-Picking Manager has been re-started
        stopAndShowCodeMsg(901);
        break;
    case 0: // all ok
        product->parts_on_hs--;
        product->oc_part_was_picked = true;
        product->part_placed_on_hs = false;
        break;
    case 1: // Unknown product
        stopAndShowCodeMsg(3001);
        break;
    case 2: // No objects were recognized
        product->oc_part_was_picked = false;
        break;
    case 3: // Regrip at HS failed
        product->oc_part_was_picked = false;
        break;
    case 10: // SCAPE Error
        stopAndShowCodeMsg(3010);
        break;
    default:
        // Unsupported return code
        stopAndShowCodeMsg(3999);
        break;
    }
}

static void update_gripper_tool(int tool_unit_id, int gripper_id, int gripper_tool_index)
{
    int result = 0;
    double gripper_offset[6];

    result = EC_GetGripperOffset(tool_unit_id, gripper_id, gripper_offset);
    switch (result)
    {
    case -2: // SCAPE Bin-Picking Manager has been stopped
        stopAndShowCodeMsg(902);
        break;
    case -1: // SCAPE Bin-Picking Manager has been re-started
        stopAndShowCodeMsg(901);
        break;
    case 0:    // all ok
        break; //
    case 3:    // Unknown toolunit ID
        stopAndShowCodeMsg(24003);
        break;
    case 4: // Unknown gripper ID
        stopAndShowCodeMsg(24004);
        break;
    default:
        // Unsupported return code
        stopAndShowCodeMsg(24999);
        break;
    }
    // if (gripper_tool_index == SCAPE_TOOL_INDEX)
    //{
    //; "GRIPPER_TOOL_INDEX is the same as SCAPE_TOOL_INDEX. This is not allowed!"
    // stopAndShowCodeMsg(908)
    //}
    // else
    //{
    // TOOL_DATA[GRIPPER_TOOL_INDEX] = TOOL_DATA[SCAPE_TOOL_INDEX] : GRIPPER_OFFSET
    //}
}

static void update_bin(Scp_Product *product, Bin *bin)
{
    product->bin_is_empty = false;
    product->bp_acq_needed = true;
    product->height_is_initialized = false;
    product->parts_bin_picked = 0;

    bin->bin_status = 0;
    bin->picked_parts_count = 0;
    bin->remain_parts_height_mm = 2500;
}

//*******************************************
//             scape output functions
//*******************************************
static int check_robot_config(const Robot *robot)
{
    if (!robot)
        return -1;
    // check robot communication
    if (!(robot->fnConnectToScape))
        return -2;
    if (!(robot->fnDisconnectToScape))
        return -3;
    if (!(robot->fnSendToScape))
        return -4;
    if (!(robot->fnReceiveFromScape))
        return -5;

    if (!(robot->fnRunScapeTask))
        return -6;
    if (!(robot->fnWaitScapeTaskComplete))
        return -7;

    if (!(robot->jGlobalBestConfig))
        return -10;
    // valid message show
    if (!(robot->fnShowMsg))
        return -11;
    if (!(robot->fnWaitSec))
        return -12;
    if (!(robot->fnCleanMsg))
        return -13;
    return 0;
}

static int scape_calibration(void)
{
    cleanMsg();
    currentRobotPose = atUnknown;
    connectToScape("Robot_in_Calibration_Mode;");
    os_core(false);
    return 0;
}

static int scape_initialize(unsigned int product_group_id, unsigned int product_number_in_group)
{
    int i = 0;
    cleanMsg();
    currentRobotPose = atUnknown;
    products_num_of_current_group = product_number_in_group;
    current_selected_product_id = 0;
    if (product_number_in_group > MAX_PRODUCT_NUM_IN_GROUP)
        stopAndShowErr("scape_init too much products!");
    connectToScape("Robot_in_Bin-Picking_Mode;");
    if (_dbg_SCAPE_)
        printf("\nscp: scape_init: wait for scape ready\n");
    waitForScapeReady();
    if (_dbg_SCAPE_)
        printf("\nscp: scape_init: init product group\n");
    init_product_group(product_group_id);

    for (i = 0; i < product_number_in_group; i++)
    {
        reset_product_data(&product_data[i]);
        product_data[i].product_group_id = product_group_id;
        product_data[i].product_id = i + 1;
        set_up_sensor_type(&product_data[i]);
        product_data[i].product_init_done = true;
    }
    current_group_id = product_group_id;
    start_of_cycle(true);
    return 0;
    if (_dbg_SCAPE_)
        printf("\n scape_init completed\n");
}

/// OC ONLY specific code, user will responsible for robot at clear of hs when recognition is called!!!
static void start_oc_recog_if_needed_when_regrip(unsigned bin_idx)
{
    if (product_data[bin_idx].oc_recog_started == false && product_data[bin_idx].parts_on_hs > 0)
    {
        /*
        if (product_data[bin_idx].oc_use_tool_cam == false)
        {
            request_robot_pose(atClearHS);
        }
        */
        start_oc_recognition(&product_data[bin_idx], true);
    }
}

static int scape_pick(Bin *bin, short rescan, PickCfg pickCfg)
{
    int current_pd_idx = 0;
    short need_to_re_bp = false;

    cleanMsg();
    if (bin == NULL)
    {
        stopAndShowErr("scape_pick bin is NULL!");
    }
    
    if (bin->product_id > MAX_PRODUCT_NUM_IN_GROUP || bin->product_id < 1)
    {
        stopAndShowErr("scp pick: product id error!");
    }
    if (bin->product_group_id != current_group_id)
    {
        stopAndShowErr("scp pick: product group id not same as initialized group id!");
    }

    current_selected_product_id = bin->product_id;
    current_pd_idx = current_selected_product_id - 1;

    if (product_data[current_pd_idx].product_init_done == false)
    {
        stopAndShowErr("scp pick: scp init need to be performed!");
    }

    switch (pickCfg)
    {
    case BP_AND_OC:
        if (product_data[current_pd_idx].use_bp == false || product_data[current_pd_idx].use_oc == false)
        {
            stopAndShowErr("scp pick: pick configuration error!");
        }
        request_robot_pose(atBinEntry);
        if (bin->bin_status < 0)
            update_bin(&product_data[current_pd_idx], bin);
        height_init_if_needed(&product_data[current_pd_idx], bin->start_height_mm);
        if (rescan && product_data[current_pd_idx].bp_acq_needed == false)
        {
            acquire_bin_data(&product_data[current_pd_idx], true);
        }

    BP:
        need_to_re_bp = false;
        pick_object(&product_data[current_pd_idx], bin);

        if (product_data[current_pd_idx].bin_is_empty == false)
        {
            perform_oc(&product_data[current_pd_idx], &need_to_re_bp);
        }

        if (need_to_re_bp == true)
        {
            goto BP;
        }

        // ROBOT ALWAYS STOP AT HS !!!
        request_robot_pose(atHS);
        request_robot_pose(atHSExit);

        if (bin->bin_status >= 0)
        {
            start_of_cycle(false);
        }
        run_job(JOB_EXIT, bin->bin_status, bin->remain_parts_height_mm, 0);
        return bin->bin_status;
    case BP_AND_PLACE_ON_HS:
        // IF SUCCEED: ROBOT STOP AT HS
        // IF FAILED: ROBOT STOP AT BIN
        if (product_data[current_pd_idx].use_bp == false || product_data[current_pd_idx].use_oc == false || product_data[current_pd_idx].total_zones_on_hs != 1 || product_data[current_pd_idx].oc_use_tool_cam)
        {
            stopAndShowErr("scp pick: pick configuration error!");
        }

        request_robot_pose(atBinEntry);
        if (bin->bin_status < 0)
            update_bin(&product_data[current_pd_idx], bin);
        height_init_if_needed(&product_data[current_pd_idx], bin->start_height_mm);
        if (rescan && product_data[current_pd_idx].bp_acq_needed == false)
        {
            acquire_bin_data(&product_data[current_pd_idx], true);
        }

        pick_object(&product_data[current_pd_idx], bin);

        if (product_data[current_pd_idx].bin_is_empty == false)
        {
            request_robot_pose(atHSEntry);
            // what if two zones????
            if (product_data[current_pd_idx].parts_on_hs > 0)
            {
                move_clear_and_empty_hs(&product_data[current_pd_idx]);
            }
            place_part_on_hs(&product_data[current_pd_idx], false);
        }

        if (bin->bin_status >= 0)
        {
            request_robot_pose(atHS);
            request_robot_pose(atHSExit);
            // start oc recognition.
            start_oc_recognition(&product_data[current_pd_idx], false);
            start_of_cycle(false);
        }
        run_job(JOB_EXIT, bin->bin_status, bin->remain_parts_height_mm, 0);
        return bin->bin_status;
    case BP_ONLY:
        if (product_data[current_pd_idx].use_bp == false)
        {
            stopAndShowErr("scp pick: pick configuration error!");
        }

        request_robot_pose(atBinEntry);
        if (bin->bin_status < 0)
            update_bin(&product_data[current_pd_idx], bin);
        height_init_if_needed(&product_data[current_pd_idx], bin->start_height_mm);
        if (rescan)
        {
            acquire_bin_data(&product_data[current_pd_idx], true);
        }

        pick_object(&product_data[current_pd_idx], bin);

        request_robot_pose(atBin);
        request_robot_pose(atBinExit);
        if (bin->bin_status >= 0)
        {
            start_of_cycle(false);
        }
        run_job(JOB_EXIT, bin->bin_status, bin->remain_parts_height_mm, 0);
        return bin->bin_status;
    case OC_ONLY:
        if (product_data[current_pd_idx].use_oc == false || product_data[current_pd_idx].oc_use_tool_cam)
        {
            stopAndShowErr("scp pick: pick configuration error!");
        }

        request_robot_pose(atHSEntry);
        // wait part was placed on HS, TP program make sure this
        if (product_data[current_pd_idx].part_placed_on_hs == false)
        {
            place_part_on_hs(&product_data[current_pd_idx], true);
        }

        start_oc_recog_if_needed_when_regrip(current_pd_idx);
        check_oc_recognition_result(&product_data[current_pd_idx]);
        if (product_data[current_pd_idx].oc_recog_success)
        {
            perform_regrip_at_hs(&product_data[current_pd_idx]);
        }

        if (product_data[current_pd_idx].oc_recog_success == false || product_data[current_pd_idx].oc_part_was_picked == false)
        {
            move_clear_and_empty_hs(&product_data[current_pd_idx]);
        }

        if (product_data[current_pd_idx].oc_part_was_picked)
        {

            bin->bin_status = product_data[current_pd_idx].grip_family_id;
        }
        else
        {

            bin->bin_status = RC_OC_FAILED;
        }
        
        request_robot_pose(atHS);
        request_robot_pose(atHSExit);
        if (bin->bin_status >= 0)
        {
            start_of_cycle(false);
        }
        run_job(JOB_EXIT, bin->bin_status, 0, 0);
        return bin->bin_status;
    default:
        stopAndShowErr("scape_pick: undefined configuration.");
    }
    return -999;
}

/*tuo pu project specific code*/
const double tp_start_pose[6] = {435.359,-39.513,-212.004,0.0023,-0.0081,176.9633};
const double tp_start_angle[6] = {-3.5071,-30.1972,-71.0887,0.003,-49.1165,0.4683};
const int CHANGE_BIN = -1;
const int OC_NO_PART_CAN_PICK = 0;
const int OC_HAS_PART_TO_PICK = 1;
const int OC_NEED_CLEAN = -1;

void get_task_core(short run_in_ec_mode,Scape_Task_Internal tasks[],short* task_len)
{
    short io_ready = false, task_ready = false, stay_in_os_core = false;
    short task_type = 0, task_left = -1, task_counter = 0, i = 0;

    short ret_core_need = false;

    *task_len = 0;
    while (1)
    {
        is_IO_or_task_ready(&io_ready, &task_ready, &task_type, &stay_in_os_core);
        if (stay_in_os_core == false && run_in_ec_mode) return;
        if (io_ready){
            setVal("SET GripSensor1_VALUE 1 GripSensor2_VALUE 1 GripSensor3_VALUE 1;");
            setVal("SET ROBOT_SIGNAL_READY 0;");
        }
        if (task_ready){
            if (ret_core_need) stopAndShowErr("get task core error, ret core need!");
            tasks[task_counter] = get_task(task_type, &task_left, task_counter);
            if (tasks[task_counter].user_task.job_id == JOB_GET_TCP_POSE){
                for (i=0;i<6;i++) tasks[task_counter].user_task.target[i] = tp_start_pose[i];
                send_scape_tcp_pose(tasks[task_counter]);
                continue;
            }
            else if (tasks[task_counter].user_task.job_id == JOB_GET_JOINTS){
                for (i=0;i<6;i++) tasks[task_counter].user_task.joints[i] = tp_start_angle[i];
                send_scape_joint_value(tasks[task_counter]);
                continue;
            }
            task_counter++;
            *task_len = task_counter;
            if (task_counter > MAX_SCAPE_TASKS_LEN - 1) stopAndShowErr("scape tasks too much!");
        }

        if (task_left == 0 && task_counter > 0){
            ret_core_need = true;
            setVal("SET ROBOT_TASKS_LEFT 0;");
            task_counter = 0;
            task_left = -1;
        }
    }
}

int EC_GetPickObjectTask(int iProduct, int *GripFamilyID, short *CalibrationIsNeed, short *AcqIsNeed,Scape_Task_Internal tasks[],short* task_len)
{
    char cmd[BUFF_LEN] = "SET_GROUP_VALUES PICKOBJSETGROUP 2 %d %d %d;";
    int ret = 0;
    setVal(getStr(cmd, 3, iProduct, 0, 1));
    get_task_core(true,tasks,task_len);
    ret = EC_WaitForCompletion(2);
    getVal("GET_GROUP_VALUES PICKOBJGETGROUP;");
    *GripFamilyID = (int)tempData[7];
    *CalibrationIsNeed = (int)tempData[8];
    *AcqIsNeed = (int)tempData[9];
    return ret;
}

void tp_get_pick_object_task(Scp_Product *product, Bin *bin,Scape_Task_Internal tasks[],short* len)
{
    int result = 0;

    request_robot_pose(atBin);
    bin->bin_status = 0;
    if (product->use_bp == false)
        return;
    result = EC_GetPickObjectTask(product->product_id,&product->grip_family_id, &product->bp_calib_needed, &product->bp_acq_needed,tasks,len);
    switch (result)
    {
    case -2: // SCAPE Bin-Picking Manager was stopped
        stopAndShowCodeMsg(902);
        break;
    case -1: // SCAPE Bin-Picking Manager was restarted
        stopAndShowCodeMsg(901);
        break;
    case 0: // all ok
        bin->bin_status = product->grip_family_id;
        bin->picked_parts_count++;
        product->parts_bin_picked++;
        product->bin_is_empty = false;
        get_object_height_in_bin(product,&bin->remain_parts_height_mm);
        break;
    case 1: // Unknown product
        stopAndShowCodeMsg(2001);
        break;
    case 2: // No objects found
        product->bin_is_empty = true;
        product->height_is_initialized = false;
        bin->bin_status = RC_BIN_FINISH;
        break;
    case 3: // Height initialization has not been performed
        stopAndShowCodeMsg(2003);
        break;
    case 4: // Gripping failed 20 times in a row
        stopAndShowCodeMsg(2004);
        break;
    case 5: // Layer is empty
        product->layer_done = true;
        bin->bin_status = RC_LAYER_FINISH;
        //product->height_is_initialized = false;
    case 9:
        // Loaded product does not contain a bin-picking Session.
        stopAndShowCodeMsg(2009);
        break;
    case 10: // SCAPE Error
        stopAndShowCodeMsg(2010);
        break;
    case 20: // Camera connection error
        stopAndShowCodeMsg(2020);
        break;
    case 21: // Camera images too bright
        stopAndShowCodeMsg(2021);
        break;
    case 22: // Camera images too dark
        stopAndShowCodeMsg(2022);
        break;
    case 23: // Sliding scanner error
        stopAndShowCodeMsg(2023);
        break;
    case 24: // Empty point cloud 3 times in a row
        stopAndShowCodeMsg(2024);
        break;
    case 30:  // No more PICKABLE parts in BIN, but at least one recognized part
    	product->bin_is_empty = true;
        bin->bin_status = RC_UNPICKABLE_PART_IN_BIN;
        product->height_is_initialized = false;
        break;
    case 31:  // No more PICKABLE parts in LAYER, but at least one recognized part
        bin->bin_status = RC_UNPICKABLE_PART_IN_LAYER;
        //product->height_is_initialized = false;
        break;
    case 32:  // No recognized parts in BIN, but SOMETHING was left in bin
    	product->bin_is_empty = true;
        bin->bin_status = RC_SOMETHING_IN_BIN;
        product->height_is_initialized = false;
        break;
    case 33:  // No recognized parts in LAYER, but SOMETHING was left in Layer
        bin->bin_status = RC_SOMETHING_IN_LAYER;
        //product->height_is_initialized = false;
        break;    
    default:
        // Unsupported return code
        stopAndShowCodeMsg(2999);
        break;
    }
}

int EC_GetPlaceOnHSTask(int iProduct, short *HSIsFull,Scape_Task_Internal tasks[],short* task_len)
{
    char cmd[BUFF_LEN] = "SET_GROUP_VALUES PLACEONHSSETGROUP 14 %d %d %d;";
    int ret = 0;
    setVal(getStr(cmd, 3, iProduct, 1, 0));
    get_task_core(true,tasks,task_len);
    ret = EC_WaitForCompletion(14);
    getVal("GET ExtControlParamOut0 ExtControlParamOut1;");
    *HSIsFull = (short)tempData[0];
    return ret;
}

short tp_get_place_part_on_hs_task(Scp_Product *product,Scape_Task_Internal tasks[],short* task_len)
{
    int result = 0;

    result = EC_GetPlaceOnHSTask(product->product_id, &product->hs_is_full,tasks,task_len);
    switch (result)
    {
    case -2: // SCAPE Bin-Picking Manager was stopped
        stopAndShowCodeMsg(902);
        break;
    case -1: // SCAPE Bin-Picking Manager was restarted
        stopAndShowCodeMsg(901);
        break;
    case 0: // all ok
        product->parts_on_hs++;
        product->part_placed_on_hs = true;
        return true;
    case 1: // Unknown product
        stopAndShowCodeMsg(14001);
        break;
    case 2: // No part was bin-picked
        stopAndShowCodeMsg(14002);
        break;
    case 3: // Gripper was empty when robot reached HS
        // stopAndShowCodeMsg(14003)
        break;
    case 4: // Handling station is already full!
        stopAndShowCodeMsg(14004);
        break;
    case 5: // Place pose type is ROBOT POSE and PLACE_PART_MANUALLY is FALSE
        stopAndShowCodeMsg(14005);
        break;
    case 6: // Place pose is not relative to Handling Station
        stopAndShowCodeMsg(14006);
        break;
    case 10: // SCAPE Error
        stopAndShowCodeMsg(14010);
        break;
    default:
        // Unsupported return code
        stopAndShowCodeMsg(14999);
        break;
    }
    return false;
}

int EC_GetRegripAtHSTask(int iProduct, int *pGripFamilyID,Scape_Task_Internal tasks[],short* task_len)
{
    char cmd[BUFF_LEN] = "SET_GROUP_VALUES REGRIPGROUP 3 %d %d %d;";
    int ret = 0;
    setVal(getStr(cmd, 3, iProduct, 0, 0));
    // Run SCAPE_OS_CORE to let SCAPE control the robot while a part is being picked.
    get_task_core(true,tasks,task_len);
    // Wait until SCAPE has carried out the EC API Command
    ret = EC_WaitForCompletion(3);
    getVal("GET ExtControlParamOut0;");
    *pGripFamilyID = tempData[0];
    return ret;
}

void tp_get_regrip_at_hs_task(Scp_Product *product,Scape_Task_Internal tasks[],short* task_len)
{
    int result = 0;


    product->oc_part_was_picked = false;
    result = EC_GetRegripAtHSTask(product->product_id, &product->grip_family_id,tasks,task_len);
    switch (result)
    {
    case -2: // SCAPE Bin-Picking Manager has been stopped
        stopAndShowCodeMsg(902);
        break;
    case -1: // SCAPE Bin-Picking Manager has been re-started
        stopAndShowCodeMsg(901);
        break;
    case 0: // all ok
        product->parts_on_hs--;
        product->oc_part_was_picked = true;
        product->part_placed_on_hs = false;
        break;
    case 1: // Unknown product
        stopAndShowCodeMsg(3001);
        break;
    case 2: // No objects were recognized
        product->oc_part_was_picked = false;
        break;
    case 3: // Regrip at HS failed
        product->oc_part_was_picked = false;
        break;
    case 10: // SCAPE Error
        stopAndShowCodeMsg(3010);
        break;
    default:
        // Unsupported return code
        stopAndShowCodeMsg(3999);
        break;
    }
}

// check_oc_recognition_result(Scp_Product *product)
int tp_check_oc_result(Scp_Product* product){
    int i = 0;
    if (product->parts_on_hs == 0) return OC_NO_PART_CAN_PICK;
    check_oc_recognition_result(product);
    if (product->oc_recog_success == false) return OC_NEED_CLEAN;
    return OC_HAS_PART_TO_PICK;
}

ScapeTask tp_get_via_task(const int product)
{
    Scape_Task_Internal task={0};
    int i;
    task.task_was_executed = false;
    task.user_task.taskIndex = 1;
    task.user_task.taskNumInTotal = 1;
    task.user_task.motionValid = false;
    task.user_task.motionType = NotMov;
    // if (product == 1){
    //     task.user_task.target[0] = 974;
    //     task.user_task.target[1] = -236;
    //     task.user_task.target[2] = -279;
    //     task.user_task.target[3] = 0;
    //     task.user_task.target[4] = 0;
    //     task.user_task.target[5] = 177;
    //     task.user_task.joints[0] = -12;
    //     task.user_task.joints[1] = 29;
    //     task.user_task.joints[2] = -20;
    //     task.user_task.joints[3] = 0;
    //     task.user_task.joints[4] = -42;
    //     task.user_task.joints[5] = 10;
    // }
    // else{
    //     task.user_task.target[0] = 974;
    //     task.user_task.target[1] = 224;
    //     task.user_task.target[2] = -179;
    //     task.user_task.target[3] = 0;
    //     task.user_task.target[4] = 0;
    //     task.user_task.target[5] = 177;
    //     task.user_task.joints[0] = 13.3;
    //     task.user_task.joints[1] = 28;
    //     task.user_task.joints[2] = -19;
    //     task.user_task.joints[3] = 0;
    //     task.user_task.joints[4] = -41;
    //     task.user_task.joints[5] = -16;
    // }
    task.user_task.speed = 100;
    task.user_task.acc = 100;
    task.user_task.blend = 200;
    task.user_task.logicValid = true;
    task.user_task.job_id = 226;
    task.user_task.par0 = product;
    task.user_task.par1 = 0;
    task.user_task.par2 = 0;
    return task.user_task;
}

void tp_move_to_via_pos(Scp_Product* product){
    ScapeTask via_task = tp_get_via_task(product->product_id);
    scapeRobot->fnRunScapeTask(&via_task);
}
void tp_clear_and_empty_hs(Scp_Product *product);
int tp_pick_and_place_on_HS(Scp_Product* product,Bin *bin){
    pick_object(product,bin);
    if (product->bin_is_empty) return CHANGE_BIN;
    //tp_move_to_via_pos(product);
    int check_result = tp_check_oc_result(product);
    if (check_result == OC_NEED_CLEAN){
        tp_clear_and_empty_hs(product);
    }
    place_part_on_hs(product,false);
    if (!product->oc_recog_started && product->parts_on_hs > 0) start_oc_recognition(product, false);
    return product->grip_family_id;
}

int tp_get_pick_and_place_on_HS(Scp_Product* product,Bin* bin,Scape_Task_Internal tasks[],short* len){
    Scape_Task_Internal _tasks[15];
    short _len=0,j;
    tp_get_pick_object_task(product,bin,tasks,len);
    if (product->bin_is_empty) return CHANGE_BIN;
    ScapeTask via_task = tp_get_via_task(product->product_id);
    tasks[*len].user_task = via_task;
    (*len)++;
    tp_get_place_part_on_hs_task(product,_tasks,&_len);
    for(j=0;j<_len;j++) {
        tasks[j+(*len)] = _tasks[j];
    }
    return product->grip_family_id;
}

ScapeTask tp_get_ExitJob(const int counter,const int p0,const int p1,const int p2)
{
    Scape_Task_Internal task={0};
    int i;
    task.task_was_executed = false;
    task.user_task.taskIndex = counter;
    task.user_task.taskNumInTotal = counter;
    task.user_task.motionValid = false;
    task.user_task.motionType = NotMov;
    for (i = 0; i < 6; i++)
        task.user_task.target[i] = 0;
    for (i = 0; i < SCAPE_ROBOT_AXIS_NUM; i++)
        task.user_task.joints[i] = 0;
    task.user_task.speed = 0;
    task.user_task.acc = 0;
    task.user_task.blend = 0;
    task.user_task.logicValid = true;
    task.user_task.job_id = JOB_EXIT;
    task.user_task.par0 = p0;
    task.user_task.par1 = p1;
    task.user_task.par2 = p2;
    return task.user_task;
}

void tp_clear_and_empty_hs(Scp_Product *product)
{
    run_job(JOB_MOV_CLEAR_HS,product->product_group_id, product->product_id, 0);
    run_job(JOB_EMPTY_HS, product->product_group_id, product->product_id, 0);
    if (product->use_oc)
    {
        product->oc_recog_success = false;
        product->oc_recog_started = false;
        product->parts_on_hs = 0;
        product->oc_part_was_picked = false;
        product->part_placed_on_hs = false;
    }
}

void tp_empty_hs(Scp_Product *product)
{
    run_job(JOB_EMPTY_HS, product->product_group_id, product->product_id, 0);
    if (product->use_oc)
    {
        product->oc_recog_success = false;
        product->oc_recog_started = false;
        product->parts_on_hs = 0;
        product->oc_part_was_picked = false;
        product->part_placed_on_hs = false;
    }
}

static void waitSec(float sec)
{
    mpTaskDelay((int)(1000 * sec/mpGetRtc()));
}

static void put_i_val(int var,unsigned short idx)
{
    MP_USR_VAR_INFO varInfo;

    memset(&varInfo, 0, sizeof(varInfo));
    varInfo.var_type = MP_VAR_I;
    varInfo.var_no = idx;
    varInfo.val.i = var;
    while(1)
    {
        if(mpPutUserVars(&varInfo) == 0) break;
        waitSec(0.01);
    }
}
int tp_pick(Bin *bin, short forcescan){
    Scape_Task_Internal t_pick_place[15];
    Scape_Task_Internal t_regrip[10];
    ScapeTask t_robot[30];
    short t_len_pick_place=0, t_len_regrip = 0,t_len_robot=0;
    Scp_Product* current_product = NULL;
    BOOL needscan = false;
    int check_result = 0 ,rc = 0,i=0;
    if (bin == NULL) return -10000;
    current_product = &product_data[bin->product_id - 1];
    height_init_if_needed(current_product, bin->start_height_mm);
    acquire_bin_data(current_product, forcescan);
    check_result = tp_check_oc_result(current_product);
    if (check_result == OC_NEED_CLEAN){
        tp_empty_hs(current_product);
    }
    pick_object(current_product,bin);
    if (current_product->bin_is_empty){
        rc = CHANGE_BIN;
        put_i_val(rc, 65);
        current_product->grip_family_id = rc;
        ScapeTask finalTask = tp_get_ExitJob(1,current_product->grip_family_id,0,0);
        scapeRobot->fnRunScapeTask(&finalTask);
        if (!current_product->oc_recog_started && current_product->parts_on_hs > 0) start_oc_recognition(current_product, false);
        return rc;
    }
    if (check_result == OC_HAS_PART_TO_PICK){
        run_job(226,current_product->product_group_id, current_product->product_id, 0);
        place_part_on_hs(current_product,false);
        run_job(226,current_product->product_group_id, current_product->product_id, 0);
        perform_regrip_at_hs(current_product);
    }
    else {
        PLACE:
        run_job(226,current_product->product_group_id, current_product->product_id, 0);
        place_part_on_hs(current_product,false);
        run_job(226,current_product->product_group_id, current_product->product_id, 0);
        start_oc_recognition(current_product, false);
        if(needscan){
            run_job(227,current_product->product_group_id, current_product->product_id, 0);
            needscan = false;
        }
        pick_object(current_product,bin);
        if (current_product->bin_is_empty){
            rc = CHANGE_BIN;
            put_i_val(rc, 65);
            current_product->grip_family_id = rc;
            ScapeTask finalTask = tp_get_ExitJob(1,current_product->grip_family_id,0,0);
            scapeRobot->fnRunScapeTask(&finalTask);
            if (!current_product->oc_recog_started && current_product->parts_on_hs > 0) start_oc_recognition(current_product, false);
            return rc;
        }
        run_job(226,current_product->product_group_id, current_product->product_id, 0);
        check_result = tp_check_oc_result(current_product);
        if (check_result == OC_HAS_PART_TO_PICK){
            run_job(226,current_product->product_group_id, current_product->product_id, 0);
            place_part_on_hs(current_product,false);
            run_job(226,current_product->product_group_id, current_product->product_id, 0);
            perform_regrip_at_hs(current_product);
        }
        else{
            tp_clear_and_empty_hs(current_product);
            needscan = true;
            goto PLACE;
        }
    }
    put_i_val(current_product->grip_family_id, 65);
    ScapeTask finalTask = tp_get_ExitJob(1,current_product->grip_family_id,0,0);
    scapeRobot->fnRunScapeTask(&finalTask);
    if (!current_product->oc_recog_started && current_product->parts_on_hs > 0) start_oc_recognition(current_product, false);
    start_of_cycle(false);
    return current_product->grip_family_id;
}


//int tp_pick(Bin *bin){

    // 返回非负 GripFaimily ID, 返回负数 箱子需要移动
    // Scape_Task_Internal t_pick_place[15];
    // Scape_Task_Internal t_regrip[10];
    // ScapeTask t_robot[30];
    // short t_len_pick_place=0, t_len_regrip = 0,t_len_robot=0;
    // Scp_Product* current_product = NULL;
    // int check_result = 0 ,rc = 0,i=0;
    
    // if (bin == NULL) {
    //     put_i_val(-1, 65);
    //     ScapeTask finalTask = tp_get_ExitJob(1,current_product->grip_family_id,0,0);
    //     scapeRobot->fnRunScapeTask(&finalTask);
    //     return -10000;
    // }
    // current_product = &product_data[bin->product_id - 1];
    // height_init_if_needed(current_product, bin->start_height_mm);
    // acquire_bin_data(current_product, current_product->bp_acq_needed);
    // check_result = tp_check_oc_result(current_product);
    // if (check_result == OC_NEED_CLEAN) tp_clear_and_empty_hs(current_product);
    // pick_object(current_product,bin);
    // if (current_product->bin_is_empty) {
    //     put_i_val(-1, 65);
    //     ScapeTask finalTask = tp_get_ExitJob(1,current_product->grip_family_id,0,0);
    //     scapeRobot->fnRunScapeTask(&finalTask);
    //     return CHANGE_BIN;
    // }
    // if (check_result == OC_HAS_PART_TO_PICK){
    //     place_part_on_hs(current_product,false);
    //     perform_regrip_at_hs(current_product);
    // }
    // else {
    //     PLACE:
    //     place_part_on_hs(current_product,false);
    //     start_oc_recognition(current_product, false);
    //     pick_object(current_product,bin);
    //     if (current_product->bin_is_empty) {
    //         put_i_val(-1, 65);
    //         ScapeTask finalTask = tp_get_ExitJob(1,current_product->grip_family_id,0,0);
    //         scapeRobot->fnRunScapeTask(&finalTask);
    //         return CHANGE_BIN;
    //     }
    //     check_result = tp_check_oc_result(current_product);
    //     if (check_result == OC_HAS_PART_TO_PICK){
    //         place_part_on_hs(current_product,false);
    //         perform_regrip_at_hs(current_product);
    //     }
    //     else{
    //         tp_clear_and_empty_hs(current_product);
    //         goto PLACE;
    //     }
    // }
    // ScapeTask finalTask = tp_get_ExitJob(1,current_product->grip_family_id,0,0);
    // scapeRobot->fnRunScapeTask(&finalTask);
    // if (!current_product->oc_recog_started && current_product->parts_on_hs > 0) start_oc_recognition(current_product, false);
    // start_of_cycle(false);
    // put_i_val(current_product->grip_family_id, 65);
    // return current_product->grip_family_id;


    // if (){
        
    //     // int check_result = tp_check_oc_result(current_product);
    //     // if (check_result == OC_NEED_CLEAN){
    //     //     tp_clear_and_empty_hs(current_product);
    //     // }
    //     // place_part_on_hs(current_product,false);
    //     // if (rc < 0) return CHANGE_BIN;
    //     // if (current_product->parts_on_hs < 2){
    //     //     //tp_move_to_via_pos(current_product);
    //     //     // rc = tp_pick_and_place_on_HS(current_product,bin);
    //     //     pick_object(current_product,bin);
    //     //     if (current_product->bin_is_empty) return CHANGE_BIN;
    //     //     //tp_move_to_via_pos(current_product);
    //     //     int check_result = tp_check_oc_result(current_product);
    //     //     if (check_result == OC_NEED_CLEAN){
    //     //         tp_clear_and_empty_hs(current_product);
    //     //         goto BPP;
    //     //     }
    //     //     place_part_on_hs(current_product,false);
    //     //     if (rc < 0) return CHANGE_BIN;
    //     // }
    //     // perform_regrip_at_hs(current_product);
    //     ScapeTask finalTask = tp_get_ExitJob(1,current_product->grip_family_id,0,0);
    //     scapeRobot->fnRunScapeTask(&finalTask);
    //     if (!current_product->oc_recog_started && current_product->parts_on_hs > 0) start_oc_recognition(current_product, false);
    //     start_of_cycle(false);
    //     return 0;
    // }

    // if (check_result == OC_HAS_PART_TO_PICK){ // oc 有一个可以抓取的产品
    //     rc = tp_get_pick_and_place_on_HS(current_product,bin,t_pick_place,&t_len_pick_place);
    //     if (rc < 0) return CHANGE_BIN;
    //     tp_get_regrip_at_hs_task(current_product,t_regrip,&t_len_regrip);
    //     t_len_robot = t_len_pick_place + t_len_regrip;
    //     for(i=0;i<t_len_pick_place;i++){
    //         t_robot[i] = t_pick_place[i].user_task;
    //     }
    //     for(i=0;i<t_len_regrip;i++){
    //         t_robot[i+t_len_pick_place] = t_regrip[i].user_task;
    //     }
    //     for(i=0;i<t_len_robot;i++){
    //         t_robot[i].taskIndex = i+1;
    //         t_robot[i].taskNumInTotal = t_len_robot;
    //     }
    //     t_robot[t_len_robot] = tp_get_ExitJob(t_len_robot+1,current_product->grip_family_id,0,0);
    //     t_len_robot++;
    //     for(i=0;i<t_len_robot;i++){
    //         scapeRobot->fnRunScapeTask(&t_robot[i]);
    //     }
    //     if (!current_product->oc_recog_started && current_product->parts_on_hs > 0) start_oc_recognition(current_product, false);
    //     start_of_cycle(false);
    //     return 0;
    // }
//}

/*tuo pu project specific code*/


static int scape_start_scan(Bin* bin)
{
    //if (bin->bin_status < 0) return 0;
    height_init_if_needed(&product_data[bin->product_id - 1], bin->start_height_mm);
    acquire_bin_data(&product_data[bin->product_id - 1], product_data[bin->product_id - 1].bp_acq_needed);
    return 0;
}

static int scape_start_hs_recognition(Bin* bin)
{
    int pd_idx;

    pd_idx = bin->product_id - 1;
    if (product_data[pd_idx].oc_recog_started)
    {
        return 1;
    }

    if (product_data[pd_idx].part_placed_on_hs == false)
    {
        place_part_on_hs(&product_data[pd_idx], true);
    }

    start_oc_recognition(&product_data[pd_idx], false);
    return 0;
}

int init_robot(Robot *robot, IScp *scp)
{
    int rc = 0;
    // if robot configuration not complete then return -1
    rc = check_robot_config(robot);
    if (rc != 0)
    {
        if (_dbg_SCAPE_)
            printf("\nscp: check_rob_config falid: %d\n", rc);
        return -1;
    }

    if (scp == NULL)
    {
        if (_dbg_SCAPE_)
            printf("\nscp: init_robot, scp is NULL\n");
        return -2;
    }
    scapeRobot = robot;
    scp->scp_calibration = scape_calibration;
    scp->scp_initialize = scape_initialize;
    scp->scp_pick = scape_pick;
    scp->scp_start_scan = scape_start_scan;
    scp->scp_start_handling_station_recog = scape_start_hs_recognition;
    scp->TPPick = tp_pick;
    return 0;
}
