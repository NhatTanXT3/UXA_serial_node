#include "uxa_serial.h"

#define PC2MCU_TERMINATOR_  0xFE
#define PC2MCU_HEADER_      0xFF
#define ENABLE_ACTUATOR_

/*
 * =========== timer variable ===========
 */
#define LOOP_RATE_1000Hz_ 1000

/*
 * COUNT is defined for timer 1ms
 */
#define COUNT_50_HZ_	20
#define COUNT_100_HZ_	10
struct TimerCountType{
    unsigned char Hz_100;
    unsigned char Hz_50;
}Timer_Count;

struct FlagTimerType{
    unsigned char Hz_50:1;
    unsigned char Hz_100:1;
}FlagTimer;
//======timer handler run in 1000Hz=========
void Timer_handler(){
    Timer_Count.Hz_50++;
    Timer_Count.Hz_100++;
    if(Timer_Count.Hz_50==COUNT_50_HZ_){
        Timer_Count.Hz_50=0;
        FlagTimer.Hz_50=1;
    }
    if(Timer_Count.Hz_100==COUNT_100_HZ_)
    {
        Timer_Count.Hz_100=0;
        FlagTimer.Hz_100=1;
    }
}
//=====received data handle==============


/*
 * ===============  communication=====================
 */
//#define PC_SAM_READ_ALL_POS12_ 0xF0
#define PC_SAM_READ_ALL_POS12_ 0xCC
//#define PC_SAM_SP_MODE_3_ 0xAA
//#define PC_SAM_SP_MODE_4_ 0x95
//#define PC_SAM_SP_MODE_5_ 0xEC
//#define PC_SAM_SP_MODE_6_ 0x88
//#define PC_SAM_SP_MODE_7_ 0x81
//#define PC_SAM_SP_MODE_8_ 0xBB

//#define PC_SAM_SP_MODE_9_ 0xBD//Set Average Torque of many SAMs
//#define PC_SAM_SP_MODE_10_ 0xBF//Read Average Torque of many SAMs

//#define PC_SAM_SP_MODE_11_ 0xC1 //Set quick PD of many SAMs
//#define PC_SAM_SP_MODE_12_ 0xC3//Read Quick PD of many SAMs
unsigned char flag_capture=0;
unsigned char flag_receive=0;
unsigned char dataIndex=0;
struct Communication{
    unsigned char flagDataReceived:1;
    unsigned char flagEnableCapute:1;
    unsigned char flagDataReceived_readAllPos12:1;
    unsigned char flagDataAvailable_readAllPos12:1;

    unsigned char flagDataReceived_readAllPos8:1;

    unsigned char dataIndex;
    unsigned char NumofSam;
    unsigned int samPos12[24];
    unsigned char samPos12Avail[24];
}myCom;
void Recev_Data_hanlder();
/*
 * ============================== Scene =========================
 */
class Scene_class{
    unsigned int samTorq[12];
    unsigned char samP[12];
    unsigned char samI[12];
    unsigned char samD[12];
    double slope_begin[12];
    double slope_end[12];
    double slope_midle[12];
public:
    unsigned int *beginPose=NULL;
    unsigned int *endPose=NULL;
    unsigned int frame;
    unsigned int numOfFrame;
    struct Flag{
        unsigned char start:1;
        unsigned char finish:1;
        unsigned char enable:1;
        unsigned char delay:1;
    }flag;
    void setFrame(unsigned int value);
    void setBeginPose(const unsigned int *value);
    void setEndPose(const unsigned int *value);
    void setNumOfFrame(unsigned int value);
    void setDelayScene(unsigned int fr);
    void setUpMyScene(unsigned int fr, const unsigned int *beginpose,const unsigned int *endpose);
    Scene_class(){

    }
};

Scene_class myscene;
/*
 * =========== Task======================
 */

#define TASK_INIT_ 0
#define TASK_TEST_ 1
#define TASK_SITDOWN_  2
#define TASK_STANDING_  3
#define TASK_INIT_WALKING_  4
#define TASK_WALK_1STEP_   5
#define TASK_POSES_TEST_  6

#define TASK_IDLE_   10
typedef struct{
    unsigned char startFlag:1;
    unsigned char finishFlag:1;
    unsigned char id;
    unsigned char scene;
}task_type;
task_type task;


unsigned char Trans_chr[_SERIAL_BUFF_SIZE];
unsigned char Recev_chr[_SERIAL_BUFF_SIZE];
unsigned char Store_chr[_SERIAL_BUFF_SIZE];


const int samPos12_offset[12]={1640,1689,2050,2052,663,3446,1235,2910,2761,2163,1260,2341};// zero of the real system 12bits
const unsigned char hardZeroPos[12]={96,100,127,128,21,235,65,193,182,136,67,150};// zero of the real system 8bit
const int offsetZeroPos12[12]={-404,-355,6,8,-351,372,-809,866,717,119,-784,297};// (hardZero-softZero)*pos8toPos12

const double degree = M_PI/180;
const double pos12bitTorad=0.083*M_PI/180;
const double pos8bitTorad=1.08*M_PI/180;
const double pos8toPos12_1=13.039525692; //3299/253
const double pos8toPos12_2=387.960474308;//98154/253

const unsigned char default_samP[12]={30,30,30,30,30,30,30,30,30,30,30,30};
const unsigned char default_samI[12]={0,0,0,0,0,0,0,0,0,0,0,0};
const unsigned char default_samD[12]={5,5,5,5,5,5,5,5,5,5,5,5};
const unsigned int default_averageTorq[12]={2000,2000,2000,2000,2000,2000,2000,2000,2000,2000,2000,2000};

/*
 * ================================= use able Pose =======================
 */
const unsigned char softZeroPos[12]={127,127,127,127,48,206,127,127,127,127,127,127};// zero of the virtual system
const unsigned int softZeroPos12[12]={2044,2044,2044,2044,1014,3074,2044,2044,2044,2044,2044,2044};// zero of the virtual system

//====== standing pose for walking=============//
const unsigned char Pose_init_walking[12]={125,125,104,150,88,166,145,109,126,128,129,125};
//====== sitdown=============//
const unsigned char Pose_sitdown[12]={127,127,77,177,190,64,250,4,124,130,134,124};
const unsigned int sitdown_averageTorq[12]={3000,3000,3000,3000,3000,3000,3000,3000,3000,3000,3000,3000};
const unsigned char sitdown_samP[12]={40,40,40,40,40,40,40,40,40,40,40,40};
const unsigned char sitdown_samD[12]={15,15,15,15,15,15,15,15,15,15,15,15};
//====== standing pose============//
const unsigned char Pose_standing_0[12]={127,127,77,177,190,64,250,4,124,130,134,124};
const unsigned char Pose_standing_1[12]={127,127,87,167,156,98,230,24,127,127,129,125};
const unsigned char Pose_standing_2[12]={127,127,122,132,46,208,126,128,127,127,129,125};

const unsigned int numOfFrames_standing[2]={124,241};

//======pose for walking 1 step- M04===========//
//const unsigned char Pose_M04_0[12]={126,128,102,150,89,166,146,109,126,128,129,125};
const unsigned char Pose_M04_0[12]={125,125,104,150,88,166,145,109,126,128,129,125};// stading pose
const unsigned char Pose_M04_1[12]={137,135,102,150,89,166,146,109,117,118,129,125};// COM transfer to left foot
const unsigned char Pose_M04_2[12]={134,140,87,153,129,166,166,109,118,125,129,125};// left standing
const unsigned char Pose_M04_3[12]={135,139,95,153,89,175,151,127,117,126,131,125};// right step forward, toe contact

const unsigned char Pose_M04_4[12]={130,130,104,153,89,175,151,127,122,124,131,125};// transfer to right foot contact
const unsigned char Pose_M04_5[12]={120,123,108,155,89,179,151,130,132,132,131,122};// all foot contact//COM transfer to right foot

const unsigned char Pose_M04_6[12]={115,119,108,163,89,143,151,94,130,137,129,120};// right standing

const unsigned char Pose_M04_7[12]={122,121,104,150,89,166,146,109,131,137,129,122};// left step to stand position

const unsigned char Pose_M04_8[12]={126,127,102,150,89,166,146,109,126,128,129,125};
const unsigned char Pose_M04_9[12]={125,125,104,150,88,166,145,109,126,128,129,125};
unsigned char pose_M04=0;
const unsigned int walking1Step_averageTorq[12]={4000,4000,4000,4000,4000,4000,4000,4000,4000,4000,4000,4000};
const unsigned int numOfFrames_M04[10]={22,22,18,10,20,22,20,12,24};///x1
//const unsigned int numOfFrames_M04[10]={44,44,36,20,40,44,40,24,48};//x0.5
//const unsigned int numOfFrames_M04[10]={88,88,72,40,80,88,80,48,96};//x0.25
void mapSoftPose8ToPose12(const unsigned char *pose8,unsigned int *pose12,unsigned char size){
    for(unsigned char i=0;i<size;i++)
    {
        *(pose12+i)= (((double)(*(pose8+i)))*pos8toPos12_1+pos8toPos12_2+(double)offsetZeroPos12[i]);

    }
}

//void mapHardPose8ToPose12(unsigned char *pose8,unsigned int *pose12,unsigned char size){
//    for(unsigned char i=0;i<size;i++)
//    {
//        *(pose12+i)= (((double)(*(pose8+i)))*pos8toPos12_1+pos8toPos12_2);

//    }
//}

unsigned char Pose_test[12];

const int hip_pitch_offset=5;
const int hip_roll_offset=2;
int hip_pitch_angle;
int hip_roll_angle;

int knee_angle;
int ankle_pitch_angle;
int ankle_roll_angle;
//======== COM transfer ==============
void process_pose(){
    hip_pitch_angle=-knee_angle-ankle_pitch_angle;
    hip_roll_angle=-ankle_roll_angle;

    //ankle_roll
    Pose_test[0]=(int)softZeroPos[0]+ankle_roll_angle;
    Pose_test[1]=(int)softZeroPos[1]+ankle_roll_angle;
    //ankle_pitch
    Pose_test[2]=(int)softZeroPos[2]-ankle_pitch_angle;
    Pose_test[3]=(int)softZeroPos[3]+ankle_pitch_angle;
    //knee
    Pose_test[4]=(int)softZeroPos[4]-knee_angle;
    Pose_test[5]=(int)softZeroPos[5]+knee_angle;
    //hip_pitch
    Pose_test[6]=(int)softZeroPos[6]+hip_pitch_angle+hip_pitch_offset;
    Pose_test[7]=(int)softZeroPos[7]-hip_pitch_angle-hip_pitch_offset;
    //hip_roll
    Pose_test[8]=(int)softZeroPos[8]+hip_roll_angle-hip_roll_offset;
    Pose_test[9]=(int)softZeroPos[9]+hip_roll_angle+hip_roll_offset;
    //hip_yaw
    Pose_test[10]=softZeroPos[10];
    Pose_test[11]=softZeroPos[11];
}

#define ID_LEFT_FOOT 0
#define ID_RIGHT_FOOT 1
//standing on one foot


int st_ankle_roll_offset_2;
int sw_knee_angle_2;
int sw_hip_pitch_angle_2;
int sw_hip_roll_angle_2;

int st_hip_roll_offset_2;
int sw_ankle_pitch_angle_2;
//======== stand 1 leg ==============
void process_pose_2(unsigned char foot_ID){
    sw_ankle_pitch_angle_2=-sw_knee_angle_2-sw_hip_pitch_angle_2;
    st_hip_roll_offset_2=-st_ankle_roll_offset_2;
    if(foot_ID==ID_LEFT_FOOT){
        //============ankle_roll============
        Pose_test[0]=(int)softZeroPos[0]+ankle_roll_angle+st_ankle_roll_offset_2;
        //        Pose_test[1]=(int)softZeroPos[1]+ankle_roll_angle;
        //============hip_roll==============
        Pose_test[8]=(int)softZeroPos[8]+hip_roll_angle+st_hip_roll_offset_2;
        //        Pose_test[9]=(int)softZeroPos[9]+hip_roll_angle;

        //==========ankle_pitch=============
        //        Pose_test[2]=(int)softZeroPos[2]-ankle_pitch_angle;
        Pose_test[3]=(int)softZeroPos[3]+sw_ankle_pitch_angle_2;
        //==========hip_pitch===============
        //        Pose_test[6]=(int)softZeroPos[6]+hip_pitch_angle;
        Pose_test[7]=(int)softZeroPos[7]-sw_hip_pitch_angle_2;

        //==========knee====================
        //        Pose_test[4]=(int)softZeroPos[4]-knee_angle;
        Pose_test[5]=(int)softZeroPos[5]+sw_knee_angle_2;



        //hip_yaw
        //        Pose_test[10]=softZeroPos[10];
        //        Pose_test[11]=softZeroPos[11];
    }
    else{
        //ankle_roll
        //        Pose_test[0]=(int)softZeroPos[0]+ankle_roll_angle;
        Pose_test[1]=(int)softZeroPos[1]-ankle_roll_angle+st_ankle_roll_offset_2;
        //ankle_pitch
        Pose_test[2]=(int)softZeroPos[2]-sw_ankle_pitch_angle_2;
        //        Pose_test[3]=(int)softZeroPos[3]+ankle_pitch_angle;
        //knee
        Pose_test[4]=(int)softZeroPos[4]-sw_knee_angle_2;
        //        Pose_test[5]=(int)softZeroPos[5]+knee_angle;
        //hip_pitch
        Pose_test[6]=(int)softZeroPos[6]+sw_hip_pitch_angle_2;
        //        Pose_test[7]=(int)softZeroPos[7]-hip_pitch_angle-hip_pitch_offset;
        //hip_roll
        //        Pose_test[8]=(int)softZeroPos[8]+hip_roll_angle;
        Pose_test[9]=(int)softZeroPos[9]-hip_roll_angle+st_hip_roll_offset_2;
        //hip_yaw
        //        Pose_test[10]=softZeroPos[10];
        //        Pose_test[11]=softZeroPos[11];


    }
}

int forward_pitch_3;
int st_knee_3;
int sw_knee_3;
int sw_hip_pitch_3;

int st_ankle_pitch_3;
int st_hip_pitch_3;
int sw_ankle_pitch_3;
//======== step forward ==============
void process_pose_3(unsigned char foot_ID){
    st_ankle_pitch_3=ankle_pitch_angle+forward_pitch_3;
    sw_ankle_pitch_3=-sw_knee_3-sw_hip_pitch_3;
    st_hip_pitch_3=-st_knee_3-st_ankle_pitch_3;
    if(foot_ID==ID_LEFT_FOOT){
        //============ankle_roll============
        Pose_test[0]=(int)softZeroPos[0]+ankle_roll_angle;
        //        Pose_test[1]=(int)softZeroPos[1]+ankle_roll_angle;
        //============hip_roll==============
        Pose_test[8]=(int)softZeroPos[8]+hip_roll_angle+st_hip_roll_offset_2;
        //        Pose_test[9]=(int)softZeroPos[9]+hip_roll_angle;

        //==========ankle_pitch=============
        Pose_test[2]=(int)softZeroPos[2]-st_ankle_pitch_3;
        Pose_test[3]=(int)softZeroPos[3]+sw_ankle_pitch_3;
        //==========hip_pitch===============
        Pose_test[6]=(int)softZeroPos[6]+st_hip_pitch_3;
        Pose_test[7]=(int)softZeroPos[7]-sw_hip_pitch_3;

        //==========knee====================
        Pose_test[4]=(int)softZeroPos[4]-st_knee_3;
        Pose_test[5]=(int)softZeroPos[5]+sw_knee_3;



        //hip_yaw
        //        Pose_test[10]=softZeroPos[10];
        //        Pose_test[11]=softZeroPos[11];
    }
    else{
        //============ankle_roll============
//        Pose_test[0]=(int)softZeroPos[0]+ankle_roll_angle;
                Pose_test[1]=(int)softZeroPos[1]-ankle_roll_angle;
        //============hip_roll==============
//        Pose_test[8]=(int)softZeroPos[8]+hip_roll_angle+st_hip_roll_offset_2;
                Pose_test[9]=(int)softZeroPos[9]-hip_roll_angle+st_hip_roll_offset_2;
        //==========ankle_pitch=============
        Pose_test[2]=(int)softZeroPos[2]-sw_ankle_pitch_3;
        Pose_test[3]=(int)softZeroPos[3]+st_ankle_pitch_3;
        //==========hip_pitch===============
        Pose_test[6]=(int)softZeroPos[6]+sw_hip_pitch_3;
        Pose_test[7]=(int)softZeroPos[7]-st_hip_pitch_3;

        //==========knee====================
        Pose_test[4]=(int)softZeroPos[4]-sw_knee_3;
        Pose_test[5]=(int)softZeroPos[5]+st_knee_3;



        //hip_yaw
        //        Pose_test[10]=softZeroPos[10];
        //        Pose_test[11]=softZeroPos[11];
    }
}



int ankle_roll_4;
int st_ankle_pitch_4;
int sw_ankle_pitch_4;
int st_knee_4;
int sw_knee_4;

int hip_roll_4;
int st_hip_pitch_4;
int sw_hip_pitch_4;
//======== COM transfer ==============
void process_pose_4(){
    st_hip_pitch_4=-st_knee_4-st_ankle_pitch_4;
    sw_hip_pitch_4=-sw_knee_4-sw_ankle_pitch_4;
    hip_roll_4=-ankle_roll_4;

    //ankle_roll
    Pose_test[0]=(int)softZeroPos[0]+ankle_roll_4;
    Pose_test[1]=(int)softZeroPos[1]+ankle_roll_4;
    //ankle_pitch
    Pose_test[2]=(int)softZeroPos[2]-st_ankle_pitch_4;
    Pose_test[3]=(int)softZeroPos[3]+sw_ankle_pitch_4;
    //knee
    Pose_test[4]=(int)softZeroPos[4]-st_knee_4;
    Pose_test[5]=(int)softZeroPos[5]+sw_knee_4;
    //hip_pitch
    Pose_test[6]=(int)softZeroPos[6]+st_hip_pitch_4;
    Pose_test[7]=(int)softZeroPos[7]-sw_hip_pitch_4;

    Pose_test[8]=(int)softZeroPos[8]+hip_roll_4;
    Pose_test[9]=(int)softZeroPos[9]+hip_roll_4;
    //hip_yaw
    //    Pose_test[10]=softZeroPos[10];
    //    Pose_test[11]=softZeroPos[11];
}

/*
 * ============== sam class============
 */

#define SAM_UPPER_12BIT_BOUNDARY_  3700
#define SAM_LOWER_12BIT_BOUNDARY_ 401
class SAMmodule{

public:
    //    unsigned char enableSam[24];
    void setSamPos12(unsigned char ID,unsigned int Pos);
    void getSamPos12(unsigned char ID);

    void setSamAverageTorq(unsigned char ID,unsigned int ATorq);
    void getSamAverageTorq(unsigned char ID);

    void setSamPos8(unsigned char ID,unsigned char Pos,unsigned char Mode);
    void getSamPos8(unsigned char ID);
    void setPassive(unsigned char ID);

    void getPID(unsigned char ID);
    void setPID(unsigned char ID,unsigned char Pvalue,unsigned char Ivalue,unsigned char Dvalue);

    void setPDQuick(unsigned char ID,unsigned char Pvalue,unsigned char Dvalue);

    void getAllPos12();
    void getAllPos8Torq8();
    void setAllPassive();
    void SAM_Power_enable(unsigned char state);
    void setAllPos12(unsigned int *Pos,unsigned char numOfSam);
    void setAllAverageTorque(const unsigned int *Atorq,unsigned char numOfSam);
    void getAllAverageTorque();

    void setAllPDQuick(const unsigned char *Pvalue,const unsigned char *Dvalue,unsigned char numOfSam);
    void getAllPDQuick();
};
SAMmodule mySam;


double angle[24];
double delta_angle[24];
double pos12_double[24];
unsigned int pos12[24];
double delta_pos12[24];
unsigned int currentSamPos12[24];


int main(int argc, char **argv)
{

    ros::init(argc, argv, "uxa_serial");
    ros::NodeHandle n;
    ros::Publisher myMessage = n.advertise<sensor_msgs::JointState>("UXAJointState", 1000);

    sensor_msgs::JointState joint_state;
    joint_state.name.resize(12);
    joint_state.position.resize(12);
    joint_state.name[0] ="Joint0";
    joint_state.name[1] ="Joint1";
    joint_state.name[2] ="Joint2";
    joint_state.name[3] ="Joint3";
    joint_state.name[4] ="Joint4";
    joint_state.name[5] ="Joint5";
    joint_state.name[6] ="Joint6";
    joint_state.name[7] ="Joint7";
    joint_state.name[8] ="Joint8";
    joint_state.name[9] ="Joint9";
    joint_state.name[10] ="Joint10";
    joint_state.name[11] ="Joint11";

    //    ros::Publisher uxa_serial_pub = n.advertise<uxa_serial_msgs::receive>("uxa_serial_publisher", _MSG_BUFF_SIZE);
    //    ros::Subscriber uxa_serial_sub = n.subscribe<uxa_serial_msgs::transmit>("uxa_serial_subscriber", _MSG_BUFF_SIZE, rev_func);

    ros::Rate loop_rate(LOOP_RATE_1000Hz_);

    //    if((Serial = Init_Serial(_SERIAL_PORT)) != -1)
    //    {

    memset(Trans_chr, '\0', sizeof(Trans_chr));
    memset(Recev_chr, '\0', sizeof(Recev_chr));

    //        unsigned char cnt = 0;
    //        mySam.setSamPos12(0,1700);

    //        sleep(1);
    //        mySam.setSamPos12(0,1500);

    //        sleep(1);
    //        mySam.setSamPos12(0,1800);

    //        sleep(1);

    cout << "SERIAL : " <<  "Serial communication stand by." << endl << endl;


    unsigned int taskCount=0;
    unsigned char flagTask=0;

    unsigned int samPos12;
    unsigned char samID;




    usleep(500);
    for(unsigned char i=0; i<12;i++)
    {
        mySam.setPID(i,default_samP[i],default_samI[i],default_samD[i]);
        usleep(500);
    }

    mapSoftPose8ToPose12((unsigned char*)softZeroPos,(unsigned int*)pos12,12);
    //        mySam.setAllPos12(pos12,12);

    task.id=TASK_POSES_TEST_;
    while(ros::ok())
    {
        loop_rate.sleep();
        Timer_handler();
        /*
             * ========== your code begins frome here===============
             */

        if(FlagTimer.Hz_50)
        {
            FlagTimer.Hz_50=0;
            //================
            mySam.getAllPos12();
            //                for (unsigned char i=0;i<12;i++)
            //                {
            //                    cout <<angle[i]<<":";
            //                }
            //                cout<<endl;

        }


        if(FlagTimer.Hz_100)
        {
            FlagTimer.Hz_100=0;
            //================

            switch (task.id){
            case TASK_POSES_TEST_:
                if(task.startFlag==0){
                    task.startFlag=1;
                    task.scene=0;
                    unsigned int beginPose12[12];
                    unsigned int endPose12[12];

                    knee_angle =-40;//40 (8bit angle/1.08degree)
                    ankle_pitch_angle=25;
                    ankle_roll_angle=-8;
                    process_pose();
                    mapSoftPose8ToPose12(Pose_init_walking,beginPose12,12);

                    mapSoftPose8ToPose12(Pose_test,endPose12,12);
                    myscene.setUpMyScene(300,beginPose12,endPose12);
                }
                else if(myscene.flag.finish)
                {
                    myscene.flag.finish=0;
                    task.scene++;

                    unsigned int beginPose12[12];
                    unsigned int endPose12[12];
                    switch(task.scene){
                    case 0:
                        knee_angle =-40;//40 (8bit angle/1.08degree)
                        ankle_pitch_angle=25;
                        ankle_roll_angle=10;
                        process_pose();
                        mapSoftPose8ToPose12(Pose_test,beginPose12,12);

                        knee_angle =-40;//40 (8bit angle/1.08degree)
                        ankle_pitch_angle=25;
                        ankle_roll_angle=-10;
                        process_pose();
                        mapSoftPose8ToPose12(Pose_test,endPose12,12);
                        myscene.setUpMyScene(100,beginPose12,endPose12);

                        break;
                    case 1:


                        knee_angle =-40;//40 (8bit angle/1.08degree)
                        ankle_pitch_angle=25;
                        ankle_roll_angle=-8;
                        process_pose();
                        mapSoftPose8ToPose12(Pose_test,beginPose12,12);

                        sw_knee_angle_2=-60;
                        sw_hip_pitch_angle_2=40;
                        st_ankle_roll_offset_2=0;
                        process_pose_2(ID_LEFT_FOOT);
                        mapSoftPose8ToPose12(Pose_test,endPose12,12);
                        myscene.setUpMyScene(50,beginPose12,endPose12);
                        break;
                    case 2:
                        //                            myscene.setDelayScene(300);

                        process_pose_2(ID_LEFT_FOOT);
                        mapSoftPose8ToPose12(Pose_test,beginPose12,12);

                        forward_pitch_3=7;
                        st_knee_3=-30;
                        sw_knee_3=-40;
                        sw_hip_pitch_3=25;

                        process_pose_3(ID_LEFT_FOOT);
                        mapSoftPose8ToPose12(Pose_test,endPose12,12);
                        myscene.setUpMyScene(50,beginPose12,endPose12);
                        break;
                    case 3:
                        mapSoftPose8ToPose12(Pose_test,beginPose12,12);

                        ankle_roll_4=8;
                        st_ankle_pitch_4=32;
                        sw_ankle_pitch_4=25;
                        st_knee_4=-25;
                        sw_knee_4=-40;
                        process_pose_4();

                        mapSoftPose8ToPose12(Pose_test,endPose12,12);
                        myscene.setUpMyScene(300,beginPose12,endPose12);

                        break;
                    case 4:
                        mapSoftPose8ToPose12(Pose_test,beginPose12,12);

                        sw_knee_angle_2=-60;
                        sw_hip_pitch_angle_2=15;
                        st_ankle_roll_offset_2=0;
                        process_pose_2(ID_RIGHT_FOOT);
                        mapSoftPose8ToPose12(Pose_test,endPose12,12);
                        myscene.setUpMyScene(300,beginPose12,endPose12);

                        break;

                    case 5:
                        process_pose_2(ID_RIGHT_FOOT);
                        mapSoftPose8ToPose12(Pose_test,beginPose12,12);

                        forward_pitch_3=0;
                        st_knee_3=-40;
                        sw_knee_3=-40;
                        sw_hip_pitch_3=20;

                        process_pose_3(ID_RIGHT_FOOT);
                        mapSoftPose8ToPose12(Pose_test,endPose12,12);
                        myscene.setUpMyScene(100,beginPose12,endPose12);
                    default:
                        task.finishFlag=1;
                        break;
                    }
                    //                task.finishFlag=1;
                }
                else if(task.finishFlag){
                    task.finishFlag=0;
                    task.startFlag=0;
                    task.id=TASK_INIT_;

                }
                break;
            case TASK_INIT_:
                if(task.startFlag==0){
                    task.startFlag=1;
                    //===============
                    //                        myscene.setUpMyScene(200,pose_init,softZeroPos);
                    //                        mySam.getAllPos12();
                }
                else if(myCom.flagDataAvailable_readAllPos12)
                {
                    myCom.flagDataAvailable_readAllPos12=0;
                    //====================================
                    unsigned char count=0;
                    for(unsigned char i=0;i<12;i++){
                        if(myCom.samPos12Avail[i]==1)
                            count++;
                    }

                    if((count==12)&&(myCom.samPos12[0]>0))
                    {
                        task.finishFlag=1;
                        cout <<"init task done"<<endl;
                        for(unsigned char i=0;i<12;i++)
                        {
                            currentSamPos12[i]=myCom.samPos12[i];
                        }
                        for (unsigned char i=0;i<12;i++)
                        {
                            cout <<currentSamPos12[i]<<":";
                        }
                        cout<<endl;

                        for(unsigned char i=0;i<12;i++)
                        {
                            pos12[i]=currentSamPos12[i];
                            angle[i]=((double)pos12[i]-(double)samPos12_offset[i])*pos12bitTorad;

                        }


                    }
                    else{
                        sleep(1);
                        cout <<"init error: "<<(int)count<<endl;
                        mySam.getAllPos12();
                    }



                }
                else if(task.finishFlag){
                    task.finishFlag=0;
                    task.startFlag=0;
                    task.id=TASK_TEST_;
                    cout <<"finish test task "<<endl;
                    mySam.setAllAverageTorque(sitdown_averageTorq,12);
                    usleep(1000);
                    mySam.setAllPDQuick(sitdown_samP,sitdown_samD,12);
                    usleep(1000);
                }
                break;

            case TASK_TEST_:
                if(task.startFlag==0){
                    task.startFlag=1;
                    unsigned int endPose12[12];
                    mapSoftPose8ToPose12(Pose_init_walking,endPose12,12);
                    //                        mapSoftPose8ToPose12(Pose_M04_4,endPose12,12);
                    myscene.setUpMyScene(300,currentSamPos12,endPose12);
                }

                else if(task.finishFlag){
                    task.finishFlag=0;
                    task.startFlag=0;
                    task.id=TASK_POSES_TEST_;
                }
                else if(myscene.flag.finish)
                {
                    task.finishFlag=1;
                    mySam.setAllAverageTorque(walking1Step_averageTorq,12);
                    usleep(1000);

                }
                break;

            case TASK_STANDING_:
                if(task.startFlag==0){
                    task.startFlag=1;
                    task.scene=0;
                    unsigned int beginPose12[12];
                    unsigned int endPose12[12];
                    mapSoftPose8ToPose12(Pose_standing_0,beginPose12,12);
                    mapSoftPose8ToPose12(Pose_standing_1,endPose12,12);
                    myscene.setUpMyScene(numOfFrames_standing[task.scene],beginPose12,endPose12);
                }
                else if(myscene.flag.finish)
                {
                    myscene.flag.finish=0;
                    task.scene++;

                    unsigned int beginPose12[12];
                    unsigned int endPose12[12];
                    switch(task.scene){
                    case 0:
                        mapSoftPose8ToPose12(Pose_standing_0,beginPose12,12);
                        mapSoftPose8ToPose12(Pose_standing_1,endPose12,12);
                        myscene.setUpMyScene(numOfFrames_standing[task.scene],beginPose12,endPose12);
                        break;
                    case 1:
                        mapSoftPose8ToPose12(Pose_standing_1,beginPose12,12);
                        mapSoftPose8ToPose12(Pose_standing_2,endPose12,12);
                        myscene.setUpMyScene(numOfFrames_standing[task.scene],beginPose12,endPose12);
                        break;
                    default:
                        task.finishFlag=1;
                        break;
                    }
                    //                task.finishFlag=1;
                }
                else if(task.finishFlag){
                    task.finishFlag=0;
                    task.startFlag=0;
                    task.id=TASK_INIT_WALKING_;
                    sleep(3);
                }
                break;


            case  TASK_SITDOWN_:
                if(task.startFlag==0){
                    task.startFlag=1;
                    task.scene=1;
                    unsigned int beginPose12[12];
                    unsigned int endPose12[12];
                    mapSoftPose8ToPose12(Pose_standing_2,beginPose12,12);
                    mapSoftPose8ToPose12(Pose_standing_1,endPose12,12);
                    myscene.setUpMyScene(numOfFrames_standing[task.scene],beginPose12,endPose12);
                }
                else if(myscene.flag.finish)
                {
                    myscene.flag.finish=0;
                    task.scene--;

                    unsigned int beginPose12[12];
                    unsigned int endPose12[12];
                    switch(task.scene){
                    case 0:
                        mapSoftPose8ToPose12(Pose_standing_1,beginPose12,12);
                        mapSoftPose8ToPose12(Pose_standing_0,endPose12,12);
                        myscene.setUpMyScene(numOfFrames_standing[task.scene],beginPose12,endPose12);
                        break;
                    case 1:
                        mapSoftPose8ToPose12(Pose_standing_2,beginPose12,12);
                        mapSoftPose8ToPose12(Pose_standing_1,endPose12,12);
                        myscene.setUpMyScene(numOfFrames_standing[task.scene],beginPose12,endPose12);
                        break;
                    default:
                        task.finishFlag=1;
                        break;
                    }
                    //                task.finishFlag=1;
                }
                else if(task.finishFlag){
                    task.finishFlag=0;
                    task.startFlag=0;
                    task.id=TASK_STANDING_;
                    sleep(4);

                    //                        mySam.setAllPassive();
                    //                        sleep(1);
                }
                break;
            case TASK_INIT_WALKING_:
                if(task.startFlag==0){
                    task.startFlag=1;
                    unsigned int beginPose12[12];
                    unsigned int endPose12[12];
                    mapSoftPose8ToPose12(Pose_standing_2,beginPose12,12);
                    mapSoftPose8ToPose12(Pose_init_walking,endPose12,12);
                    myscene.setUpMyScene(300,beginPose12,endPose12);
                }
                else if(myscene.flag.finish)
                {
                    myscene.flag.finish=0;
                    task.finishFlag=1;
                }
                else if(task.finishFlag){
                    task.finishFlag=0;
                    task.startFlag=0;
                    task.id=TASK_WALK_1STEP_;
                }
                break;
            case TASK_WALK_1STEP_:
                if(task.startFlag==0){
                    task.startFlag=1;
                    task.scene=0;
                    //===== you code begin from here====
                    unsigned int beginPose12[12];
                    unsigned int endPose12[12];
                    mapSoftPose8ToPose12(Pose_M04_0,beginPose12,12);
                    mapSoftPose8ToPose12(Pose_M04_1,endPose12,12);
                    myscene.setUpMyScene(numOfFrames_M04[task.scene],beginPose12,endPose12);
                }
                else if(myscene.flag.finish)
                {
                    myscene.flag.finish=0;
                    task.scene++;

                    unsigned int beginPose12[12];
                    unsigned int endPose12[12];
                    switch(task.scene){
                    case 0:
                        mapSoftPose8ToPose12(Pose_M04_0,beginPose12,12);
                        mapSoftPose8ToPose12(Pose_M04_1,endPose12,12);
                        myscene.setUpMyScene(numOfFrames_M04[task.scene],beginPose12,endPose12);
                        break;
                    case 1:
                        mapSoftPose8ToPose12(Pose_M04_1,beginPose12,12);
                        mapSoftPose8ToPose12(Pose_M04_2,endPose12,12);
                        myscene.setUpMyScene(numOfFrames_M04[task.scene],beginPose12,endPose12);

                        break;
                    case 2:
                        mapSoftPose8ToPose12(Pose_M04_2,beginPose12,12);
                        mapSoftPose8ToPose12(Pose_M04_3,endPose12,12);
                        myscene.setUpMyScene(numOfFrames_M04[task.scene],beginPose12,endPose12);


                        break;
                    case 3:
                        mapSoftPose8ToPose12(Pose_M04_3,beginPose12,12);
                        mapSoftPose8ToPose12(Pose_M04_4,endPose12,12);
                        myscene.setUpMyScene(numOfFrames_M04[task.scene],beginPose12,endPose12);

                        break;
                    case 4:
                        mapSoftPose8ToPose12(Pose_M04_4,beginPose12,12);
                        mapSoftPose8ToPose12(Pose_M04_5,endPose12,12);
                        myscene.setUpMyScene(numOfFrames_M04[task.scene],beginPose12,endPose12);
                        break;
                    case 5:
                        mapSoftPose8ToPose12(Pose_M04_5,beginPose12,12);
                        mapSoftPose8ToPose12(Pose_M04_6,endPose12,12);
                        myscene.setUpMyScene(numOfFrames_M04[task.scene],beginPose12,endPose12);

                        break;

                    case 6:
                        mapSoftPose8ToPose12(Pose_M04_6,beginPose12,12);
                        mapSoftPose8ToPose12(Pose_M04_7,endPose12,12);
                        myscene.setUpMyScene(numOfFrames_M04[task.scene],beginPose12,endPose12);
                        //task.scene=9;
                        break;
                    case 7:
                        mapSoftPose8ToPose12(Pose_M04_7,beginPose12,12);
                        mapSoftPose8ToPose12(Pose_M04_8,endPose12,12);
                        myscene.setUpMyScene(numOfFrames_M04[task.scene],beginPose12,endPose12);

                        break;
                    case 8:
                        mapSoftPose8ToPose12(Pose_M04_8,beginPose12,12);
                        mapSoftPose8ToPose12(Pose_M04_9,endPose12,12);
                        myscene.setUpMyScene(numOfFrames_M04[task.scene],beginPose12,endPose12);

                        break;
                    default:
                        task.finishFlag=1;
                        break;
                    }
                    //                task.finishFlag=1;
                }
                else if(task.finishFlag){
                    task.finishFlag=0;
                    task.startFlag=0;
                    //==========your code==========
                    task.id=TASK_WALK_1STEP_;
                }
                break;
            default:
                break;
            }

            if(myscene.flag.enable){
                myscene.frame++;

                if(myscene.flag.delay){
                    if(myscene.flag.start==0){
                        myscene.flag.start=1;
                        myscene.frame=0;
                    }
                    else if(myscene.frame>myscene.numOfFrame)
                    {
                        myscene.flag.enable=0;
                        myscene.flag.finish=1;
                        myscene.flag.delay=0;
                    }
                }
                else{
                    if(myscene.flag.start==0){
                        myscene.flag.start=1;
                        myscene.frame=0;
                        for(unsigned char i=0;i<12;i++)
                        {
                            pos12_double[i]=*(myscene.beginPose+i);
                            angle[i]=(pos12_double[i]-(double)samPos12_offset[i])*pos12bitTorad;
                        }
                        for(unsigned char i=0;i<12;i++)
                        {
                            unsigned int b=*(myscene.beginPose+i);
                            unsigned int a=*(myscene.endPose+i);
                            delta_pos12[i]=((double)a-(double)b)/(double)myscene.numOfFrame;
                            delta_angle[i]=delta_pos12[i]*pos12bitTorad;

                        }
                    }
                    else if(myscene.frame<myscene.numOfFrame)
                    {
                        for(unsigned char i=0;i<12;i++)
                        {
                            angle[i]+=delta_angle[i];
                            pos12_double[i]+=delta_pos12[i];
                        }
                    }
                    else{
                        myscene.flag.enable=0;
                        myscene.flag.finish=1;
                        for(unsigned char i=0;i<12;i++)
                        {
                            pos12_double[i]=*(myscene.endPose+i);
                            angle[i]=(pos12_double[i]-(double)samPos12_offset[i])*pos12bitTorad;

                        }
                    }

                    //                    for (unsigned char i=0;i<12;i++)
                    //                    {
                    //                        cout <<(unsigned int)pos12[i]<<":";
                    //                    }
                    //                    cout<<endl;


                    for (unsigned char i=0;i<12;i++){
                        //                        if((pos12_double[i]>=SAM_LOWER_12BIT_BOUNDARY_)&&(pos12_double[i]<SAM_UPPER_12BIT_BOUNDARY_))
                        //                        {

                        //                        }
                        pos12[i]=(unsigned int)pos12_double[i];
                    }
#ifdef ENABLE_ACTUATOR_
                    mySam.setAllPos12(pos12,12);
#endif
                }
            }

            joint_state.header.stamp = ros::Time::now();
            joint_state.position[0] = angle[0];
            joint_state.position[1] = angle[1];
            joint_state.position[2] = -angle[2];
            joint_state.position[3] = angle[3];
            joint_state.position[4] = -angle[4];
            joint_state.position[5] = angle[5];
            joint_state.position[6] = angle[6];
            joint_state.position[7] = -angle[7];
            joint_state.position[8] = angle[8];
            joint_state.position[9] = angle[9];
            joint_state.position[10] = angle[10];
            joint_state.position[11] = angle[11];
            myMessage.publish(joint_state);

        }

        //================communication block=================
        Recev_Data_hanlder();
        if(myCom.flagDataReceived_readAllPos12)
        {
            myCom.flagDataReceived_readAllPos12=0;
            myCom.flagDataAvailable_readAllPos12=1;
            //=============================
            for (unsigned char i=0;i<myCom.NumofSam;i++)
            {
                cout <<dec<< myCom.samPos12[i]<<":";
            }
            cout<<endl;

            //                                for(unsigned char i=0;i<12;i++)
            //                                {
            //                                                    angle[i]=((double)myCom.samPos12[i]-(double)(samPos12_offset[i]))*pos12bitTorad;
            //                                }

            //                joint_state.header.stamp = ros::Time::now();
            //                joint_state.position[0] = angle[0];
            //                joint_state.position[1] = angle[1];
            //                joint_state.position[2] = -angle[2];
            //                joint_state.position[3] = angle[3];
            //                joint_state.position[4] = -angle[4];
            //                joint_state.position[5] = angle[5];
            //                joint_state.position[6] = -angle[6];
            //                joint_state.position[7] = -angle[7];
            //                joint_state.position[8] = angle[8];
            //                joint_state.position[9] = angle[9];
            //                joint_state.position[10] = angle[10];
            //                joint_state.position[11] = angle[11];
            //                myMessage.publish(joint_state);
        }

        //======================== end of your code======================
        ros::spinOnce();
    }

    //    }

    cout << endl;
    cout << "SERIAL : " << Serial << " Device close." << endl;
    close(Serial);
    cout << "SERIAL : " << "uxa_serial node terminate." << endl;
    return 0;
}


int Init_Serial(const char *Serial_Port)
{
    termios Serial_Setting;
    if((Serial = open(Serial_Port, O_RDWR | O_NONBLOCK | O_NOCTTY)) == -1)
    {
        cout << "SERIAL : " << Serial_Port << " Device open error" << endl;
        cout << "SERIAL : " << Serial_Port << " Device permission change progress...." << endl;
        for(int temp = 0; temp < 5; temp++)
        {
            if(chmod(Serial_Port, __S_IREAD | __S_IWRITE) == 0){

                cout << "SERIAL : " << Serial_Port << " Device permission change complete" << endl;
                Serial = open(Serial_Port, O_RDWR | O_NONBLOCK | O_NOCTTY);

                if(Serial == -1)
                {
                    cout << "SERIAL : " << Serial_Port << " Device Not Found" << endl;
                    return -1;
                }
                else
                    cout << "SERIAL : " << Serial_Port <<" Device open" << endl;
            }
            else
            {
                cout << "SERIAL : " << Serial_Port << " Device permission change error" << endl;
                //return -1;
            }
        }
    }
    else
        cout << "SERIAL : " << Serial_Port << " Device open" << endl;


    memset(&Serial_Setting, 0, sizeof(Serial_Setting));
    Serial_Setting.c_iflag = 0;
    Serial_Setting.c_oflag = 0;
    Serial_Setting.c_cflag = _BAUDRATE | CS8 | CREAD | CLOCAL;
    Serial_Setting.c_lflag = 0;
    Serial_Setting.c_cc[VMIN] = 1;
    Serial_Setting.c_cc[VTIME] = 0;

    cfsetispeed(&Serial_Setting, _BAUDRATE);
    cfsetospeed(&Serial_Setting, _BAUDRATE);

    tcflush(Serial, TCIFLUSH);
    tcsetattr(Serial, TCSANOW, &Serial_Setting);
    return Serial;
}


//int Serial_Test(int Serial, unsigned int Test_size)
//{
//    char Trans_chr[Test_size];
//    char Recei_chr[Test_size];

//    memset(Trans_chr, 0b10101010, sizeof(Trans_chr));
//    memset(Recei_chr, '\0', sizeof(Recei_chr));

//    write(Serial, Trans_chr, sizeof(Trans_chr));

//    if(read(Serial, Recei_chr, sizeof(Recei_chr)) > 0)
//    {
//        cout << "Receive" << endl;
//        if(strcmp(Trans_chr, Recei_chr) != 0)
//        {
//            close(Serial);
//            cout << endl;
//            cout << "ERR" << endl;
//            cout << "ERR Receive CHR = " << Recei_chr << endl;
//            return -1;
//        }
//        else
//        {
//            cout << endl;
//            cout << "OK" <<endl;
//            memset(Recei_chr, '\0', sizeof(Recei_chr));
//        }

//    }
//}

void Send_Serial_String(int Serial, unsigned char *Trans_chr, int Size)
{
    write(Serial, Trans_chr, Size);
}

void Send_Serial_Char(int Serial, unsigned char *Trans_chr)
{
    write(Serial, Trans_chr, 1);
}

int Read_Serial_Char(int Serial, unsigned char *Recei_chr)
{
    unsigned int dataSize=read(Serial, Recei_chr, 50);
    if( dataSize> 0)
    {
        return dataSize;
    }
    return -1;
}


//void rev_func(const uxa_serial_msgs::transmit::ConstPtr &msg)
//{
//    ROS_INFO("receive msg : 0x%x",msg->tx_data);
//    *msg_buf = msg->tx_data;
//    Send_Serial_Char(Serial, msg_buf);
//}



void SAMmodule::setSamPos12(unsigned char ID, unsigned int Pos)
{
    unsigned char Mode=8;
    unsigned char ba[6];
    ba[0] = 0xff;
    ba[1] = (((Mode&0x0C)<<3)+ID)&0x7F;
    ba[2] = (((Mode&0x03)<<5)+(Pos>>7))&0x7F;
    ba[3] = Pos&0x7F;
    ba[4] = (ba[1]^ba[2]^ba[3])&0x7F;
    ba[5] =0xfe;
    Send_Serial_String(Serial,ba,6);
}

void SAMmodule::getSamPos12(unsigned char ID)
{
    unsigned char Mode=7;
    unsigned int Pos=0;
    unsigned char ba[6];
    ba[0] = 0xff;
    ba[1] = (((Mode&0x0C)<<3)+ID)&0x7F;
    ba[2] = (((Mode&0x03)<<5)+(Pos>>7))&0x7F;
    ba[3] = Pos&0x7F;
    ba[4] = (ba[1]^ba[2]^ba[3])&0x7F;
    ba[5] =0xfe;
    Send_Serial_String(Serial,ba,6);
}

void SAMmodule::setSamAverageTorq(unsigned char ID, unsigned int ATorq)
{
    unsigned char Mode=9;
    unsigned char ba[6];
    ba[0] = 0xff;
    ba[1] = (((Mode&0x0C)<<3)+ID)&0x7F;
    ba[2] = (((Mode&0x03)<<5)+(ATorq>>7))&0x7F;
    ba[3] = ATorq&0x7F;
    ba[4] = (ba[1]^ba[2]^ba[3])&0x7F;
    ba[5] =0xfe;
    Send_Serial_String(Serial,ba,6);
}

void SAMmodule::getSamAverageTorq(unsigned char ID)
{
    unsigned char Mode=10;
    unsigned char ba[6];
    ba[0] = 0xff;
    ba[1] = (((Mode&0x0C)<<3)+ID)&0x7F;
    ba[2] = ((Mode&0x03)<<5);
    ba[3] = 0;
    ba[4] = (ba[1]^ba[2]^ba[3])&0x7F;
    ba[5] =0xfe;
    Send_Serial_String(Serial,ba,6);
}

void SAMmodule::setSamPos8(unsigned char ID, unsigned char Pos,unsigned char Mode)
{
    if (Mode>4)
        Mode=4;
    unsigned char ba[6];
    ba[0] = 0xff;
    ba[1] = (((Mode&0x0C)<<3)+ID)&0x7F;
    ba[2] = (((Mode&0x03)<<5)+((Pos>>7)&0x01))&0x7F;
    ba[3] = Pos&0x7F;
    ba[4] = (ba[1]^ba[2]^ba[3])&0x7F;
    ba[5] =0xfe;
    Send_Serial_String(Serial,ba,6);
}

void SAMmodule::getSamPos8(unsigned char ID)
{

    unsigned char  Mode=5;
    unsigned char Pos=0;
    unsigned char ba[6];
    ba[0] = 0xff;
    ba[1] = (((Mode&0x0C)<<3)+ID)&0x7F;
    ba[2] = (((Mode&0x03)<<5)+((Pos>>7)&0x01))&0x7F;
    ba[3] = Pos&0x7F;
    ba[4] = (ba[1]^ba[2]^ba[3])&0x7F;
    ba[5] =0xfe;
    Send_Serial_String(Serial,ba,6);
}

void SAMmodule::setPassive(unsigned char ID)
{
    unsigned char Mode=6;
    unsigned int Pos=0;
    unsigned char ba[6];
    ba[0] = 0xff;
    ba[1] = (((Mode&0x0C)<<3)+ID)&0x7F;
    ba[2] = (((Mode&0x03)<<5)+(Pos>>7))&0x7F;
    ba[3] = Pos&0x7F;
    ba[4] = (ba[1]^ba[2]^ba[3])&0x7F;
    ba[5] =0xfe;
    Send_Serial_String(Serial,ba,6);
}

void SAMmodule::getPID(unsigned char ID)
{
    unsigned char ba[4];
    ba[0] = 0xff;
    ba[1] = 0x95;
    ba[2] = ID&0x1F;
    ba[3] =0xfe;
    Send_Serial_String(Serial,ba,4);
}

void SAMmodule::setPID(unsigned char ID, unsigned char Pvalue, unsigned char Ivalue, unsigned char Dvalue)
{
    unsigned char ba[9];
    ba[0] = 0xff;
    ba[1]=0xaa;
    ba[2] = ID&0x1F;
    ba[3] = ((Pvalue&0x80)>>5)+((Ivalue&0x80)>>6)+((Dvalue&0x80)>>7);
    ba[4] = Pvalue&0x7F;
    ba[5] = Ivalue&0x7F;
    ba[6] = Dvalue&0x7F;
    ba[7] = (ba[2]^ba[3]^ba[4]^ba[5]^ba[6])&0x7F;
    ba[8] =0xfe;
    Send_Serial_String(Serial,ba,9);
}

void SAMmodule::setPDQuick(unsigned char ID, unsigned char Pvalue, unsigned char Dvalue)
{
    unsigned char ba[7];
    ba[0] = 0xff;
    ba[1]=0xbb;
    ba[2] = (ID&0x1F)+((Pvalue&0x80)>>1)+((Dvalue&0x80)>>2);
    ba[3] = Pvalue&0x7F;
    ba[4] = Dvalue&0x7F;
    ba[5] = (ba[2]^ba[3]^ba[4])&0x7F;
    ba[6] =0xfe;
    Send_Serial_String(Serial,ba,7);
}

void SAMmodule::getAllPos12()
{
    unsigned char ba[3];
    ba[0] = 0xff;
    ba[1] = 0xcc;
    ba[2] =0xfe;
    Send_Serial_String(Serial,ba,3);
}

void SAMmodule::getAllPos8Torq8()
{

    unsigned char ba[3];
    ba[0] = 0xff;
    ba[1] = 0xec;
    ba[2] =0xfe;
    Send_Serial_String(Serial,ba,3);
}

void SAMmodule::setAllPassive()
{
    unsigned char ba[3];
    ba[0] = 0xff;
    ba[1] = 0x88;
    ba[2] =0xfe;
    Send_Serial_String(Serial,ba,3);
}

void SAMmodule::SAM_Power_enable(unsigned char state)
{
    if(state)
        state=1;
    else
        state=0;
    unsigned char ba[4];
    ba[0] = 0xff;
    ba[1] = 0x81;
    ba[2] = state;
    ba[3] =0xfe;
    Send_Serial_String(Serial,ba,4);
}

void SAMmodule::setAllPos12(unsigned int *Pos, unsigned char numOfSam)
{
    unsigned char ba[numOfSam*4+3];
    ba[0] = 0xff;
    ba[1] = 0xf0;

    unsigned char refIndex=2;
    for(unsigned char i=0; i<numOfSam;i++)
    {
        if((*(Pos+i)>400)&&(*(Pos+i)<3701))
        {
            ba[refIndex++]=i;//id
            ba[refIndex++]=(*(Pos+i)>>7)&0x7F;
            ba[refIndex++]=*(Pos+i)&0x7F;
            ba[refIndex]=(ba[refIndex-3]^ba[refIndex-2]^ba[refIndex-1])&0x7F;
            refIndex++;
            cout <<(unsigned int)*(Pos+i)<<":";
        }
    }
    ba[refIndex] =0xfe;
    cout<<endl;
    Send_Serial_String(Serial,ba,numOfSam*4+3);
}

void SAMmodule::setAllAverageTorque(const unsigned int *Atorq, unsigned char numOfSam)
{
    unsigned char ba[numOfSam*4+3];
    ba[0] = 0xff;
    ba[1] = 0xbd;

    unsigned char refIndex=2;
    for(unsigned char i=0; i<numOfSam;i++)
    {
        if(*(Atorq+i)<4001)
        {
            ba[refIndex++]=i;//id
            ba[refIndex++]=(*(Atorq+i)>>7)&0x7F;
            ba[refIndex++]=*(Atorq+i)&0x7F;
            ba[refIndex]=(ba[refIndex-3]^ba[refIndex-2]^ba[refIndex-1])&0x7F;
            refIndex++;
        }
    }
    ba[refIndex] =0xfe;
    Send_Serial_String(Serial,ba,numOfSam*4+3);
}

void SAMmodule::getAllAverageTorque()
{
    unsigned char ba[3];
    ba[0] = 0xff;
    ba[1] = 0xbf;
    ba[2] =0xfe;
    Send_Serial_String(Serial,ba,3);
}

void SAMmodule::setAllPDQuick(const unsigned char *Pvalue, const unsigned char *Dvalue, unsigned char numOfSam)
{
    unsigned char ba[numOfSam*4+3];
    ba[0] = 0xff;
    ba[1] = 0xc1;

    unsigned char refIndex=2;
    for(unsigned char i=0; i<numOfSam;i++)
    {

        ba[refIndex++]=(i&0x1F)+(((*(Pvalue+i))&0x80)>>1)+(((*(Dvalue+i))&0x80)>>2);//id
        ba[refIndex++]=(*(Pvalue+i))&0x7F;
        ba[refIndex++]=(*(Dvalue+i))&0x7F;
        ba[refIndex]=(ba[refIndex-3]^ba[refIndex-2]^ba[refIndex-1])&0x7F;
        refIndex++;
    }
    ba[refIndex] =0xfe;
    Send_Serial_String(Serial,ba,numOfSam*4+3);
}

void SAMmodule::getAllPDQuick()
{
    unsigned char ba[3];
    ba[0] = 0xff;
    ba[1] = 0xc3;
    ba[2] =0xfe;
    Send_Serial_String(Serial,ba,3);
}


void Scene_class::setBeginPose(const unsigned int *value)
{
    beginPose = (unsigned int*)value;
}

void Scene_class::setEndPose(const unsigned int *value)
{
    endPose = (unsigned int*)value;
}

void Scene_class::setNumOfFrame(unsigned int value)
{
    numOfFrame = value;
}

void Scene_class::setDelayScene(unsigned int fr)
{
    this->setNumOfFrame(fr);
    this->frame=0;
    this->flag.start=0;
    this->flag.finish=0;
    this->flag.enable=1;
    this->flag.delay=1;
}

void Scene_class::setUpMyScene(unsigned int fr, const unsigned int *beginpose, const unsigned int *endpose)
{
    this->setNumOfFrame(fr);
    this->setBeginPose(beginpose);
    this->setEndPose(endpose);
    this->frame=0;
    this->flag.start=0;
    this->flag.finish=0;
    this->flag.enable=1;
    this->flag.delay=0;
}

void Scene_class::setFrame(unsigned int value)
{
    frame = value;
}


//================= communication======================
void Recev_Data_hanlder(){
    int dataSize=read(Serial, Recev_chr, 60);
    if(dataSize > 0)
    {
        for (unsigned char i=0;i<dataSize;i++)
        {
            if(Recev_chr[i]==PC2MCU_HEADER_){
                myCom.dataIndex=0;
                myCom.flagEnableCapute=1;
                Store_chr[myCom.dataIndex++]=Recev_chr[i];
            }else if((Recev_chr[i]==PC2MCU_TERMINATOR_)&&(myCom.flagEnableCapute)){
                myCom.flagDataReceived=1;
                myCom.flagEnableCapute=0;
                Store_chr[myCom.dataIndex++]=Recev_chr[i];

            }else if((myCom.flagEnableCapute)&&(myCom.dataIndex<_SERIAL_BUFF_SIZE))
            {
                Store_chr[myCom.dataIndex++]=Recev_chr[i];
            }else if(myCom.dataIndex>=_SERIAL_BUFF_SIZE){
                myCom.dataIndex=0;
            }

        }

        if(myCom.flagDataReceived)
        {
            myCom.flagDataReceived=0;
            memset(Recev_chr, '\0', sizeof(Recev_chr));
            /*
             * begin coding
             */

            //                    Trans_chr[0] = '4';
            //                    Send_Serial_String(Serial, Trans_chr, 1);

            //                    cout<<dec<<(int)dataIndex<<"begin";
            //                    for(unsigned char i=0;i<dataIndex;i++)
            //                    {
            //                        cout << hex << (int)Store_chr[i]<<":";
            //                    }
            //                    cout<<"End"<<endl;

            if(Store_chr[1]==PC_SAM_READ_ALL_POS12_){
                myCom.flagDataReceived_readAllPos12=1;
                memset(myCom.samPos12Avail, '\0', sizeof(myCom.samPos12Avail));
                //===============================
                myCom.NumofSam=(myCom.dataIndex-3)/4;
                for (unsigned char i=0;i<myCom.NumofSam;i++)
                {
                    if(Store_chr[i*4+5]==((Store_chr[i*4+2]^Store_chr[i*4+3]^Store_chr[i*4+4])&0x7F))
                    {
                        myCom.samPos12[Store_chr[i*4+2]&0x1F]=(Store_chr[i*4+4]&0x7F)+((Store_chr[i*4+3]&0x1F)<<7);
                        myCom.samPos12Avail[Store_chr[i*4+2]&0x1F]=1;
                    }
                    else
                        cout<<"error checksum 1"<<endl;
                }

            }else if(dataIndex==6){
                //                myCom.samPos12=(Store_chr[3]&0x7F)+((Store_chr[2]&0x1F)<<7);
                //                cout <<dec<< samPos12<<endl;
            }
        }
    }
}
