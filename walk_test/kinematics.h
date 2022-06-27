#include <math.h>
#define PI 3.14159

//============ GAIT PARAMS =============================================
#define STEP_SIZE 40
#define STEP_HEIGHT 10
#define N_TICKS 100 // number of ticks per cycle

//============ INVERSE KINEMATICS PARAMS ====================================
//link lenght
#define LEG_LENGHT 110
#define VERT_OFFSET  155.5635 //must be adjustable (default 45 degree angle between links)
#define HOR_OFFSET  20 //NOT ACTUAL VALUE!!!   <=================================
//motor offset
#define HIP_ANGLE_OFFSET 1.5708 //90 degree
#define FEMUR_ANGLE_OFFSET 0.7854 //45 degree
#define TIBIA_ANGLE_OFFSET 1.5708 //90degree1
//==========================================================================================

typedef struct state {

    int leg_i;
    int dt;

    float hipAngle;
    float femurAngle;
    float tibiaAngle;
    float c_x;
    float c_y;
    float c_z;
    float c_R;
    float c_P;
    float c_Y;
    int ticks;
    int mode;

    float p_x;
    float p_y;
    float p_z;

    bool swingPhase;
    
} STATE;

typedef struct command {
    
    float v_x;
    float v_y;
    float v_z;
    float roll;
    float pitch;
    float yaw;

} COMMAND;

class Kinematics 
{
    //PRIVATE SECTION

    //CONFIGS

    public:
    //=========================== PUBLIC SECTION =============================================
    STATE state;

    //Constructor
    Kinematics(int _leg_i, int _dt);

    //methods
    void parse_command(COMMAND command, STATE state);
    void gait_controller(STATE state);
    void compute_stance(STATE state);
    void compute_swing(STATE state);
    void compute_IK(STATE state);
    void motor(STATE state);

};
