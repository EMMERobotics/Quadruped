#include <math.h>
#define PI 3.14159

class Kinematics 
{
    //PRIVATE SECTION
    int leg_i; //leg_index (FL, FR, BL, BR) = (1, 2, 3, 4)
    int dt;

    //CONFIGS
    //============ GAIT PARAMS =============================================
    #define STEP_SIZE 50
    #define STEP_HEIGHT 10
    #define N_TICKS 100 // number of ticks per cycle

    //============ INVERSE KINEMATICS PARAMS ====================================
    //link lenght
    #define LEG_LENGHT 110
    float VERT_OFFSET = LEG_LENGHT*sqrt(2); //must be adjustable (default 45 degree angle between links)
    float HOR_OFFSET = 20; //NOT ACTUAL VALUE!!!   <=================================
    //motor offset
    #define HIP_ANGLE_OFFSET 1.5708 //90 degree
    #define FEMUR_ANGLE_OFFSET 0.7854 //45 degree
    #define TIBIA_ANGLE_OFFSET 1.5708 //90degree
    //==========================================================================================

    //=========================== PUBLIC SECTION =============================================
    public: 

    //state
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


    //constructor
    Kinematics(int _leg_i, int _dt) {
        leg_i = _leg_i;
        dt = _dt;

        //set default angle when object is created
        hipAngle = PI/2;
        femurAngle = PI/4;
        tibiaAngle = PI/2;
        c_x = 0;
        c_y = 0;
        c_z = 0;
        ticks = 0;
        p_x = 0;
        p_y = 0;
        p_z = 0; //relative to its default pose with 45 degree angle between links

        if (leg_i == 1 || leg_i == 4) swingPhase = true;
        else swingPhase = false;
    }

    //methods

    void parse_command(float _c_x, float _c_y, float _c_z) {  //not necessary, just parse command direcly to gait_controller()
        // will add RPY later
        c_x = _c_x;
        c_y = _c_y;
        c_z = _c_z;
    }

    void gait_controller(float c_x, float c_y, float c_z) {

        /* 
        UNFINISHED
            Add overlap time support
            Add trasnsition

        GOALS:
            Determine the steps' frequency from the input velocity
        
        Conditions:
            The step lenght is fixed

        Input:
            Crawl/Trot velocity command (mm/s)
            Time

        Output:
            Contact mode of each legs (Swing legs, Stance legs)
            Phase of each swing (ticks) since each step is executed

        */

       int incremented_ticks;
       float period_x; //ms for 1 cycle
       float ms_per_ticks;

       if (ticks == 100) {
           ticks = 0;
           swingPhase = !swingPhase;
       }

       period_x = 1000 * STEP_SIZE/c_x;
       ms_per_ticks = period_x / N_TICKS;
       incremented_ticks = ceil(dt/ ms_per_ticks); //ceil or floor works better???

       ticks += incremented_ticks;
       if (ticks > N_TICKS) ticks = N_TICKS;

       if (!swingPhase) compute_stance(ticks);
       if (swingPhase) compute_swing(ticks);

    } 

    void compute_stance(int ticks) {
        /*
        GOALS:
            Compute the (X,Y,Z,R,P,Y) of each leg from the phase ticks for the stance legs

        Input:
            Contact mode
            Ticks

        Output:
            (X,Y,Z,R,P,Y) for each stance legs
        */

       p_x = STEP_SIZE/2 - (STEP_SIZE*ticks/100);
    }

    void compute_swing(int ticks) {
        /*
        GOALS:
            Compute the (X,Y,Z,R,P,Y) of each leg from the phase ticks for the swing legs

        Input:
            Contact mode
            Ticks

        Output:
            (X,Y,Z,R,P,Y) for each swing legs
        */
    }

    void compute_IK(int leg_i, float x, float y, float z, float roll, float pitch, float yaw) {

        if (leg_i == 2 || leg_i == 4) {
            y *= -1;
            HOR_OFFSET *= -1;  
        }

        //only x,y,z, for now
        float hip_dis;
        float leg_dis;
        float theta;

        hip_dis = pow((VERT_OFFSET - z), 2) + pow(y +  HOR_OFFSET, 2); //squared
        leg_dis = hip_dis - pow(HOR_OFFSET, 2) + pow(x,2);
        hip_dis = sqrt(hip_dis);
        theta = atan(x/(VERT_OFFSET -z));
        
        hipAngle = acos(HOR_OFFSET/hip_dis) + atan((y +  HOR_OFFSET)/(VERT_OFFSET - z));
        femurAngle = atan(x/(VERT_OFFSET -z)) + acos(leg_dis/ (2*LEG_LENGHT*sqrt(leg_dis)));
        tibiaAngle = acos((2*pow(LEG_LENGHT,2) - leg_dis)/(2*pow(LEG_LENGHT,2)));
    }

    void motor() {}

};
