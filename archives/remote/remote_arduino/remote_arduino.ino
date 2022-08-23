#define X_i 14
#define Y_i 15
#define Z_i 16
#define R_i 17
#define P_i 18
#define A_i 19

int X_raw = 0;
int Y_raw = 0;
int Z_raw = 0;
int R_raw = 0;
int P_raw = 0;
int A_raw = 0;

int X_val = 0;
int Y_val = 0;
int Z_val = 0;
int R_val = 0;
int P_val = 0;
int A_val = 0;

#define DEAD_MIN 400
#define DEAD_MAX 600

int dt = 100;
unsigned long previous_millis = 0; 

void setup() {
    
    Serial.begin(115200);
    pinMode(X_i, INPUT);
    pinMode(Y_i, INPUT);
    pinMode(Z_i, INPUT);
    pinMode(R_i, INPUT);
    pinMode(P_i, INPUT);
    pinMode(A_i, INPUT);

}

void loop() {
    
    unsigned long current_millis = millis();

    if (current_millis - previous_millis > dt) {

        previous_millis = current_millis;

        X_raw = analogRead(X_i);
        Y_raw = analogRead(Y_i);
        Z_raw = analogRead(Z_i);
        R_raw = analogRead(R_i);
        P_raw = analogRead(P_i);
        A_raw = analogRead(A_i);

        if (X_raw > DEAD_MIN && X_raw < DEAD_MAX) X_raw = 512;
        if (Y_raw > DEAD_MIN && Y_raw < DEAD_MAX) Y_raw = 512;
        if (Z_raw > DEAD_MIN && Z_raw < DEAD_MAX) Z_raw = 512;
        if (P_raw > DEAD_MIN && P_raw < DEAD_MAX) P_raw = 512;
        if (R_raw > DEAD_MIN && R_raw < DEAD_MAX) R_raw = 512;
        if (A_raw > DEAD_MIN && A_raw < DEAD_MAX) A_raw = 512;


        X_val = map(X_raw, 0, 1023, 1000, 2000);
        Y_val = map(Y_raw, 0, 1023, 1000, 2000); 
        Z_val = map(Z_raw, 0, 1023, 1000, 2000); 
        R_val = map(R_raw, 0, 1023, 1000, 2000); 
        P_val = map(P_raw, 0, 1023, 1000, 2000); 
        A_val = map(A_raw, 0, 1023, 1000, 2000);

        //String msg = String(X_val)+","+String(Y_val)+","+String(Z_val)+","+String(R_val)+","+String(P_val)+","+String(A_val);
        String msg = String(X_raw)+","+String(Y_raw)+","+String(Z_raw)+","+String(R_raw)+","+String(P_raw)+","+String(A_raw);

        Serial.println(msg);

        delay(dt);

        //Serial.write(&send_data, sizeof(SEND_DATA));

    }
}
