#include <kipr/botball.h>

int BLACK = 4000; 
int BURNING = 0; //is the first bulding burning-- closer to the firemen
double DEGREES_CONVERSION = 6444.444;
int rightM = 0;
int leftM = 3;
int leftIR = 1;
int rightIR = 0;

double bias; //variable to hold the calibration value
int right_motor, left_motor;//ports

//initializes the ports for the motors.
void declare_motors(int lmotor, int rmotor)
{
    right_motor = rmotor;
    left_motor = lmotor;
}

//Gyroscopes are always off by a specific angular velocity so we need to subtract this bias from all readings.
//Call calibrate_gyro to find what the gyroscope reads when the robot is at a complete stop (this is what the bias is). 
//The bias may change between turning the robot on when compared to the bias that the gyroscope has when it starts moving.
void calibrate_gyro()
{
   	//takes the average of 50 readings
    int i = 0;
    double avg = 0;
    while(i < 50)
    {
        avg += gyro_z();
        msleep(1);
        i++;
    }
    bias = avg / 50.0;
    printf("New Bias: %f\n", bias);//prints out your bias. COMMENT THIS LINE OUT IF YOU DON'T WANT BIAS READINGS PRINTED OUT
}

//the same as calibrate_gyro() except faster to type
void cg()
{
    calibrate_gyro();
}
		
//turns the robot to reach a desired theta. 
//If you are expecting this function to work consistantly then don't take your turns too fast.
void turn_with_gyro(int left_wheel_speed, int right_wheel_speed, double targetTheta)
{
    double theta = 0;//declares the variable that stores the current degrees
    targetTheta = targetTheta * DEGREES_CONVERSION;	//uses degrees instead of KIPR degrees
    mav(right_motor, right_wheel_speed);//starts the motors
    mav(left_motor, left_wheel_speed);
    //keeps the motors running until the robot reaches the desired angle
    while(theta < targetTheta)
    {
        msleep(10);//turns for .01 seconds
        theta += abs(gyro_z() - bias) * 10;//theta = omega(angular velocity, the value returned by gyroscopes) * time
    }
    //stops the motors after reaching the turn
    mav(right_motor, 0);
    mav(left_motor, 0);
}

//drives straight forward or backwards. The closer speed is to 0 the faster it will correct itself 
//and the more consistent it will be but just do not go at max speed. Time is in ms. 
void drive_with_gyro(int speed, double time)
{ 
    double startTime = seconds();
    double theta = 0;
    while(seconds() - startTime < (time / 1000.0))
    {
        if(speed > 0)
        {
            mav(right_motor, (double)(speed - speed * (1.920137e-16 + 0.000004470956*theta - 7.399285e-28*pow(theta, 2) - 2.054177e-18*pow(theta, 3) + 1.3145e-40*pow(theta, 4))));  //here at NAR, we are VERY precise
            mav(left_motor, (double)(speed + speed * (1.920137e-16 + 0.000004470956*theta - 7.399285e-28*pow(theta, 2) - 2.054177e-18*pow(theta, 3) + 1.3145e-40*pow(theta, 4))));
        }
        else//reverses corrections if it is going backwards
        {
            mav(right_motor, (double)(speed + speed * (1.920137e-16 + 0.000004470956*theta - 7.399285e-28*pow(theta, 2) - 2.054177e-18*pow(theta, 3) + 1.3145e-40*pow(theta, 4)))); 
            mav(left_motor, (double)(speed - speed * (1.920137e-16 + 0.000004470956*theta - 7.399285e-28*pow(theta, 2) - 2.054177e-18*pow(theta, 3) + 1.3145e-40*pow(theta, 4))));
        }
        //updates theta
        msleep(10);
        theta += (gyro_z() - bias) * 10;
    }
}

void simple_drive_with_gyro(int speed, double time)
{
    double startTime = seconds(); //used to keep track of time
    double theta = 0; //keeps track of how much the robot has turned
    while(seconds() - startTime < time)
    {
        //if the robot is essentially straight then just drive straight
        if(theta < 1000 && theta > -1000)
        {
            mav(right_motor, speed);
            mav(left_motor, speed);
        }
        //if the robot is off to the right then drift to the left
        else if(theta < 1000)
        {
            mav(right_motor, speed + 100);
            mav(left_motor, speed - 100);
        }
        //if the robot is off to the left then drift to the right
        else
        {
            mav(right_motor, speed - 100);
            mav(left_motor, speed + 100);
        }
        //updates theta
        msleep(10);
        theta += (gyro_z() - bias) * 10;
    }
}

int main() {
    //preparation
    declare_motors(leftM, rightM);
    calibrate_gyro();
    
    //getting out of the box
    while (analog(leftIR) < BLACK) { 
    	simple_drive_with_gyro(1000, 0.001); //drive forward until the black line is detected
    } 
    
    simple_drive_with_gyro(1000, 0.5);
    printf("Yeee boi i cleared the black tape\n") ; 
    
    
    //getting to the line
    while (analog(leftIR) < BLACK) { 
    	simple_drive_with_gyro(1000, .0010);
    } 
    printf("Yeee boi i found the black line\n") ;

    calibrate_gyro();
    turn_with_gyro(-500, 500, 90.0); //turning to face the buildings 
    
    calibrate_gyro(); // following the black line 
    while(analog(leftIR) < BLACK) {
    	if(analog(rightIR) < BLACK) {
            printf("Seeing white");
            mav(rightM, 900);
            mav(leftM, 600);
        } else {
            printf("Seeing black");
            mav(rightM, 600);
            mav(leftM, 900);
        }
        msleep(50);
        ao();
    }
    
    camera_open_black();
    camera_update();
    
    printf(" yee boi looking for building...");
    
   int yes = 1; 
    int b = 0; 
    camera_open_black();
    camera_update();
    
    while( yes < 25 ) { 
        camera_update();
    	if(get_object_count(0) ==0){ 
        	printf( " yee boi there is no building here" ) ;
            yes++;
    	} 
      	else { 
        	printf("  yee boi ive FINALLY e building" ); 
        	b = 1 ; 
        	yes++;
    	} 
    }
    
    if (b==1 ) { 
        printf( "YEE BOI there is a burning building" ) ; 
        // drop off the fireman 
        
    }
    else { 
        printf("Yee boi there isnt a buring building" ) ; 
        turn_with_gyro(500, -500, 90.0);
    } 
    
   
    
    return 0;
}