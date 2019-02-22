#include <kipr/botball.h>

//constants
int BLACK = 3960; 
double DEGREES_CONVERSION = 6444.444;

//ports
int rightM = 0;
int leftM = 3;
int leftIR = 1;
int rightIR = 0;

//<<-------------------------------------------GYROSCOPE CODE: EDIT WITH CAUTION-------------------------------------->>
//Gyroscope code "DemoGyroLib" created by Norman Advanced Robotics; used and updated by GMHS 
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
//<<---------------------------------------------------END GYROSCOPE CODE---------------------------------------------->>

//<<---------------------------------------------------OTHER USER METHODS---------------------------------------------->>

//a simple version of turning for simple turns
void turnL(double angle) {
    
    turn_with_gyro(-500, 500, angle);
    
}

void turnR(double angle) {
    
    turn_with_gyro(500, -500, angle);
    
}


//<<----------------------------------------------------END USER METHODS----------------------------------------------->>
int main() {
    
    //preparation
    declare_motors(leftM, rightM);
    calibrate_gyro();
    
    //getting out of the box
    while (analog(leftIR) < BLACK) { 
    	drive_with_gyro(1000, 10); //drive forward until the black line is detected
    } 
    
    drive_with_gyro(1000, 500);
    printf("Yeee boi i cleared the black tape\n") ; 
    
    
    //getting to the line
    while (analog(leftIR) < BLACK) { 
    	drive_with_gyro(1000, 10);
    } 
    printf("Yeee boi i found the black line\n") ;

    calibrate_gyro();
    turnL(90.0); //turning to face the potentially burning buildings 
    
    calibrate_gyro(); 
    while(analog(leftIR) < BLACK) {		//until robot reaches the corner,
    	if(analog(rightIR) < BLACK) {	// following the black line 
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
    
    int count = 0; 
    int timesFound = 0; 
    camera_open_black();
    camera_update();
    
    //looking for fire marker 25 times to ensure consistent results 
    while(count < 25) { 
        camera_update();
    	if(get_object_count(0) ==0){ 			//if nothing is seen
        	printf("yee boi there is no building here \n") ;
            count++;
    	} 
      	else { 											//when the marker is seen
            if(get_object_bbox(0, 0).width > 30) {		//ensuring that the proper object is seen 
                printf("Building has been found \n"); 
                timesFound++; 
                count++;
            }
            else {
            	printf("Marker has been ignored. \n");
            }
            printf("Size: %d\n", get_object_bbox(0, 0).width);
    	} 
    }
    
    //recalibration
    cg();
    
    //now that the camera has checked for the marker, check to make sure it saw the marker multiple times
    if (timesFound > 5) { 
        printf("Conclusion: There is a fire here \n") ; 
        int firstBuilding = 1;
        //go to the second building 
        
    }
    else { 
        printf("Conclusion: There is no fire here \n" ) ; //now must head towards the other building 
        int firstBuilding = 0;
        turnR(90.0);
        drive_with_gyro(1000, 2000);
    } 
    
    return 0;
}