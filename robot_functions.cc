
//#region config_globals
vex::brain Brain;
vex::motor rMotor(vex::PORT1, vex::gearSetting::ratio18_1, false);
vex::motor lMotor(vex::PORT3, vex::gearSetting::ratio18_1, true);
vex::gyro  myGyro(Brain.ThreeWirePort.A);
vex::sonar sonar_L(Brain.ThreeWirePort.C);
vex::sonar sonar_Top(Brain.ThreeWirePort.E);
vex::sonar sonar_Cube(Brain.ThreeWirePort.G);
//#endregion config_globals

/*
* pre autonomous function
* sets the brake mode of the motors  at "hold"
*/
void pre_auton(void){
    rMotor.setStopping(vex::brakeType::hold);
    lMotor.setStopping(vex::brakeType::hold);
}


/* 
* driveStop function
* this function intiates the motors to brake 
*/
void driveStop(){
    rMotor.stop();
    lMotor.stop();
}

/*
* intialSearch function
* This function spins the robot counter clockwise to search for cubes
* If it finds a cube, it prints it to the console
* By making the robot count the number of cubes at the beginning of the program, I can have my code only run that number of times.
* This way I can have as many cubes on the field as possible.
* Returns cubeCount
*/
int intialSearch(){
    int searchDistance = 480; //clipping distance in which it finds the cube
    int cubeCount = 0; //this variable stores the number of cubes on the field
    int bearing = 0; //stores the bearing of the gyro
    
    //setting velocity
    rMotor.setVelocity(8,vex::velocityUnits::pct);
    lMotor.setVelocity(8,vex::velocityUnits::pct);
    
    rMotor.spin(fwd);
    lMotor.spin(reverse);
    Brain.Screen.print("Finding Cubes......");
    
    while(true){
        bearing = myGyro.value(vex::rotationUnits::deg);
        
        // if sonar distance is less than or equal to the searchDistance, add 1 to cubeCount
        if(sonar_Cube.distance(vex::distanceUnits::mm) <= searchDistance){
            cubeCount++;
            sleepMs(580);// sleep for a small amount of time to allow the robot to move past the cube
            Brain.Screen.print(cubeCount);
            Brain.Screen.print(", ");
        }
        
        //robot will do a 360 degree turn then break out of the loop
        //i have it at 355 instead of 360 because of the delay in Robot Mesh
        if(bearing > 355){
            break;
        }
    }
    
    driveStop();
    
    //prints the total number of cubes to the console
    Brain.Screen.setCursor(3,1);
    Brain.Screen.print("Total number of cubes on the field = ");
    Brain.Screen.print(cubeCount);
    return cubeCount;

}

/*
* searchCube function
* this fucntion rotates the robot until it finds a cube
* returns "true" once a cube has been found
*/
bool searchCube(){
    int searchSpeed = 5; //speed at which the robot will turn
    int searchDistance = 480; //clipping distance for the cube
    
    //setting velocity
    rMotor.setVelocity(searchSpeed,vex::velocityUnits::pct);
    lMotor.setVelocity(searchSpeed,vex::velocityUnits::pct);
    
    rMotor.spin(fwd);
    lMotor.spin(reverse);
    
    
    while (sonar_Cube.distance(vex::distanceUnits::mm) > searchDistance){
        // this is kinda the refresh rate for the condition to check if it is true
        sleepMs(100);
    }
    driveStop();
    return true;
}

/* intake funciton
* robot drives forward toward the cube until it is 20mm away from the sensor.
* @param vel  - how fast the robot should drive toward the cube
*/
void intake(int vel){
    
    rMotor.setVelocity(vel,vex::velocityUnits::pct);
    lMotor.setVelocity(vel,vex::velocityUnits::pct);
    
    rMotor.spin(fwd);
    lMotor.spin(fwd);
    
    while(sonar_Cube.distance(vex::distanceUnits::mm) > 20){
        sleepMs(100);
    }
    driveStop();
}

/* 
* clockWiseTurn function
* turns the robot clockwise to set number of degrees
* @param degrees - turns the robot to this specific bearing
*/
void clockWiseTurn(int degrees){
    int bearing = 0;
    // robot turns at velocity of 2
    rMotor.setVelocity(2,vex::velocityUnits::pct);
    lMotor.setVelocity(2,vex::velocityUnits::pct);
    
    rMotor.spin(reverse);
    lMotor.spin(fwd);
    
    while(true){
        bearing = myGyro.value(vex::rotationUnits::deg);
        
        // if the bearing is equal to degrees, break from the loop
        if(bearing == degrees){
            break;
        }
    }
    driveStop();
}

/* 
* counterTurn function
* turns the robot counter clock wise to set number of degrees
* @param degrees - turns the robot to this specific bearing
*/
void counterTurn(int degrees){
    int bearing = 0;
    //robot turns at velocity of 2
    rMotor.setVelocity(2,vex::velocityUnits::pct);
    lMotor.setVelocity(2,vex::velocityUnits::pct);
    
    rMotor.spin(fwd);
    lMotor.spin(reverse);
    
    while(true){
        bearing = myGyro.value(vex::rotationUnits::deg);
        
        //if the bearing is equal to degrees, break from the loop
        if(bearing == degrees){
            break;
        }
    }
    driveStop();
}

/* 
* allignWall function
* alligns the robot to the back wall for precision
* @param time - amount of time in Ms the robot should drive backward for
* @param vel - how fast the robot should drive
*/
void allignWall (int time, int vel){
    rMotor.setVelocity(vel,vex::velocityUnits::pct);
    lMotor.setVelocity(vel,vex::velocityUnits::pct);
    
    rMotor.spin(reverse);
    lMotor.spin(reverse);
    sleepMs(time);
    driveStop();
}

/* 
* searchWall function
* uses the sonar_L sensor to search for a wall
* returns "true" once a wall is found
*/
bool searchWall(){
    int searchSpeed = 5; //speed for the robot to drive
    int searchDistance = 435; //clipping distance for the wall
    rMotor.setVelocity(searchSpeed,vex::velocityUnits::pct);
    lMotor.setVelocity(searchSpeed,vex::velocityUnits::pct);
    
    rMotor.spin(fwd);
    lMotor.spin(fwd);
    
    while (sonar_L.distance(vex::distanceUnits::mm) > searchDistance){
        sleepMs(100);
    }
    driveStop();
    return true;
}

/* 
* scoreThenBack function
* Drives the robot towards the scoring zone
* deposits the cube by driving backward
*/
void scoreThenBack(){
    int searchSpeed = 20; // speed at which the robot should drive
    int scoringDistance = 50; //distance for how far the wall should be from the sonar_Top sensor when driving towards the wall
    int backupDistance = 700; //distance for how far the wall should be from the sonar_Top sensor when driving away from the wall
    rMotor.setVelocity(searchSpeed,vex::velocityUnits::pct);
    lMotor.setVelocity(searchSpeed,vex::velocityUnits::pct);
    
    rMotor.spin(fwd);
    lMotor.spin(fwd);
    
    while(sonar_Top.distance(vex::distanceUnits::mm) > scoringDistance){
        sleepMs(100);
    }
    driveStop();
    
    //waits half a second before driving back
    sleepMs(500);
    
    rMotor.spin(reverse);
    lMotor.spin(reverse);
    
    while(sonar_Top.distance(vex::distanceUnits::mm) < backupDistance){
        sleepMs(100);
    }
    driveStop();
}

void autonomous(void){
    int cubeCount = intialSearch(); //intiates the intialSearch funtion and stores the value in cubeCount
    
    //I have my code in a for loop so I can score as many cubes as I wish because I made the robot search for cubes
    //at the beginning of the program. 
    //This loop will repeat for each cube
    for (int i = 0; i < cubeCount; i++){
       
       //if search == true, intake the cube
       if(searchCube() == true){
        intake(5);
    }
    
    //If the robot is facing the bottom right or bottom left quadrant, do these actions
    //This is so the robot turns depedning on which way of turning is faster to finish the task
    //decreases the amount of time it takes for the program to run
    if (myGyro.value(vex::rotationUnits::deg) < 90 ||myGyro.value(vex::rotationUnits::deg) > 270){
        
        //if in the bottom right quadrant, turn counter clockwise
        if(myGyro.value(vex::rotationUnits::deg) < 90){
            counterTurn(180);
        }
        // else turn clockwise, which means it is in the bottom left quadrant
        else{
            clockWiseTurn(180);
        }
        
        //back up for 4 seconds at velocity of 10
        allignWall(4000, 10);
    }
    
    //else, do these actions
    else{
        counterTurn(0);
        
        allignWall(4000, 10);
    }
    
    //drives to get the cube again
    if(searchCube() == true){
        intake(5);
    }
    
    //searchs for the wall opposite to the robot so the robot will be in the vertical center of the field
    if(searchWall() == true){
        
        //if in the bottom half of the field, turn counterclockwise
        if (myGyro.value(vex::rotationUnits::deg) < 90 || myGyro.value(vex::rotationUnits::deg) > 270){
            counterTurn(90);
        }
        //else turn clockwise
        else{
            clockWiseTurn(90);
        }
        
    }
    
    //scores the cube then drives back
    scoreThenBack(); 
    
    //do it all over again for the number of cubes
    }
}
