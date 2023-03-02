#include "main.h" 
#include "lemlib/api.hpp"
#include "api.h"
#include "pros/adi.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"

pros::Rotation CataRot(12);
pros::Motor catapult(18);
pros::Motor lF(15);
pros::Motor lM(6);
pros::Motor lB(7);
pros::Motor rF(-9);
pros::Motor rM(-14);
pros::Motor rB(-13);
pros::Motor intake(10);
pros::MotorGroup leftMotors({lF, lM, lB});
pros::MotorGroup rightMotors({rF, rM, rB});
pros::Imu imu2(19);
pros::Imu imu1(17);
pros::ADIDigitalOut expansion ('A');
pros::ADIDigitalOut Bandboost ('B');
pros::Rotation verticalRot(1, true);
pros::ADIDigitalIn CataSwitch ('H');

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Controller partner(pros::E_CONTROLLER_PARTNER);

lemlib::TrackingWheel vertical(&verticalRot, 2.75, 0);

lemlib::OdomSensors_t sensors {
	&vertical,
        nullptr,
    nullptr,
        nullptr,
  &imu1
};

// kP as high as possible
// without it oscillating

// increase kP until it start oscillatin
// then increase kD until is stops oscillating

// repeat until oscillating cant stop


// forwards/backwards PID
lemlib::ChassisController_t lateralController {
	20, // kP
	200, // kD
	1,
	100,
	3,
	500,
};


// turning PID
lemlib::ChassisController_t angularController {
	15, // kP
	200, // kD
	1.5,
	100,
	3,
	500,
};

lemlib::Chassis chassis(&leftMotors, &rightMotors, 10000, lateralController, angularController, sensors);

void screen(void*) {
	// loop forever
  while (true) {
    // get robot position
    lemlib::Pose pose = chassis.getPose();
    pros::Controller master (pros::E_CONTROLLER_MASTER);

    // print on the screen
    master.print(0, 0, "Heading", CataRot.get_angle());
  	master.print(0, 0, "X: %f", pose.x);
  	master.print(0, 1, "Y: %f", pose.y);
    master.print(2, 2, "Heading: %f", pose.theta);
  	pros::lcd::print(0, "X: %f", pose.x);
    pros::lcd::print(1, "Y: %f", pose.y);
    pros::lcd::print(2, "Heading: %f", pose.theta);
    pros::lcd::print(3, "IntakeEfficency: %f", intake.get_efficiency());
    pros::lcd::print(4, "CatapultPower: %f", catapult.get_voltage());
    pros::lcd::print(5, "CatapultEfficency: %f", catapult.get_efficiency());
    pros::lcd::print(6, "CataRot: %f", CataRot.get_angle()/1000);

    // delay to save resources
    pros::delay(10);
  }
}

void Rollers(){
  intake.move(-127);
  pros::delay(800);
  intake.move(0);
}

void Start_Intake(){
intake.move(-127);
}

void Stop_Intake(){
intake.move(0);
}

void Expand (){
  expansion.set_value(true);
}

bool CataLimit = false;



// void Limit_Control (void*){
//   while(true){
//     if(CataSwitch.get_value() == true){
//       pros::delay(50);
//       CataLimit = true;

//     }else{
//       pros::delay(100);
//       CataLimit = false;
//     }
//   }
//   pros::delay(10);
// }




bool fireCata = false;
bool revCata = false;

void firePult() {
    fireCata = true;
    pros::delay(200);
    fireCata = false;
  }

void cataControl(void*) {
  CataRot.reset_position();
  //CataRot.reverse();
  CataRot.set_data_rate(10);
  catapult.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

  //int motor_power = 0;

  /*while (true) {
    // Get the current rotation value
    int rotation_value = CataRot.get_angle();
    
    if(CataRot.get_angle() <= 8635){

    // Calculate the error between the current position and desired position
    int error = (rotation_value - 8635) ;

    // Adjust the motor power based on the error
   if (error > 300){
    catapult.move(127);
    }
  else{
  catapult.move(86);
  }
    // Move the motor to the desired position
    
    }
    else{
      catapult.move(0);
      catapult.brake();
    }
*/


   while (true) {//true
    // Get the current rotation value
    int rotation_value = CataRot.get_angle();
    if(fireCata == true || CataRot.get_angle() <= 8800){//8635
      if(fireCata == true){
        catapult.move(130);
      }else if(CataRot.get_angle() <= 7000){
        catapult.move(130);
      }
      else if (7000<CataRot.get_angle()){//<= 7060
       catapult.move(95);
      }
    // Move the motor to the desired position
    }
    else{
      catapult.move(0);
      catapult.brake();
    }
   }

  pros::delay(10);
}




  //must hold to make move - 6:39pm feb 28th
   /*
   if(fireCata == true){
      catapult.move(125);
    }else if (CataLimit == false && CataRot.get_angle() > 27700) {
      //fully down - 270 degrees == 27000 on CataRot
      //full speed from 360 - 290
      //slower speed from 290 - 270
      catapult.move(125);
    }else if ((CataLimit == false && (CataRot.get_angle() <= 27700 && CataRot.get_angle() > 26500))){
      catapult.move(80);
    }else{
      catapult.brake();
    }
   */ 




// void Roller(){
//   intake.move_relative(1000);
// }




/**
 * Runs initialization code. This occurs as soon as the program is started. *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
  CataRot.reset_position();
  CataRot.reverse();
	pros::Task screen_task(screen);
  pros::Task Cata_task(cataControl);
  //pros::Task Cata_task2(Limit_Control);
  chassis.calibrate();
  chassis.setPose(60, -35, 90); //Change 180 when Liam tells u orientation
  //chassis.setPose(0, 0, 0);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
//setpos is in initialize

//chassis.turnTo(0, -24, 10000);

Start_Intake();
pros::delay(100);
chassis.moveTo(69.5, -35, 500, 70);
pros::delay(100);
chassis.moveTo(60, -35, 1000);
chassis.turnTo(43, -64, 1000);
chassis.moveTo(43, -65, 2000, 32.5);
pros::delay(100);
chassis.moveTo(39, -57, 1000);
chassis.turnTo(-7, -57, 1000, true);
chassis.moveTo(-7, -60, 3000, 80);
chassis.turnTo(-53, -53, 1000, true);
firePult();
chassis.turnTo(-13, -28, 1000);
chassis.moveTo(-13, -28, 5000, 20);
chassis.moveTo(-13, -60, 2000);
chassis.turnTo(-53, -53, 2000, true);
firePult();
chassis.turnTo(15, -38, 1000);
chassis.moveTo(15, -38, 2000, 50);
chassis.turnTo(-12, -12, 1000);
chassis.moveTo(-12, -12, 2000, 50);
chassis.turnTo(-53, -53, 1000, true);
chassis.moveTo(-20, -20, 1000);
firePult();
chassis.moveTo(-12, -12, 1000);
chassis.turnTo(-25, -20, 1000);
chassis.moveTo(-55, -20, 3000, 30);
pros::delay(1000);
chassis.turnTo(-53, -53, 1000, true);
firePult();
chassis.turnTo(-35, 12, 1000);
chassis.moveTo(-35, 12, 1000);
chassis.moveTo(-28, 20, 1500, 40);
chassis.turnTo(-60, -20, 1000, true);
chassis.moveTo(-60, -20, 1000, 80);
chassis.turnTo(-53, -53, 1000, true);
firePult();
chassis.turnTo(-48, 40, 1000);
chassis.moveTo(-48, 40, 1000);
chassis.turnTo(-69.5, 40, 1000);
chassis.moveTo(-69.5, 40, 1000, 70);
pros::delay(1000);
chassis.moveTo(-60, 40, 1000);
chassis.turnTo(-35, 35, 1000);
chassis.moveTo(-35, 35, 1000, 80);
chassis.moveTo(-21, 37, 5000, 30);
chassis.turnTo(-40, 56, 1000);
chassis.moveTo(-40, 56, 1000);
chassis.turnTo(-40, 70, 1000);
chassis.moveTo(-40, 60, 1000, 80); // Change to 70 for real field  
pros::delay(1000);
chassis.moveTo(-40, 60, 1000);
chassis.turnTo(20, 60, 1000, true);
chassis.moveTo(20, 60, 3000);
firePult();
chassis.turnTo(-16, 40, 1000);
chassis.moveTo(-16, 40, 1000);
chassis.turnTo(12, 12, 1000);
chassis.moveTo(12, 12, 1000);
chassis.turnTo(53, 53, 1000);
chassis.moveTo(20, 20, 1000);
firePult();
//good

// chassis.turnTo(40, -47, 1000, true);
// chassis.moveTo(40, -50, 1000);
// chassis.turnTo(-10, -57, 1000, true);
// chassis.moveTo(-10, -57, 3000);
// chassis.turnTo(-53, -53, 1000, true);
// firePult();
// chassis.moveTo(-8, -57, 1000);
// chassis.turnTo(15, -30, 1000);
// Start_Intake();
// chassis.moveTo(12, -36, 3000);
// chassis.turnTo(-12, -12, 1000);
// chassis.moveTo(-12, -11, 1500);
// chassis.turnTo(-53, -53, 1000, true);
// Stop_Intake();
// firePult();
// chassis.turnTo(-28, -17, 1000);
// Start_Intake();
// chassis.moveTo(-50, -18, 2500);
// chassis.turnTo(-53, -53, 1000, true);
// Stop_Intake();
// firePult();
// chassis.turnTo(-59, 59, 1000);
// chassis.moveTo(-59, 40, 1000);
// chassis.turnTo(-69, 40, 500);
// chassis.moveTo(-67, 40, 500);
// Rollers();
// chassis.moveTo(-59, 40, 500);
// chassis.turnTo(-59, 59, 1000);
// chassis.moveTo(-59, 59, 1000);
// chassis.turnTo(50, -60, 1000);
// Expand();


// Start_Intake();
// chassis.moveTo(40, -53, 1000000);
// Stop_Intake();
// chassis.moveTo(40, -65, 1000000);
// //Roller();
// chassis.turnTo(-20, -60, 1000000, true);
// chassis.moveTo(-20, -60, 1000000, true);
// firePult();
// chassis.turnTo(-20, -45, 1000000);
// Start_Intake();
// chassis.moveTo(-20, -20, 1000000);
// chassis.moveTo(-20, -60, 1000000);
// Stop_Intake();
// chassis.turnTo(-53, -53, 1000000, true);
// firePult();
// chassis.turnTo(12, -36, 1000000);
// Start_Intake();
// chassis.moveTo(15, -34, 1000000);
// chassis.moveTo(-12, -12, 100000);
// chassis.turnTo(-53, -53, 1000000, true);
// chassis.moveTo(-17, -17, 10000000, true);
// firePult();
// chassis.turnTo(-60, -18, 10000000);
// Start_Intake();
// chassis.moveTo(-60, -18, 10000000);
// Stop_Intake();
// chassis.turnTo(-53, -53, 10000000, true);
// firePult();
// chassis.turnTo(-35, 12, 10000000);
// chassis.moveTo(-35, 12, 10000000);


	// chassis.setPose(-12, 60, 180);
  // chassis.moveTo(-10, 36, 1000000);
  // Start_Intake();
  // chassis.turnTo(12,12, 1000000);
  // chassis.moveTo(12,12, 1000000);
  // chassis.turnTo(-60,-60, 1000000, true);
  // Stop_Intake();
  // firePult();
  // Start_Intake();
  // chassis.moveTo(12,)

  

  //chassis.m
  
  //chassis.turnTo(15,0,100000);
	//pros::Task screen_task(Setpult);
  //pros::delay(1000);
  //Firepult();
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
  // double turnImportance = 0.1;
  // leftMotors.set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);
  // rightMotors.set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);

  // while(true){

  //   double turnVal = pros::E_CONTROLLER_ANALOG_RIGHT_X;
  //   double moveVal = pros::E_CONTROLLER_ANALOG_RIGHT_X;
  //   double turnVolt = turnVal * 0.12;
  //   double moveVolt = moveVal *0.12 *(1 - std::abs(turnVolt/12.0) * turnImportance);

  //   double leftVolt = moveVolt + turnVolt;
  //   double rightVolt = moveVolt - turnVolt;

  //   leftMotors.move(leftVolt);
  //   rightMotors.move(rightVolt);

    leftMotors.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
    rightMotors.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);

  while (true) {
    //Driver Control

    int power = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    double CurvedPower = pow(power, 3) / pow(127, 2);
    //double CurvedTurn = pow(turn, 3) / pow(127, 2);
    int left = CurvedPower + turn;
    int right = CurvedPower - turn;
    leftMotors.move(left);
    rightMotors.move(right);

    if(power + turn == 0){
      leftMotors.brake();
      rightMotors.brake();
    }

    //Intake
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
      intake.move(-127);
    }else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      intake.move(127);
    }else{
      intake.brake();
    }

    //Catapult
    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
      firePult();
    }
    //Expansion
    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
      Expand();
    }

    if(partner.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
      Expand();
    }

    pros::delay(10);

  }
}
