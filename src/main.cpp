#include "main.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <list>

using namespace pros;

/*****************************************
 * 
 * 
 * 
 *   SETUP
 * 
 * 
 * 
******************************************/

Motor FL(11, E_MOTOR_GEARSET_06, 1);
Motor BL(13, E_MOTOR_GEARSET_06, 1);
Motor ML(12, E_MOTOR_GEARSET_06, 1);
Motor FR(14, E_MOTOR_GEARSET_06, 0);
Motor BR(17, E_MOTOR_GEARSET_06, 0);
Motor MR(15, E_MOTOR_GEARSET_06, 0);

Motor INTL(4, E_MOTOR_GEARSET_06, 1);
Motor INTR(5, E_MOTOR_GEARSET_18, 0);

MotorGroup leftMotors({BL, ML, FL});
MotorGroup rightMotors({BR, MR, FR});
MotorGroup intake({INTL, INTR});

ADIDigitalOut FRwing('A');
ADIDigitalOut FLwing('B');
ADIDigitalOut BRwing('C');
ADIDigitalOut BLwing('D');

Distance DIST(7);
Imu INR(5);
Rotation ROT(6);

ADILed frontLEDs(8, 27);
ADILed leftDriveLEDs(7, 27);
ADILed rightDriveLEDs(6, 27);

bool blockerUp = false;
int globalCataSpeed = 90;
int cataDelay = 10; // in ms
bool autoReadyOn;
bool competitionMode = false;
bool driveControlStarted = false;
bool disabledMode = false;
bool endGame = false;
bool autoFireOn = false;
bool autoHangOn = false;

Controller Controller1(CONTROLLER_MASTER);
Controller Controller2(CONTROLLER_PARTNER);

int triballsFired = 0;

bool flowOn = false;
bool flashOn = false;
bool sparkOn = false;
std::vector<uint32_t> colors;
u_int32_t tempColor;
u_int32_t tempColor2;
int speed;
bool LEDbuttonToggle = false;

lemlib::Drivetrain_t drivetrain {
	&leftMotors,
	&rightMotors,
	10, // distance between front wheels
	2.75, // wheel diameter
	450, // wheel rpm (gearing * motor rpm)
	2 // chasePower
};

// forward/backward PID
lemlib::ChassisController_t lateralController {
    100, // kP
    150, // kD
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    8 // slew rate
};
 
// turning PID
lemlib::ChassisController_t angularController {
    4, // kP
    35, // kD
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    2 // slew rate
};

// odometry struct
lemlib::OdomSensors_t sensors {
	nullptr, // vertical tracking wheel 1
	nullptr, // vertical tracking wheel 2
	nullptr, // horizontal tracking wheel 1 (add later)
    nullptr, // horizontal tracking wheel 2
    &INR // inertial sensor
};

// create the chassis
lemlib::Chassis chassis(drivetrain, lateralController, angularController, sensors);

/*****************************************
 * 
 * 
 * 
 *   CUSTOM FUNCTIONS
 * 
 * 
 * 
******************************************/

void overheatWarning(Motor motor) {
    if (motor.get_port() > 0 && motor.get_temperature() >= 74) {
		// print overheat statement in the form "OVERHEAT (motor, temp)"
        Controller1.set_text(2, 4, "OVERHEAT (" + std::to_string(motor.get_port()) + ", " + std::to_string(motor.get_temperature()) + ")");
    }
}

void screenDisplay1() {
    while (true) {
        lemlib::Pose pose = chassis.getPose(); 
        lcd::print(0, "x: %f", pose.x); 
        lcd::print(1, "y: %f", pose.y); 
        lcd::print(2, "heading: %f", pose.theta); 
        delay(10);
    }
}

void autoHang() {
	while(true) {
		if (autoHangOn) {
			// implement wings 
			autoHangOn = !autoHangOn;
		}
		delay(100);
	}
}

void controllerScreen() {
	while (true) {
		Controller1.set_text(1,1, "CS:" + std::to_string(globalCataSpeed) + " AF:" + std::to_string(autoFireOn) + " L:" + std::to_string(autoReadyOn));
		//Controller1.set_text(1,1, "CR_T:" + std::to_string(CR.get_torque()) + " CL_T:" + std::to_string(CL.get_torque()));
		//Controller1.set_text(1,1, "ROT:" + std::to_string(ROT.get_angle()));
		delay(100);
	}
}

void driveMove(int power) {
	BL.move(power);
	BR.move(power);
	ML.move(power);
	MR.move(power);
	FL.move(power);
	FR.move(power);
}

// RGB CONTROL BELOW

void ledUpdater() {
	while(true){
		frontLEDs.update();
		delay(20);
		rightDriveLEDs.update();
		delay(20);
		leftDriveLEDs.update();
		delay(20);
	}
}

uint32_t hexToDec(const std::string& hex) {
    std::stringstream ss;
    ss << std::hex << hex;
    uint32_t dec;
    ss >> dec;
    return dec;
}

std::string decToHex(uint32_t dec) {
    std::stringstream ss;
    ss << std::hex << std::setw(8) << std::setfill('0') << dec;
    return ss.str();
}

std::vector<uint32_t> genGradient(uint32_t startColor, uint32_t endColor, size_t length) {
    std::vector<uint32_t> gradient;
    gradient.reserve(length);

    // extract RGB components of startColor
    uint8_t startR = (startColor >> 16) & 0xFF;
    uint8_t startG = (startColor >> 8) & 0xFF;
    uint8_t startB = startColor & 0xFF;

    // extract RGB components of endColor
    uint8_t endR = (endColor >> 16) & 0xFF;
    uint8_t endG = (endColor >> 8) & 0xFF;
    uint8_t endB = endColor & 0xFF;

    // calculate the step size for each color component
    double stepR = static_cast<double>(endR - startR) / (length - 1);
    double stepG = static_cast<double>(endG - startG) / (length - 1);
    double stepB = static_cast<double>(endB - startB) / (length - 1);

    // generate the gradient
    for (size_t i = 0; i < length; ++i) {
        uint8_t r = static_cast<uint8_t>(startR + (stepR * i));
        uint8_t g = static_cast<uint8_t>(startG + (stepG * i));
        uint8_t b = static_cast<uint8_t>(startB + (stepB * i));

        // Combine the RGB components into a single uint32_t color
        uint32_t color = (r << 16) | (g << 8) | b;
        gradient.push_back(color);
    }

    return gradient;
}

// update if more led strands are added
void set_pixel(u_int32_t color, int i) {
	frontLEDs[i] = color;
	leftDriveLEDs[i] = color;
	rightDriveLEDs[i] = color;
}

void set_all(u_int32_t color) {
	frontLEDs.set_all(color);
	leftDriveLEDs.set_all(color);
	rightDriveLEDs.set_all(color);
}

void LEDclear() {
	flowOn = false;
	flashOn = false;
	sparkOn = false;
}

void flow(uint32_t color1, u_int32_t color2) {
	flowOn = true;
	flashOn = false;
	sparkOn = false;
	colors = genGradient(color1, color2, 60);
}

void flash(uint32_t color, int flashSpeed, u_int32_t color2 = 0x000000){
	flashOn = true;
	flowOn = false;
	sparkOn = false;
	tempColor = color;
	tempColor2 = color2;
	speed = flashSpeed;
}

void spark(uint32_t color, int sparkSpeed) {
	sparkOn = true;
	flowOn = false;
	flashOn = false;
	tempColor = color;
	speed = sparkSpeed;
}

void LEDmainLoop() {
	while (true) {

		if (flowOn) {

			// loop through each pixel gets a color, update buffer, shift color matrix by 1, repeat
			for (int i = 0; i < 60; ++i) {
				set_pixel(colors[i], i);
			}
			
			// shift color vector
			std::rotate(colors.begin(), colors.begin()+1, colors.end());
		}

		else if (flashOn) {
			set_all(tempColor);
			delay(speed*100);
			set_all(tempColor2);
			delay(speed*100);
		}

		else if (sparkOn) {
			sparkOn = false;
			for (int i = 0; i < 60; ++i) {
				set_pixel(tempColor, i);
				set_pixel(tempColor, i);
			}
		}

		delay(50);
	}
}

void competitionTimerStuff() {
	delay(75000); // 30 seconds left
	endGame = true;
}

bool resting = false;
void RGBcontrol() {
	while(true) {

		if (!endGame) {

			if (autoFireOn) {
				resting = false;
				flash(0x39FF14, 6);
				while (autoFireOn && !endGame) {
					delay(50);
				}
				flashOn = false;
				resting = true;
			}

			else if (std::abs(INTR.get_actual_velocity()) > 100) {
				resting = false;
				flash(0x1F51FF, 1, 0xFFFFFF);
				while (std::abs(INTR.get_actual_velocity()) > 100) {
					delay(50);
				}
				flashOn = false;
				resting = true;
			}

			else if (resting) {
				flow(0xFFFFFF, 0xFF00FF);
				delay(50);
				resting = false;
			}

	}

	if (competitionMode && endGame) {
		flash(0xE9D502, 5);
		delay(15000); // 15 seconds left
		flash(0xD22730, 3);
		delay(15000);
		endGame = false;
		Task competitionTimerTask(competitionTimerStuff);
	}
	
	delay(50);
	}
}

/*****************************************
 * 
 * 
 * 
 *   AUTONOMOUS AND DRIVER CONTROL
 * 	 (Use https:// path.jerryio.com/ ) - width: , length: 
 * 
 * 
 * 
******************************************/

void skillsStart() {
	autoFireOn = true;
	chassis.setPose(-48, -55, 180);
	chassis.turnTo(-56, -28, 400, false, true);
	chassis.moveTo(-56, -28, 180, 1000, false, false, 20, 0.3);
	chassis.moveTo(-56, -40, 180, 1500);
	chassis.turnTo(46, 6, 700);
	BLwing.set_value(1);
}

void nearsideRushSafe() {
	chassis.setPose(-32, -55, 0);
	intake.move(127);

	// grab central triball and back up to bar
	chassis.moveTo(-23, -12, 10, 1300, false, true, 20);
	chassis.turnTo(-23, -28, 800, false, true);
	chassis.moveTo(-23, -28, chassis.getPose().theta, 1500, false, false);
	chassis.turnTo(-57, -51, 800, false, true);
	chassis.moveTo(-57, -51, chassis.getPose().theta, 2000, false, false, 0, 0.6, 50);

	// knock out match load
	chassis.turnTo(-49, -55, 1000, false, true, 50);
	chassis.moveTo(-49, -55, chassis.getPose().theta, 1000, false, false);
	chassis.turnTo(-9, -62, 1000);
	intake.move(-127);
	chassis.moveTo(-18, -60, 90, 2000);
	chassis.moveTo(-44, -60, 90, 1500, false, false);
	chassis.turnTo(-53, -49, 800, false, true);
	chassis.moveTo(-53, -49, chassis.getPose().theta, 1000, false, false);
	BRwing.set_value(1);
}

void nearsideAWP() {
	autoFireOn = true;
	chassis.setPose(-48, -55, 180);
	chassis.turnTo(-56, -28, 400, false, true);
	chassis.moveTo(-56, -28, 180, 1000, false, false, 20, 0.3);
	chassis.moveTo(-56, -42, 180, 1000);
	chassis.turnTo(-47, -58, 1000);
	BRwing.set_value(1);
	delay(300);
	chassis.moveTo(-46, -58, chassis.getPose().theta, 1500);
	chassis.turnTo(-9, -61, 800);
	BRwing.set_value(0);
	delay(300);
	chassis.moveTo(-14, -61, 90, 2000);

}

void nearsideRushRisky() {
	driveMove(-100);
}

void sixBallMidrush() {
	// set pose
	chassis.setPose(32, -53, 0);
	intake.move(127);

	// hit triball and grab 
	FRwing.set_value(1);
	delay(200);
	chassis.moveTo(4, -4, 330, 600);
	FRwing.set_value(0);
	chassis.moveTo(4, -4, 330, 1000);

	// push triballs
	chassis.turnTo(59, chassis.getPose().y, 1500);
	FRwing.set_value(1);
	FLwing.set_value(1);
	delay(300);
	intake.move(-127);
	driveMove(100);
	delay(600);
	chassis.setPose(40, chassis.getPose().y, chassis.getPose().theta);
	FRwing.set_value(0);
	FLwing.set_value(0);
	driveMove(-40);
	delay(300);
	driveMove(0);
	intake.move(127);

	// grab center back
	chassis.turnTo(10, -17, 2000);
	chassis.moveTo(10, -17, chassis.getPose().theta, 1500);

	// move back and knock out triball
	chassis.turnTo(54, -50, 800);
	chassis.moveTo(54, -50, chassis.getPose().theta, 2000);
	chassis.turnTo(80, -27, 800, false, true);
	BRwing.set_value(1);
	delay(300);
	chassis.turnTo(63, -43, 1000, false, true);
	chassis.moveTo(63, -43, chassis.getPose().theta, 1500, false, false);
	BRwing.set_value(0);
	chassis.turnTo(65, -27, 900);
	FLwing.set_value(1);
	FRwing.set_value(1);
	delay(300);
	driveMove(100);
	delay(600);
	driveMove(-20);
	delay(400);
	driveMove(0);

	// grab under elevation bar

	// score 4 triballs
	
}

void skills() {
	// run the skills start (scores alliance triballs and lines up for shooting)
	skillsStart();

	// delay for 25 seconds to shoot
	delay(25000);

	// turn autofire on to stop cata from hitting bar
	autoFireOn = false;
	BRwing.set_value(0);

	// turn and move backwards to other side
	// pushing triballs along with robot
	chassis.turnTo(-27, -58, 700, false, true);
	chassis.moveTo(-27, -58, 300, 2500, false, false);
	chassis.turnTo(41, -58, 800, false, true);
	chassis.moveTo(41, -58, 270, 4000, false, false);
	chassis.turnTo(61, -42, 800, false, true);
	chassis.moveTo(61, -42, chassis.getPose().theta, 1000, false, false);


	// backwards push corner triballs into goal twice
	chassis.turnTo(61, -32, 800, false, true);
	chassis.moveTo(61, -32, 180, 2000, false, false, 40, 0.3);
	driveMove(30);
	delay(300);
	driveMove(-90);
	delay(500);
	driveMove(0);
	chassis.setPose(59, -32, chassis.getPose().theta);
	
	// move out of the corner and towards the middle,
	// deploying wings on way to collect triballs
	chassis.moveTo(59, -42, 180, 1500);
	chassis.turnTo(9, -27, 800, false, true);
	chassis.moveTo(9, -27, chassis.getPose().theta, 2500, false, false);
	
	// front right push
	chassis.turnTo(39, -14, 1500, false, true);
	BRwing.set_value(1);
	delay(300);
	driveMove(-90);
	delay(1000);
	driveMove(0);
	chassis.setPose(40, chassis.getPose().y, chassis.getPose().theta);

	// front left push
	chassis.moveTo(10, chassis.getPose().y - 10, 270, 1500);
	BRwing.set_value(0);
	chassis.turnTo(10, 26, 1000, false, true);
	chassis.moveTo(10, 26, 0, 2500, false, false);
	chassis.turnTo(42, 0, 800, false, true);
	BRwing.set_value(1);
	delay(300);
	driveMove(-90);
	delay(1500);
	chassis.setPose(40, chassis.getPose().y, chassis.getPose().theta);
	driveMove(50);
	delay(300);
	driveMove(0);
	
	// front middle push
	BRwing.set_value(0);
	delay(300);
	driveMove(70);
	delay(600);
	driveMove(0);
	chassis.turnTo(187, -5, 1000);
	FLwing.set_value(1);
	FRwing.set_value(1);
	delay(300);
	driveMove(90);
	delay(1000);
	chassis.setPose(30, chassis.getPose().y, chassis.getPose().theta);
	driveMove(-20);
	delay(500);
	driveMove(0);
	FLwing.set_value(0);
	FRwing.set_value(0);
	chassis.turnTo(10, -31, 800);
	chassis.moveTo(10, -31, chassis.getPose().theta, 1500);
	chassis.turnTo(0, 0, 800);
	FLwing.set_value(1);
	FRwing.set_value(1);
	delay(300);
	driveMove(100);
	delay(700);
	driveMove(-20);
	delay(400);
	driveMove(0);
	

	// elevate?

}

ASSET(skillspath1_txt);

void skills2() {
	chassis.follow(skillspath1_txt, 15000, 4);
	FLwing.set_value(1);
	FRwing.set_value(1);
	chassis.moveTo(40, 0, 90, 2000, false, true, 100, 0);
}

/*****************************************
 * 
 * 
 * 
 *   VEX BUILT IN CODE
 * 
 * 
 * 
******************************************/

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	chassis.calibrate();
	lcd::initialize();
  	selector::init();
	//Task brainScreen(screenDisplay1)
	Task controllerScreenTask(controllerScreen);
	Task ledUpdaterTask(ledUpdater);
	Task leds(LEDmainLoop);
	Task rgbcontrolTask(RGBcontrol);
	//Task autoHangTask(autoHang);
	ROT.reset();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
	disabledMode = true;
	driveControlStarted = false;
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
	competitionMode = true;
}

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
	autoFireOn = true;

    if(selector::auton == 1){nearsideAWP();} // awp
    if(selector::auton == 2){nearsideRushSafe();} // rush safe
    if(selector::auton == 3){} // rush risky
    if(selector::auton == -1){} // awp
    if(selector::auton == -2){sixBallMidrush();} // rush safe
    if(selector::auton == -3){} // rush risky
    if(selector::auton == 0){skills();} // skills
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

	driveControlStarted = true;
	disabledMode = false;

	//Task brainScreen(screenDisplay1);
	Task controllerScreenTask(controllerScreen);
  	Task competitionTimerTask(competitionTimerStuff);

	if (competitionMode) {
		LV_IMG_DECLARE(BrainScreenIdle);
		lv_obj_t *img = lv_img_create(lv_scr_act(), NULL);
		lv_img_set_src(img, &BrainScreenIdle);
		lv_obj_align(img, NULL, LV_ALIGN_CENTER, 0, 0);
	}
  
    // reset old variables
	autoFireOn = false;
	blockerUp = false;
	autoReadyOn = true;

	// initialize new variables
 	float moveSpeed = .9;
  	float turnSpeed = .5;
	bool FLwingOut = false;
	bool FRwingOut = false;
	bool BwingsOut = false;
	
	bool cataMotorOn = false;
	// declare new variables to be used later for drivetrain code
	float move;
	float turn;
	float left;
	float right;

	// set drive motors to coast
	FL.set_brake_mode(E_MOTOR_BRAKE_COAST);
	FR.set_brake_mode(E_MOTOR_BRAKE_COAST);
	BL.set_brake_mode(E_MOTOR_BRAKE_COAST);
	BR.set_brake_mode(E_MOTOR_BRAKE_COAST);
	ML.set_brake_mode(E_MOTOR_BRAKE_COAST);
	MR.set_brake_mode(E_MOTOR_BRAKE_COAST);

	// set cata and intake motors to brake
	intake.set_brake_modes(E_MOTOR_BRAKE_BRAKE);

	/* BUTTON INPUT SYSTEM
	*/

	if (competitionMode && selector::auton == 0) {
		skillsStart();
	}

	while (true) {
		// ******************************************
		// CONTROLLER 1							   // 
		// ******************************************

		// toggle ____ with "A" button
		if (Controller1.get_digital_new_press(E_CONTROLLER_DIGITAL_A)){

		}

		// toggle ____ with "B" button
		if (Controller1.get_digital_new_press(E_CONTROLLER_DIGITAL_B)) {
			
		}

		// toggle ____ with "Y" button
		if (Controller1.get_digital_new_press(E_CONTROLLER_DIGITAL_Y)) {
			
		}

		// for testing (not for match use); run selected autonomous with "DOWN" button
		if (Controller1.get_digital_new_press(E_CONTROLLER_DIGITAL_DOWN) && !competitionMode) {
			autonomous();
		}

		// decrease cata speed with "LEFT" button
		if (Controller1.get_digital_new_press(E_CONTROLLER_DIGITAL_LEFT)) {
			
		}

		// increase cata speed with "RIGHT" button
		if (Controller1.get_digital_new_press(E_CONTROLLER_DIGITAL_RIGHT)) {
			
		}

		if (Controller1.get_digital_new_press(E_CONTROLLER_DIGITAL_UP) && !competitionMode) {
			chassis.setPose(0, 0, 0);
			chassis.moveTo(chassis.getPose().x, chassis.getPose().y + 20, chassis.getPose().theta, 1500);
		}

		// toggle front left wing with "L2" button
		if (Controller1.get_digital_new_press(E_CONTROLLER_DIGITAL_L2)){
			FLwingOut = !FLwingOut;
			FLwing.set_value(FLwingOut);
		}

		// toggle front right wing with "L1" button
		if (Controller1.get_digital_new_press(E_CONTROLLER_DIGITAL_L1)){
			FRwingOut = !FRwingOut;
			FRwing.set_value(FRwingOut);
		}

		// toggle back wings with "X" button
		if (Controller1.get_digital_new_press(E_CONTROLLER_DIGITAL_X)) {
			BwingsOut = !BwingsOut;
			BRwing.set_value(BwingsOut);
			BLwing.set_value(BwingsOut);
		}

		// intake if holding R1, outtake if holding R2
		if (Controller1.get_digital(E_CONTROLLER_DIGITAL_R2)){
			intake.move(-127);
		}
		else if (Controller1.get_digital(E_CONTROLLER_DIGITAL_R1)){
			intake.move(127);
		} 
		else {
			intake.move(0);
		}

		// double arcade drive controls - left joystick controls forward/backward movement, right joystick controls turning
		move = Controller1.get_analog(ANALOG_LEFT_Y);
		turn = Controller1.get_analog(ANALOG_RIGHT_X);
     
   		if (move < 50) {
			turn *= 1.3;
		}
    
		left = (move * moveSpeed + turn * turnSpeed);
		right = (move * moveSpeed - turn * turnSpeed);

    	// move drivetrain motors
		FL.move(left);
		ML.move(left);
		BL.move(left);
		FR.move(right);
		MR.move(right);
		BR.move(right);   

		// overheating warnings for all motors
		overheatWarning(FL);
        overheatWarning(ML);
        overheatWarning(BL);
        overheatWarning(FR);
        overheatWarning(MR);
        overheatWarning(BR);
        overheatWarning(INTR);
		overheatWarning(INTL);

		delay(10);
	}
}