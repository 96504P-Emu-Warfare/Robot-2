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

Motor FL(-11, E_MOTOR_GEARSET_06);
Motor BL(-12, E_MOTOR_GEARSET_06);
Motor ML(-13, E_MOTOR_GEARSET_06);
Motor FR(1, E_MOTOR_GEARSET_06);
Motor BR(2, E_MOTOR_GEARSET_06);
Motor MR(3, E_MOTOR_GEARSET_06);

Motor INT(-15, E_MOTOR_GEARSET_06);
Motor CR(-18, E_MOTOR_GEARSET_18);
Motor CL(17, E_MOTOR_GEARSET_18);

Optical OPT(0);
Distance DIST(0);
Imu INR(5);
Rotation ROT(0);

MotorGroup leftMotors({BL, ML});
MotorGroup rightMotors({BR, MR});

ADIDigitalOut FLwing('A');
ADIDigitalOut FRwing('B');
ADIDigitalOut BLwing('C');
ADIDigitalOut BRwing('D');

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
	3.25, // wheel diameter
	360, // wheel rpm (gearing * motor rpm)
	2 // chasePower
};

// forward/backward PID
lemlib::ChassisController_t lateralController {
    65, // kP
    70, // kD
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    8 // slew rate
};
 
// turning PID
lemlib::ChassisController_t angularController {
    7, // kP
    48, // kD
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

void puncherMove(int power = 127) {
	CR.move(power);
	CL.move(power);
}

bool triballOnKicker() {
    return OPT.get_proximity() > 250;
}

bool cataInReadyPosition() {
	if (DIST.get() < 10) {return true;}
	return false;
}

// if toggled on, automatically fire detected triballs in puncher and reset
void autoPuncher() {
	while (true) {
		while (autoFireOn) {
			
			if (triballOnKicker() || !cataInReadyPosition()) {
				CR.move(127);
				while (triballOnKicker() || !cataInReadyPosition()) {
					delay(10);
				}
			}

			CR.move(0);
		}
		delay(10);
	}
}

// if toggled on, automatically lower/ready cata
void autoReady() {
	while (true) {
		if (autoReadyOn && !autoFireOn) {
			puncherMove(100);
			while (!cataInReadyPosition()) {delay(10);}
			puncherMove(0);
			autoReadyOn = false;
		}
		delay(10);
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
			autoReadyOn = true;
			autoFireOn = false;
			delay(500);
			autoReadyOn = false;
			CR.move(110);
			delay(300);
			CR.move(0);
			autoHangOn = !autoHangOn;
		}
		delay(100);
	}
}

void controllerScreen() {
	while (true) {
		Controller1.set_text(1,1, "CS:" + std::to_string(globalCataSpeed) + " AF:" + std::to_string(autoFireOn) + " L:" + std::to_string(autoReadyOn));
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

    // Extract RGB components of startColor
    uint8_t startR = (startColor >> 16) & 0xFF;
    uint8_t startG = (startColor >> 8) & 0xFF;
    uint8_t startB = startColor & 0xFF;

    // Extract RGB components of endColor
    uint8_t endR = (endColor >> 16) & 0xFF;
    uint8_t endG = (endColor >> 8) & 0xFF;
    uint8_t endB = endColor & 0xFF;

    // Calculate the step size for each color component
    double stepR = static_cast<double>(endR - startR) / (length - 1);
    double stepG = static_cast<double>(endG - startG) / (length - 1);
    double stepB = static_cast<double>(endB - startB) / (length - 1);

    // Generate the gradient
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

		delay(40);
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

			else if (std::abs(INT.get_actual_velocity()) > 100) {
				resting = false;
				flash(0x1F51FF, 1, 0xFFFFFF);
				while (std::abs(INT.get_actual_velocity()) > 100) {
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
	
	delay(10);
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

void nearsideRisky() {
	chassis.setPose(-44,-59, 135);
	chassis.moveTo(-56, -48, 135, 1000, false, false, 0, 0, 50);
	chassis.moveTo(-53, -53, 135, 1000, false, true, 0, 0, 50);
	FRwing.set_value(1);
	delay(200);
	chassis.turnTo(0, 0, 1000);
	chassis.turnTo(-60, 0, 2000);
	FRwing.set_value(0);
	delay(200);

	INT.move(-127); // outtake
	delay(300);
	chassis.turnTo(-60, -22, 1000, false, true, 50);
	chassis.moveTo(-60, -24, 180, 1000, false, false, 5.0);
	chassis.moveTo(-60, -48, 180, 1000);

	chassis.turnTo(-28, -28, 1000);
	chassis.moveTo(-28, -28, 45, 2000);

	INT.move(127); // intake
	chassis.moveTo(-28, -3, 0, 2000);
	chassis.moveTo(-28, -8, 0, 2000, false, false);
	chassis.turnTo(40, -8, 2000);

	INT.move(-127); // outtake
	FLwing.set_value(1);
	FRwing.set_value(1);
	chassis.moveTo(-8, -8, 90, 1000, false, true, 0, 0.6, 40);
	FRwing.set_value(0);
	chassis.moveTo(-8, -40, 180, 1500, false, true, 5.0);
	INT.move(0);
	chassis.turnTo(-23, -63, 1000, false, false, 20);
}

void nearsideSafe() {
	chassis.setPose(-46,-55, 135);
	chassis.moveTo(-56, -48, 145, 1000, false, false, 0, 0, 50);
	CR.move(0);
	autoFireOn = true;
	chassis.moveTo(-53, -53, 145, 1000, false, true, 0, 0, 50);
	FRwing.set_value(1);
	delay(200);
	chassis.turnTo(0, 0, 1000);
	chassis.turnTo(-60, -40, 2000, false, true);
	FRwing.set_value(0);
	delay(200);

	chassis.moveTo(-56, -47, 155, 2000, false, false, 0);
	chassis.moveTo(-60, -24, 180, 2000, false, false, 20);
	delay(200);
	INT.move(-127);
	chassis.moveTo(-34, -60, 90, 2000);
	chassis.moveTo(-12, -55, 90, 3000);
}

void nearsideRush() {
	chassis.setPose(-47, -53, 43);

	// hit alliance triball toward goal with right win
	FRwing.set_value(1);

	// grab central far triball, turn and score both central and central far
	INT.move(127);
	chassis.moveTo(-9, -9.5, 31, 200, false, true, 20);
	autoFireOn = true;
	FRwing.set_value(0);
	chassis.moveTo(-9, -9.5, 31, 2500, false, true, 35);
	delay(300);

	// back up with triball and move to match load bar
	driveMove(-70);
	delay(900);
	driveMove(0);
	chassis.moveTo(-50, -43, 30, 3000, false, false);

	// send match load and central triball down alley
	chassis.turnTo(-22, -81, 1000);
	FRwing.set_value(1);
	INT.move(-127);
	delay(300);
	chassis.turnTo(40, -30, 1000);
	FRwing.set_value(0);
	chassis.turnTo(-20, -56, 1000);
	chassis.moveTo(-12, -52, 90, 2500);
	driveMove(0);
	FLwing.set_value(1);
	delay(300);
	FLwing.set_value(0);
}

void fiveBallMidrush() {
	
}

void sixBallMidrush() {
	// release intake and set pose
	chassis.setPose(47, -53, 317);

	// hit alliance triball toward goal with right win
	FRwing.set_value(1);
	FLwing.set_value(1);

	// grab central far triball, turn and score both central and central far
	INT.move(127);
	chassis.moveTo(10, -6, 320, 200, false, true, 20);
	FRwing.set_value(0);
	FLwing.set_value(0);
	CR.move(0);
	autoFireOn = true;
	chassis.moveTo(10, -6, 320, 2400, false, true, 20);
	chassis.turnTo(40, -2, 1000);
	INT.move(-127);
	FLwing.set_value(1);
	FRwing.set_value(1);
	driveMove(110);
	delay(800);
	chassis.setPose(41, -2, 90);
	driveMove(-80);
	delay(300);
	driveMove(0);
	FLwing.set_value(0);
	FRwing.set_value(0);

	// grab central safe triball
	chassis.turnTo(14, -17, 1000);
	INT.move(127);
	chassis.moveTo(14, -17, 235, 1500);
	chassis.turnTo(59, -44, 700);
	INT.move(0);
	chassis.moveTo(59, -44, 125, 2500, false, true, 35);

	// go back and knock out matchload
	FRwing.set_value(1);
	INT.move(-127);
	chassis.turnTo(45, -24, 1000);
	FRwing.set_value(0);
	chassis.turnTo(6, -55, 1000);
	delay(300);

	// grab 6 ball
	INT.move(127);
	chassis.moveTo(10, -54, 270, 2500, false, true, 25);

	// move back
	chassis.moveTo(40, -54, 270, 2500, false, false, 25);
	chassis.turnTo(61, -28, 1000);
	INT.move(-127);
	chassis.moveTo(61, -30, 0, 900, false, true, 55, .2);
	INT.move(0);
	driveMove(110);
	delay(600);
	driveMove(-50);
	delay(400);
	driveMove(0);
}

void skills() {
	// initial setup and setting cata to global speed

	chassis.setPose(-49, -56, 225);

	// turn toward goal and fire for x seconds, lower cata 
	chassis.turnTo(46, -9, 1000, false, true);

	puncherMove(110);
	delay(500);
	autoFireOn = true;
	pros::delay(32000); // however long it takes to fire all triballs

	// turn autofire on to stop cata from hitting bar
	autoFireOn = false;

	// turn and move backwards to other side
	// pushing triballs along with robot
	INT.move(-127);
	chassis.turnTo(-19, -61, 1000, false, true);
	chassis.moveTo(-19, -61, 285, 1000, false, false);
	chassis.moveTo(39, -61, 270, 2000, false, false);

	// backwards push corner triballs into goal
	chassis.turnTo(61, -43, 1000, false, true);
	chassis.moveTo(61, -43, 230, 1500, false, false);
	chassis.turnTo(64, 0, 1000, false, true);
	driveMove(-90);
	delay(1500);
	chassis.setPose(61, -32, 180);
	driveMove(20);
	delay(750);
	driveMove(0);

	// move out of the corner and towards the middle,
	// deploying wings on way to collect triballs
	chassis.turnTo(11, -34, 1000);
	chassis.moveTo(11, -34, -75, 1500);
	chassis.moveTo(10, -21, 0, 1500);

	// first front push 
	chassis.turnTo(48, -10, 1000);
	FLwing.set_value(1);
	FRwing.set_value(1);
	// chassis.moveTo(40, -5, 90, 1500, false, true, 20);
	driveMove(100);
	delay(1000);
	chassis.setPose(40, -11, 90);
	driveMove(-20);
	delay(1000);
	driveMove(0);
	delay(100);
	FLwing.set_value(0);
	FRwing.set_value(0);

	// back up and second push
	chassis.moveTo(11, -11, 90, 1500, false, false);
	chassis.turnTo(11, 29, 1000);
	chassis.moveTo(11, 34, 0, 1500);
	chassis.turnTo(45, 0, 1000);
	FLwing.set_value(1);
	FRwing.set_value(1);
	driveMove(100);
	delay(1200);
	FLwing.set_value(0);
	FRwing.set_value(0);
	chassis.setPose(40, 11, 90);
	driveMove(-60);
	delay(700);
	driveMove(0);
	FLwing.set_value(1);
	FRwing.set_value(1);
	delay(100);
	driveMove(100);
	delay(700);
	FLwing.set_value(0);
	FRwing.set_value(0);
	driveMove(-40);
	delay(500);
	driveMove(0);


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
	//chassis.calibrate();
	lcd::initialize();
  	selector::init();
	OPT.set_led_pwm(30);
	autoFireOn = true;
	//Task brainScreen(screenDisplay1)
	//Task controllerScreenTask(controllerScreen);
	Task autoPuncherTask(autoPuncher);
	Task autoReadyTask(autoReady);
	Task ledUpdaterTask(ledUpdater);
	Task leds(LEDmainLoop);
	Task rgbcontrolTask(RGBcontrol);
	Task autoHangTask(autoHang);
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
	autoFireOn = false;
	puncherMove(80);

    if(selector::auton == 1){nearsideSafe();} // safe
    if(selector::auton == 2){nearsideRisky();} // risky
    if(selector::auton == 3){nearsideRush();} // rush
    if(selector::auton == -1){} // empty
    if(selector::auton == -2){fiveBallMidrush();} // 6 ball
    if(selector::auton == -3){sixBallMidrush();} // 5 ball
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
	bool BLwingOut = false;
	bool BRwingOut = false;
	
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
	CR.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	CL.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	INT.set_brake_mode(E_MOTOR_BRAKE_BRAKE);

	/* BUTTON INPUT SYSTEM
	*/

	while (true) {
		// ******************************************
		// CONTROLLER 1							   // 
		// ******************************************

		// toggle cata on/off with "A" button
		if (Controller1.get_digital_new_press(E_CONTROLLER_DIGITAL_A)){
			if (autoFireOn) {
				autoFireOn = false;
				cataMotorOn = false;
			}
			cataMotorOn = !cataMotorOn;
			if (cataMotorOn) {
				puncherMove(globalCataSpeed);
			}
			else {
				puncherMove(0);
			}

			BLwing.set_value(autoFireOn);
			BRwing.set_value(autoFireOn);
		}

		// toggle autoPuncher with "B" button
		if (Controller1.get_digital_new_press(E_CONTROLLER_DIGITAL_B)) {
			autoFireOn = !autoFireOn;
		}

		// toggle autoReadyOn with "Y" button
		if (Controller1.get_digital_new_press(E_CONTROLLER_DIGITAL_Y)) {
			autoFireOn = false;
			autoReadyOn = !autoReadyOn;
		}

		// for testing (not for match use); run selected autonomous with "DOWN" button
		if (Controller1.get_digital_new_press(E_CONTROLLER_DIGITAL_DOWN) && !competitionMode) {
			autonomous();
		}

		// decrease cata speed with "LEFT" button
		if (Controller1.get_digital_new_press(E_CONTROLLER_DIGITAL_LEFT)) {
			if (globalCataSpeed > 70) {globalCataSpeed -= 5;}
		}

		// increase cata speed with "RIGHT" button
		if (Controller1.get_digital_new_press(E_CONTROLLER_DIGITAL_RIGHT)) {
			globalCataSpeed += 5;
			if (globalCataSpeed < 125) {globalCataSpeed += 5;}
		}

		if (Controller1.get_digital_new_press(E_CONTROLLER_DIGITAL_UP)) {
			autoHangOn = true;
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
			BLwingOut = !BLwingOut;
			BRwingOut = !BRwingOut;

			BLwing.set_value(BLwingOut);
			BRwing.set_value(BRwingOut);
		}

		// intake if holding R1, outtake if holding R2
		if (Controller1.get_digital(E_CONTROLLER_DIGITAL_R2)){
			INT.move(-127);
		}
		else if (Controller1.get_digital(E_CONTROLLER_DIGITAL_R1)){
			INT.move(127);
		} 
		else {
			INT.move(0);
		}

		// Double arcade drive controls - left joystick controls forward/backward movement, right joystick controls turning
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
        overheatWarning(CR);
		overheatWarning(CR);
        overheatWarning(INT);

		delay(10);
	}
}