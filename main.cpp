#include "main.h"
#include "lemlib/api.hpp"
#include <cmath>

/* ------------------------------------------------------------------------
 * DRIVETRAIN HARDWARE CONFIG
 * ------------------------------------------------------------------------
 * 4x 11W (V5 Smart) motors, all blue (600rpm) cartridges
 * 6 wheels total, 3 per side, all 3.25" diameter:
 *   [omni] -- [gear] -- [gear+motor] -- [gear] -- [traction] -- [gear+motor] -- [omni]
 * i.e. each side is a gear train: motor-driven gears sit inboard, driving
 * a center traction wheel (for grip/pivot) with idle omni wheels on
 * either end (for strafing during turns).
 *
 * !! UPDATE THESE TO MATCH YOUR ROBOT !!
 * - Port numbers below are placeholders, verify against your wiring
 * - Track width MUST be re-measured for this new arrangement (center of
 *   left wheels to center of right wheels, in inches) or your turns/
 *   odometry will be wrong
 * - IMPORTANT: because each side now drives through an idler gear
 *   between the motor and the traction wheel, the number of gear meshes
 *   can flip rotation direction compared to a direct-drive setup. Don't
 *   assume the old port signs below are still correct -- spin each
 *   motor individually on the DEVICES screen and confirm the whole
 *   group drives the robot FORWARD on positive voltage before trusting
 *   autonomous or driver control.
 * ------------------------------------------------------------------------ */

// Motor groups (negative port = reversed). Verify direction on your robot:
// spin each motor forward via the DEVICES screen and make sure the group
// as a whole drives the robot forward when given positive voltage.
// NOTE: re-check signs after the gear train change described above.
pros::MotorGroup left_motors({-1, 2}, pros::MotorGears::blue);   // port 2's sign flipped relative to before
pros::MotorGroup right_motors({3, -4}, pros::MotorGears::blue);  // port 4's sign flipped relative to before

// Inertial sensor
pros::Imu imu(10);

// Controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Drivetrain: left motors, right motors, track width (in), wheel type,
// drivetrain rpm (600 for blue cartridge direct drive), horizontal drift
// (higher since we have no tracking wheels to correct sideways slip)
lemlib::Drivetrain drivetrain(&left_motors,
                               &right_motors,
                               11.5,                          // TODO: RE-MEASURE for new wheel layout
                               lemlib::Omniwheel::NEW_325,
                               600,
                               8);

// No tracking wheels -> pass nullptr for all of them, just use the IMU
lemlib::OdomSensors sensors(nullptr, nullptr, nullptr, nullptr, &imu);

// Lateral (forward/backward) PID -- starter values, will need tuning
lemlib::ControllerSettings lateral_controller(10,    // kP
                                               0,     // kI
                                               3,     // kD
                                               3,     // anti windup
                                               1,     // small error range (in)
                                               100,   // small error range timeout (ms)
                                               3,     // large error range (in)
                                               500,   // large error range timeout (ms)
                                               20     // maximum acceleration (slew)
);

// Angular (turning) PID -- starter values, will need tuning
lemlib::ControllerSettings angular_controller(2,     // kP
                                               0,     // kI
                                               10,    // kD
                                               3,     // anti windup
                                               1,     // small error range (deg)
                                               100,   // small error range timeout (ms)
                                               3,     // large error range (deg)
                                               500,   // large error range timeout (ms)
                                               0      // maximum acceleration (0 = disabled)
);

// Chassis object, used for both autonomous (chassis.moveToPoint, etc.)
// and to read the IMU heading during driver control below
lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors);

/**
 * Runs initialization code. Calibrates the IMU -- keep the robot still
 * while this happens, it takes a couple of seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Calibrating IMU...");

	chassis.calibrate();

	pros::lcd::set_text(1, "Ready!");
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	// Example LemLib autonomous movement, replace with your own routine:
	// chassis.moveToPoint(0, 24, 4000);
	// chassis.turnToHeading(90, 2000);
}

/**
 * Driver control with IMU-based heading correction.
 *
 * Standard arcade drive, EXCEPT: whenever the driver isn't actively
 * turning (right stick roughly centered), the code locks onto whatever
 * heading the robot was at the moment turning stopped, and continuously
 * nudges the drivetrain to hold that heading. This cancels out drift
 * caused by uneven motor wear, wheel friction differences, or an
 * off-center traction wheel -- the classic "robot slowly curves even
 * though I'm pushing the stick straight" problem.
 *
 * As soon as the driver touches the turn stick again, correction is
 * disabled and full manual turning takes over immediately.
 */
void opcontrol() {
	const int TURN_DEADBAND = 5;    // ignore stick noise below this
	const int DRIVE_DEADBAND = 5;
	const double HEADING_KP = 1.4;  // tune this: higher = snappier correction,
	                                 // too high will cause oscillation/wobble

	double targetHeading = chassis.getPose().theta;
	bool wasTurning = true;  // start "true" so we don't lock heading before the driver moves

	while (true) {
		int driveInput = controller.get_analog(ANALOG_LEFT_Y);
		int turnInput = controller.get_analog(ANALOG_RIGHT_X);

		if (std::abs(driveInput) < DRIVE_DEADBAND) driveInput = 0;
		if (std::abs(turnInput) < TURN_DEADBAND) turnInput = 0;

		bool isTurning = (turnInput != 0);

		if (isTurning) {
			// driver is actively steering -- just pass their input straight through
			chassis.arcade(driveInput, turnInput);
			wasTurning = true;
		} else {
			// driver just let go of the turn stick -- lock in the current heading
			if (wasTurning) {
				targetHeading = chassis.getPose().theta;
				wasTurning = false;
			}

			double currentHeading = chassis.getPose().theta;
			double error = targetHeading - currentHeading;

			// normalize error to the range -180 to 180 so correction
			// always takes the shortest path back to target
			while (error > 180) error -= 360;
			while (error < -180) error += 360;

			double correction = HEADING_KP * error;

			chassis.arcade(driveInput, correction);
		}

		pros::delay(20);
	}
}