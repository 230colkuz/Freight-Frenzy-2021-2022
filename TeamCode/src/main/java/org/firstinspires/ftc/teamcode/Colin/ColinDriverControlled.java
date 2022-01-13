package org.firstinspires.ftc.teamcode.Colin;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.Robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "ColinDriverControlled", group = "Test")
public class ColinDriverControlled extends OpMode {

    // DEFINE robot
    ColinRobotHardware robot = new ColinRobotHardware();

    //variables

    // driving variables to ensure smooth transition to moving from parked
    boolean moving = false;
    long driveStartTime = 0;
    //TESTdouble PowerMultiplyer=0.5;

    /*
    // wobble arm boolean vars
    boolean goingDown = false;
    boolean goingUp = false;
    final int travelDistance = 440; // literally just a guess
    final int encoderTolerance = 5;

    boolean add = false;
    boolean subtract = false;
    double powerValue = .5;
    */

    // CONSTANTS

    // RUN ONCE ON init()
    @Override
    public void init() {
        robot.init(hardwareMap);
        telemetry.addData("STATUS", "Initialized");
    }

    //RUN ONCE ON start()
    @Override
    public void start() {

        robot.m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); //...ZeroPowerBehavior.FLOAT
        robot.m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.m3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.m4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.m5.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.m6.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


    }

    //LOOP ON start()
    @Override
    public void loop() {
        double G1rightStickY = -gamepad1.right_stick_y;
        double G1leftStickY = -gamepad1.left_stick_y;
        double G1rightStickX = gamepad1.right_stick_x;
        double G1leftStickX = gamepad1.left_stick_x;
        boolean G1a = gamepad1.a;
        boolean G1y = gamepad1.y;
        boolean G1x = gamepad1.x;
        boolean G1b = gamepad1.b;
        double G1LT = gamepad1.left_trigger;
        double G1RT = gamepad1.right_trigger;
        boolean G1dpad_right = gamepad1.dpad_right;
        boolean G1dpad_left = gamepad1.dpad_left;
        boolean G1dpad_up = gamepad1.dpad_up;
        boolean G1dpad_down = gamepad1.dpad_down;
        boolean G1rightBumper = gamepad1.right_bumper;
        boolean G1leftBumper = gamepad1.left_bumper;

        // check if we start moving
        if ((0 < Math.abs(G1leftStickY) || 0 < Math.abs(G1leftStickX) || 0 < Math.abs(G1rightStickX))) {
            if (!moving) {
                driveStartTime = System.currentTimeMillis(); // zero our time for exponential multiplier function
            }
            moving = true;
        } else {
            moving = false;

        }

        //mecanum drive
        robot.m1.setPower(((G1leftStickY) + (G1leftStickX) - (G1rightStickX)) * MultiplierFunction(driveStartTime));
        robot.m3.setPower(((G1leftStickY) - (G1leftStickX) - (G1rightStickX)) * MultiplierFunction(driveStartTime));
        robot.m2.setPower(((G1leftStickY) - (G1leftStickX) + (G1rightStickX)) * MultiplierFunction(driveStartTime));
        robot.m4.setPower(((G1leftStickY) + (G1leftStickX) + (G1rightStickX)) * MultiplierFunction(driveStartTime));

        //Carousel Wheel
        if (G1rightBumper == true) { //turn the wheel on
            robot.m6.setPower(0.3);
        } else { //carousel Wheel off
            robot.m6.setPower(0);
        }

        //Arm
        if (G1dpad_up) { //up
            robot.m5.setPower(-0.5);
        }
        else if (G1dpad_down) { //down
            robot.m5.setPower(0.5);
        }
        else { //stop the arm
            robot.m5.setPower(0);
        }

        //Plug servos into the expansion hub
        //Claw Wrist
        if (G1leftBumper) {
            robot.s1.setPosition(0.2);
        }

        if (G1LT>0) {
            robot.s1.setPosition(0.6);
        }

        //Claw pincher
        if (G1y) { //open
            robot.s2.setPosition(0);
        } else if (G1x) { //close
            robot.s2.setPosition(0.6);
        }

            /*// shooter and intake servo statements
            if (G1x) { // use shooting and intake clean up servo
                robot.servo1.setPosition(1.0);
            } else { // reset shooting and intake clean up servo
                robot.servo1.setPosition(0.0);
            }*/



    }

    // RUN ONCE ON stop()
    // multiplier function
    private double MultiplierFunction(double driveStartTime) {
        double quadraticConstantA = (3 * Math.pow(10, -6));
        double multiplier = quadraticConstantA * Math.pow(System.currentTimeMillis() - driveStartTime, 2) + .25; // exponential function

        // cap multiplier to 1
        if (multiplier > 1) {
            multiplier = 1;
        }

        return multiplier;
    }
}
