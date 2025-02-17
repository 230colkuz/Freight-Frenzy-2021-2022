//This is where the autonomous instructions are stored

package org.firstinspires.ftc.teamcode.Colin;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Colin.ColinRobotHardware;

@Autonomous(name="The other one", group="Test")
public class Hardware_and_Auton extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    BNO055IMU             imu;
    Orientation           lastAngles = new Orientation();
    double                globalAngle, lastTargetAngle, power = .60, correction, rotation;

    //  INSTANTIATE MOTORS AND SERVOS
    public DcMotorEx m1;
    public DcMotorEx m2;
    public DcMotorEx m3;
    public DcMotorEx m4;
    public DcMotorEx m5;
    public DcMotorEx m6;
    /*public DcMotor motor7;
     DcMotor motor8;*/
    public Servo s1;
    public Servo s2;


    //INSTANTIATE SENSORS

    /*//public GyroSensor gyroSensor;
    //public ColorSensor colorSensor;
    public DcMotor LeftEncoder, RightEncoder, BackEncoder;*/

    //CREATE THE HARDWARE MAP
    HardwareMap hardwareMap;

    public void init(HardwareMap hardwareMap) {

        // DEFINE MOTORS AND SERVOS
        m1 = hardwareMap.get(DcMotorEx.class, "motor1"); // back right
        m2 = hardwareMap.get(DcMotorEx.class, "motor2"); // back left
        m3 = hardwareMap.get(DcMotorEx.class, "motor3"); // front right
        m4 = hardwareMap.get(DcMotorEx.class, "motor4"); // front left
        m5 = hardwareMap.get(DcMotorEx.class, "motor5"); // arm motor
        m6 = hardwareMap.get(DcMotorEx.class, "motor6"); // Carousel motor
        s1 = hardwareMap.get(Servo.class, "servo1");   // wrist
        s2 = hardwareMap.get(Servo.class, "servo2");   //claw

        //DEFINE SENSORS
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        //SET MOTOR POWERS
        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);
        m5.setPower(0);
        m6.setPower(0);


        //SET SERVO POSITION
        s1.setPosition(0);
        s2.setPosition(0); //0 is claw open, 0.6 is claw closed

        //SET MOTOR zeroPowerBehavior (Brake for the Wheels and Float for anything else
        m1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE); //...ZeroPowerBehavior.FLOAT
        m2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        m3.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        m4.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        m5.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        m6.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);



        //SET MOTOR DIRECTIONS
        m1.setDirection(DcMotorEx.Direction.FORWARD); //motor1.setDirection(DcMotor.Direction.REVERSE)
        m2.setDirection(DcMotorEx.Direction.REVERSE); //motor2.setDirection(DcMotor.Direction.FORWARD)
        m3.setDirection(DcMotorEx.Direction.FORWARD); //motor3.setDirection(DcMotor.Direction.REVERSE)
        m4.setDirection(DcMotorEx.Direction.REVERSE); //motor4.setDirection(DcMotor.Direction.FORWARD)
        m5.setDirection(DcMotorEx.Direction.REVERSE); //motor5.setDirection(DcMotor.Direction.REVERSE)
        m6.setDirection(DcMotorEx.Direction.FORWARD); //motor6.setDirection(DcMotor.Direction.FORWARD)




        //CALIBRATE SENSORS
        //gyroSensor.calibrate();

    }

    //Basic Methods
    public void StopDriving(){
        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);
        sleep(250);
    }
    public void DriveForward(long time) {
        m1.setPower(power);
        m2.setPower(power);
        m3.setPower(power);
        m4.setPower(power);
        sleep(time);
        StopDriving();
    }
    public void DriveBackwards(long time) {
        m1.setPower(-power);
        m2.setPower(-power);
        m3.setPower(-power);
        m4.setPower(-power);
        sleep(time);
        StopDriving();
    }
    public void StrafeRight(long time){
        m1.setPower(power);
        m2.setPower(-power);
        m3.setPower(-power);
        m4.setPower(power);
        sleep(time);
        StopDriving();
    }
    public void StrafeLeft(long time){
        m1.setPower(-power);
        m2.setPower(power);
        m3.setPower(power);
        m4.setPower(-power);
        sleep(time);
        StopDriving();
    }
    public void TurnRight(double turnPower, long time){
        m1.setPower(turnPower);
        m2.setPower(-turnPower);
        m3.setPower(turnPower);
        m4.setPower(-turnPower);
        sleep(time);
        StopDriving();
    }
    public void TurnLeft(double turnPower, long time){
        m1.setPower(-turnPower);
        m2.setPower(turnPower);
        m3.setPower(-turnPower);
        m4.setPower(turnPower);
        sleep(time);
        StopDriving();
    }

    //Method to print power & runtime to phone
    public void PrintPowerAndRuntime() {
        telemetry.addData("motor1 Power", m1.getPower());
        telemetry.addData("motor2 Power", m2.getPower());
        telemetry.addData("motor3 Power", m3.getPower());
        telemetry.addData("motor4 Power", m4.getPower());
        telemetry.addData("runtime", getRuntime());
        telemetry.update();
    }

    //Acceleration Methods
    public void AccelerateForward(double travelTime) {

        if (opModeIsActive()) ;
        //Resets the run time once the method has been called on
        runtime.reset();

        //ACCELERATE  When going for more than 2 seconds & accelerate to desired power
        while (opModeIsActive() && (runtime.seconds() < travelTime) && (travelTime > 2) && ((travelTime - runtime.seconds()) > 1)) {

            m1.setPower(power * (1 * runtime.seconds()));
            m2.setPower(power * (1 * runtime.seconds()));
            m3.setPower(power * (1 * runtime.seconds()));
            m4.setPower(power * (1 * runtime.seconds()));


            //Once the power has increased to the regular power, leave the power at that level
            if (Math.abs(power * 1 * runtime.seconds()) >= power) {
                m1.setPower(power);
                m2.setPower(power);
                m3.setPower(power);
                m4.setPower(power);

                PrintPowerAndRuntime();
            }
        }

        //DECELERATE
        while (opModeIsActive() && (runtime.seconds()< travelTime) && (travelTime > 2) && ((travelTime - runtime.seconds()) < 1)) {

            m1.setPower(power*(travelTime - runtime.seconds()));
            m2.setPower(power*(travelTime - runtime.seconds()));
            m3.setPower(power*(travelTime - runtime.seconds()));
            m4.setPower(power*(travelTime - runtime.seconds()));

            PrintPowerAndRuntime();
        }

        //When We're not traveling for a long time, just set the motors to a low power
        while (opModeIsActive() && (runtime.seconds() < travelTime) && (travelTime <= 2)) {

            m1.setPower(0.2);
            m2.setPower(0.2);
            m3.setPower(0.2);
            m4.setPower(0.2);

            PrintPowerAndRuntime();

        }

        StopDriving();

    }

    public void AccelerateBackwards(double travelTime) {

        if (opModeIsActive()) ;
        //Resets the run time once the method has been called on
        runtime.reset();

        //ACCELERATE  When going for more than 2 seconds & accelerate to desired power
        while (opModeIsActive() && (runtime.seconds() < travelTime) && (travelTime > 2) && ((travelTime - runtime.seconds()) > 1)) {

            m1.setPower(-power * (1 * runtime.seconds()));
            m2.setPower(-power * (1 * runtime.seconds()));
            m3.setPower(-power * (1 * runtime.seconds()));
            m4.setPower(-power * (1 * runtime.seconds()));


            //Once the power has increased to the regular power, leave the power at that level
            if (Math.abs(power * 1 * runtime.seconds()) >= power) {
                m1.setPower(-power);
                m2.setPower(-power);
                m3.setPower(-power);
                m4.setPower(-power);

                PrintPowerAndRuntime();
            }
        }

        //DECELERATE
        while (opModeIsActive() && (runtime.seconds()< travelTime) && (travelTime > 2) && ((travelTime - runtime.seconds()) < 1)) {

            m1.setPower(-power*(travelTime - runtime.seconds()));
            m2.setPower(-power*(travelTime - runtime.seconds()));
            m3.setPower(-power*(travelTime - runtime.seconds()));
            m4.setPower(-power*(travelTime - runtime.seconds()));

            PrintPowerAndRuntime();
        }

        //When We're not traveling for a long time, just set the motors to a low power
        while (opModeIsActive() && (runtime.seconds() < travelTime) && (travelTime <= 2)) {

            m1.setPower(-0.2);
            m2.setPower(-0.2);
            m3.setPower(-0.2);
            m4.setPower(-0.2);

            PrintPowerAndRuntime();

        }

        StopDriving();

    }

    public void AccelerateRightStrafe(double travelTime){
        if (opModeIsActive()) {
            //Resets the run time once the method has been called on
            runtime.reset();

            //ACCELERATE  When going for more than 2 seconds & accelerate to desired power
            while (opModeIsActive() && (runtime.seconds() < travelTime) && (travelTime > 2) && ((travelTime - runtime.seconds()) > 1)) {

                m1.setPower(power * (1 * runtime.seconds()));
                m2.setPower(-power * (1 * runtime.seconds()));
                m3.setPower(-power * (1 * runtime.seconds()));
                m4.setPower(power * (1 * runtime.seconds()));


                //Once the power has increased to the regular power, leave the power at that level
                if (Math.abs(power * 1 * runtime.seconds()) >= power) {
                    m1.setPower(power);
                    m2.setPower(-power);
                    m3.setPower(-power);
                    m4.setPower(power);

                    PrintPowerAndRuntime();
                }
            }

            //DECELERATE
            while (opModeIsActive() && (runtime.seconds() < travelTime) && (travelTime > 2) && ((travelTime - runtime.seconds()) < 1)) {

                m1.setPower(power * (travelTime - runtime.seconds()));
                m2.setPower(-power * (travelTime - runtime.seconds()));
                m3.setPower(-power * (travelTime - runtime.seconds()));
                m4.setPower(power * (travelTime - runtime.seconds()));

                PrintPowerAndRuntime();
            }

            //When We're not traveling for a long time, just set the motors to a low power
            while (opModeIsActive() && (runtime.seconds() < travelTime) && (travelTime <= 2)) {

                m1.setPower(0.4);
                m2.setPower(-0.4);
                m3.setPower(-0.4);
                m4.setPower(0.4);

                PrintPowerAndRuntime();

            }

            StopDriving();
        }
    }

    public void AccelerateLeftStrafe(double travelTime) { // reverse motors 2 and 3
        if (opModeIsActive()) {
            //Resets the run time once the method has been called on
            runtime.reset();

            //ACCELERATE  When going for more than 2 seconds & accelerate to desired power
            while (opModeIsActive() && (runtime.seconds() < travelTime) && (travelTime > 2) && ((travelTime - runtime.seconds()) > 1)) {

                m1.setPower(-power * (1 * runtime.seconds()));
                m2.setPower(power * (1 * runtime.seconds()));
                m3.setPower(power * (1 * runtime.seconds()));
                m4.setPower(-power * (1 * runtime.seconds()));


                //Once the power has increased to the regular power, leave the power at that level
                if (Math.abs(power * 1 * runtime.seconds()) >= power) {
                    m1.setPower(-power);
                    m2.setPower(power);
                    m3.setPower(power);
                    m4.setPower(-power);

                    PrintPowerAndRuntime();
                }
            }

            //DECELERATE
            while (opModeIsActive() && (runtime.seconds() < travelTime) && (travelTime > 2) && ((travelTime - runtime.seconds()) < 1)) {

                m1.setPower(-power * (travelTime - runtime.seconds()));
                m2.setPower(power * (travelTime - runtime.seconds()));
                m3.setPower(power * (travelTime - runtime.seconds()));
                m4.setPower(-power * (travelTime - runtime.seconds()));

                PrintPowerAndRuntime();
            }

            //When We're not traveling for a long time, just set the motors to a low power
            while (opModeIsActive() && (runtime.seconds() < travelTime) && (travelTime <= 2)) {

                m1.setPower(-0.4);
                m2.setPower(0.4);
                m3.setPower(0.4);
                m4.setPower(-0.4);

                PrintPowerAndRuntime();

            }

            StopDriving();
        }
    }

/**
 *IMU stuff that I did not write ¯\_(ツ)_/¯ (mostly)
 *This green Stuff is cool I stole it from the example code, but Now I HAVE THE POWWWEEER!!!
 */
    //Okay... I made these public instead of private, hopefully it shouldn't break anything
    public double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        //The change (delta) angle is whatever the angle was first minus our current angle.
        //lastAngles is new Orientation() as declared near the beginning.
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        //If the angle goes below -180, start subtracting the change in the angle (so -180 is the greatest change in the angle, -360 would be 0).
        //Returns a positive value (left) if the robot turns more than 180 deg right.
        if (deltaAngle < -180)
            deltaAngle += 360;
            //If the angle goes above 180, start subtracting the change in the angle (so 180 is the greatest change in the angle, 360 would be 0).
            //Returns a negative value (right) if the robot turns more than 180 deg left.
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        //Add the deltaAngle to the globalAngle (which was set to 0 at the start, but will change as the robot moves).
        globalAngle += deltaAngle;

        //Sets the current lastAngles to angles, which is different from what lastAngles was to start.
        lastAngles = angles;

        return globalAngle;
    }

    public double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.

        //Okay so this weird "correction", is not the same as the other created version of "correction",
        // it's just a number, which defaults to zero, when you call this method. Ignore it. I don't want to mess with it. it works
        double correction, angle, gain = .05;

        angle = getAngle();

        if (angle == lastTargetAngle)
            correction = 0;             // no adjustment.
        else
            correction = (lastTargetAngle - angle);        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    public void resetAngle() {
        //ZYX
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    //I don't know what this is
    public void GetTelemetry() {
        telemetry.addData("1 imu heading", lastAngles.firstAngle);
        telemetry.addData("2 global heading", globalAngle);
        telemetry.addData("3 correction", correction);
        telemetry.addData("4 turn rotation", rotation);
        telemetry.addData("runtime", getRuntime());
        telemetry.update();
    }

    public double rotate(int degrees, double travelTime) {

        if (opModeIsActive()) ;
        //Resets the run time once the method has been called on
        runtime.reset();
        double minTurnPower = .133;

        while (opModeIsActive() && (runtime.seconds() < travelTime)) {
            // getAngle() returns + when rotating counter clockwise (left) and - when rotating clockwise (right).
            // rotate until turn is completed.
            GetTelemetry();
            if (degrees < 0) {
                // On right turn we have to get off zero first.
                while (opModeIsActive() && (getAngle() > degrees || getAngle() == 0)) {
                    //do this as long as our power will be above the amount specified
                    //sets power as a relationship of the difference between our target angle and current angle
                    m1.setPower(-Math.abs(0.00555 * (degrees - getAngle())));
                    m2.setPower(Math.abs(0.00555 * (degrees - getAngle())));
                    m3.setPower(-Math.abs(0.00555 * (degrees - getAngle())));
                    m4.setPower(Math.abs(0.00555 * (degrees - getAngle())));

                    //else, keep it at this power so the robot has enough power to finish its rotation
                    if (Math.abs(m1.getPower()) < minTurnPower && Math.abs(m4.getPower()) < minTurnPower) {
                        m1.setPower(-minTurnPower);
                        m2.setPower(minTurnPower);
                        m3.setPower(-minTurnPower);
                        m4.setPower(minTurnPower);
                    }
                }
                while (opModeIsActive() && getAngle() < degrees) {
                    m1.setPower(Math.abs(0.00555 * (degrees - getAngle())));
                    m2.setPower(-Math.abs(0.00555 * (degrees - getAngle())));
                    m3.setPower(Math.abs(0.00555 * (degrees - getAngle())));
                    m4.setPower(-Math.abs(0.00555 * (degrees - getAngle())));

                    if (Math.abs(m1.getPower()) < minTurnPower && Math.abs(m4.getPower()) < minTurnPower) {
                        m1.setPower(minTurnPower);
                        m2.setPower(-minTurnPower);
                        m3.setPower(minTurnPower);
                        m4.setPower(-minTurnPower);
                    }
                }
            }
            else {   // left turn.
                while (opModeIsActive() && getAngle() < degrees) {
                    m1.setPower(Math.abs(0.00555 * (degrees - getAngle())));
                    m2.setPower(-Math.abs(0.00555 * (degrees - getAngle())));
                    m3.setPower(Math.abs(0.00555 * (degrees - getAngle())));
                    m4.setPower(-Math.abs(0.00555 * (degrees - getAngle())));

                    if (Math.abs(m1.getPower()) < minTurnPower && Math.abs(m4.getPower()) < minTurnPower) {
                        m1.setPower(minTurnPower);
                        m2.setPower(-minTurnPower);
                        m3.setPower(minTurnPower);
                        m4.setPower(-minTurnPower);
                    }
                }
                while (opModeIsActive() && getAngle() > degrees) {

                    m1.setPower(-Math.abs(0.00555 * (degrees - getAngle())));
                    m2.setPower(Math.abs(0.00555 * (degrees - getAngle())));
                    m3.setPower(-Math.abs(0.00555 * (degrees - getAngle())));
                    m4.setPower(Math.abs(0.00555 * (degrees - getAngle())));

                    if (Math.abs(m1.getPower()) < minTurnPower && Math.abs(m4.getPower()) < minTurnPower) {
                        m1.setPower(-minTurnPower);
                        m2.setPower(minTurnPower);
                        m3.setPower(-minTurnPower);
                        m4.setPower(minTurnPower);
                    }
                }
            }


            rotation = getAngle();

            // wait for rotation to stop.
            sleep(100);
        }
        // turn the motors off.
        StopDriving();
        sleep(250);
        lastTargetAngle = degrees;
        return degrees;
    }


    @Override
    public void runOpMode() {

        init(hardwareMap);

        //Makes new methods for naming simplification purposes
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = true;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        //Once the past loop finishes and the IMU is calibrated, the rest of the code continues.
        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        waitForStart();

    }
}
