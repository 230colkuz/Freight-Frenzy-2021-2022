package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Colin.ColinRobotHardware;


@Autonomous(name="Smooth Speed and IMU Auton", group="Test")

public class SmoothSpeedAndIMUAuton extends LinearOpMode
{
    //Calls the RobotHardware class
    ColinRobotHardware r = new ColinRobotHardware(); //r for robot

    double power=0.6, globalAngle, correction, rotation;;
    BNO055IMU             imu;
    private ElapsedTime runtime = new ElapsedTime();
    Orientation lastAngles = new Orientation();

    public void StopDriving(){
        r.m1.setPower(0);
        r.m2.setPower(0);
        r.m3.setPower(0);
        r.m4.setPower(0);
        sleep(250);
    }

    //Method to print power & runtime to phone
    public void PrintPowerAndRuntime() {
        telemetry.addData("motor1 Power", r.m1.getPower());
        telemetry.addData("motor2 Power", r.m2.getPower());
        telemetry.addData("motor3 Power", r.m3.getPower());
        telemetry.addData("motor4 Power", r.m4.getPower());
        telemetry.addData("runtime", getRuntime());
        telemetry.update();
    }

    public void TurnRight(double turnPower) {
        r.m1.setPower(turnPower);
        r.m2.setPower(-turnPower);
        r.m3.setPower(turnPower);
        r.m4.setPower(-turnPower);
    }

    public void TurnLeft(double turnPower) {
        r.m1.setPower(-turnPower);
        r.m2.setPower(turnPower);
        r.m3.setPower(-turnPower);
        r.m4.setPower(turnPower);
    }

    //Acceleration Methods
    public void AccelerateForward(double travelTime) {

        if (opModeIsActive()) {
            //Resets the run time once the method has been called on
            runtime.reset();

            //ACCELERATE  When going for more than 2 seconds & accelerate to desired power
            while (opModeIsActive() && (runtime.seconds() < travelTime) && (travelTime > 2) && ((travelTime - runtime.seconds()) > 1)) {

                r.m1.setPower(power * (1 * runtime.seconds()));
                r.m2.setPower(power * (1 * runtime.seconds()));
                r.m3.setPower(power * (1 * runtime.seconds()));
                r.m4.setPower(power * (1 * runtime.seconds()));


                //Once the power has increased to the regular power, leave the power at that level
                if (Math.abs(power * 1 * runtime.seconds()) >= power) {
                    r.m1.setPower(power);
                    r.m2.setPower(power);
                    r.m3.setPower(power);
                    r.m4.setPower(power);

                    PrintPowerAndRuntime();
                }
            }

            //DECELERATE
            while (opModeIsActive() && (runtime.seconds() < travelTime) && (travelTime > 2) && ((travelTime - runtime.seconds()) < 1)) {

                r.m1.setPower(power * (travelTime - runtime.seconds()));
                r.m2.setPower(power * (travelTime - runtime.seconds()));
                r.m3.setPower(power * (travelTime - runtime.seconds()));
                r.m4.setPower(power * (travelTime - runtime.seconds()));

                PrintPowerAndRuntime();
            }

            //When We're not traveling for a long time, just set the motors to a low power
            while (opModeIsActive() && (runtime.seconds() < travelTime) && (travelTime <= 2)) {

                r.m1.setPower(0.2);
                r.m2.setPower(0.2);
                r.m3.setPower(0.2);
                r.m4.setPower(0.2);

                PrintPowerAndRuntime();

            }
            StopDriving();
        }
    }

    public void AccelerateBackwards(double travelTime) {

        if (opModeIsActive()) {
            //Resets the run time once the method has been called on
            runtime.reset();

            //ACCELERATE  When going for more than 2 seconds & accelerate to desired power
            while (opModeIsActive() && (runtime.seconds() < travelTime) && (travelTime > 2) && ((travelTime - runtime.seconds()) > 1)) {

                r.m1.setPower(-power * (1 * runtime.seconds()));
                r.m2.setPower(-power * (1 * runtime.seconds()));
                r.m3.setPower(-power * (1 * runtime.seconds()));
                r.m4.setPower(-power * (1 * runtime.seconds()));


                //Once the power has increased to the regular power, leave the power at that level
                if (Math.abs(power * 1 * runtime.seconds()) >= power) {
                    r.m1.setPower(-power);
                    r.m2.setPower(-power);
                    r.m3.setPower(-power);
                    r.m4.setPower(-power);

                    PrintPowerAndRuntime();
                }
            }

            //DECELERATE
            while (opModeIsActive() && (runtime.seconds() < travelTime) && (travelTime > 2) && ((travelTime - runtime.seconds()) < 1)) {

                r.m1.setPower(-power * (travelTime - runtime.seconds()));
                r.m2.setPower(-power * (travelTime - runtime.seconds()));
                r.m3.setPower(-power * (travelTime - runtime.seconds()));
                r.m4.setPower(-power * (travelTime - runtime.seconds()));

                PrintPowerAndRuntime();
            }

            //When We're not traveling for a long time, just set the motors to a low power
            while (opModeIsActive() && (runtime.seconds() < travelTime) && (travelTime <= 2)) {

                r.m1.setPower(-0.2);
                r.m2.setPower(-0.2);
                r.m3.setPower(-0.2);
                r.m4.setPower(-0.2);

                PrintPowerAndRuntime();

            }
            StopDriving();
        }
    }

    public void AccelerateRightStrafe(double travelTime) {
        if (opModeIsActive()) {
            //Resets the run time once the method has been called on
            runtime.reset();

            //ACCELERATE  When going for more than 2 seconds & accelerate to desired power
            while (opModeIsActive() && (runtime.seconds() < travelTime) && (travelTime > 2) && ((travelTime - runtime.seconds()) > 1)) {

                r.m1.setPower(power * (1 * runtime.seconds()));
                r.m2.setPower(-power * (1 * runtime.seconds()));
                r.m3.setPower(-power * (1 * runtime.seconds()));
                r.m4.setPower(power * (1 * runtime.seconds()));


                //Once the power has increased to the regular power, leave the power at that level
                if (Math.abs(power * 1 * runtime.seconds()) >= power) {
                    r.m1.setPower(power);
                    r.m2.setPower(-power);
                    r.m3.setPower(-power);
                    r.m4.setPower(power);

                    PrintPowerAndRuntime();
                }
            }

            //DECELERATE
            while (opModeIsActive() && (runtime.seconds() < travelTime) && (travelTime > 2) && ((travelTime - runtime.seconds()) < 1)) {

                r.m1.setPower(power * (travelTime - runtime.seconds()));
                r.m2.setPower(-power * (travelTime - runtime.seconds()));
                r.m3.setPower(-power * (travelTime - runtime.seconds()));
                r.m4.setPower(power * (travelTime - runtime.seconds()));

                PrintPowerAndRuntime();
            }

            //When We're not traveling for a long time, just set the motors to a low power
            while (opModeIsActive() && (runtime.seconds() < travelTime) && (travelTime <= 2)) {

                r.m1.setPower(0.4);
                r.m2.setPower(-0.4);
                r.m3.setPower(-0.4);
                r.m4.setPower(0.4);

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

                r.m1.setPower(-power * (1 * runtime.seconds()));
                r.m2.setPower(power * (1 * runtime.seconds()));
                r.m3.setPower(power * (1 * runtime.seconds()));
                r.m4.setPower(-power * (1 * runtime.seconds()));


                //Once the power has increased to the regular power, leave the power at that level
                if (Math.abs(power * 1 * runtime.seconds()) >= power) {
                    r.m1.setPower(-power);
                    r.m2.setPower(power);
                    r.m3.setPower(power);
                    r.m4.setPower(-power);

                    PrintPowerAndRuntime();
                }
            }

            //DECELERATE
            while (opModeIsActive() && (runtime.seconds() < travelTime) && (travelTime > 2) && ((travelTime - runtime.seconds()) < 1)) {

                r.m1.setPower(-power * (travelTime - runtime.seconds()));
                r.m2.setPower(power * (travelTime - runtime.seconds()));
                r.m3.setPower(power * (travelTime - runtime.seconds()));
                r.m4.setPower(-power * (travelTime - runtime.seconds()));

                PrintPowerAndRuntime();
            }

            //When We're not traveling for a long time, just set the motors to a low power
            while (opModeIsActive() && (runtime.seconds() < travelTime) && (travelTime <= 2)) {

                r.m1.setPower(-0.4);
                r.m2.setPower(0.4);
                r.m3.setPower(0.4);
                r.m4.setPower(-0.4);

                PrintPowerAndRuntime();

            }

            StopDriving();
        }
    }

    // Resets the cumulative angle tracking to zero.
    private void resetAngle() {
        //ZYX
        lastAngles = r.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    //Get current cumulative angle rotation from last reset.
    //return Angle in degrees. + = left, - = right.
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = r.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

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

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .05;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees)
    {

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating clockwise (right).
        // rotate until turn is completed.

        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {
                TurnRight(.2);
            }

            while (opModeIsActive() && getAngle() > degrees) {
                TurnRight(.2);
            }
            while (opModeIsActive() && getAngle() < degrees) {
                TurnLeft(.2);
            }
        }
        else {   // left turn.
            while (opModeIsActive() && getAngle() < degrees) {
                TurnLeft(.2);
                PrintPowerAndRuntime();
            }
            while (opModeIsActive() && getAngle() > degrees) {
                TurnRight(.2);
            }
        }

        StopDriving();
        rotation = getAngle();

        // wait for rotation to stop.
        sleep(500);
    }


    @Override
    public void runOpMode() throws InterruptedException {

        r.init(hardwareMap);

        //Makes new methods for naming simplification purposes
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = true;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        //Gives info about what the IMU is doing on the phone
        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // When the stop button isn't pushed and the gyro (IMU) isn't calibrated, wait (! means not). This is a loop.
        while (!isStopRequested() && !r.imu.isGyroCalibrated())
        {
            //do nothing for 50 milliseconds
            sleep(50);
            //idle(); allows the program to perform other necessary tasks in between iterations of the loop.
            idle();
        }
        //Once the past loop finishes and the IMU is calibrated, the rest of the code continues.
        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", r.imu.getCalibrationStatus().toString());
        telemetry.update();

        waitForStart();


        while (opModeIsActive()) {

            correction = checkDirection();
            //Displays the realtime heading information on the phone.
            telemetry.addData("1 imu heading", lastAngles.firstAngle);
            telemetry.addData("2 global heading", globalAngle);
            telemetry.addData("3 correction", correction);
            telemetry.addData("4 turn rotation", rotation);
            telemetry.update();

            rotate(90);

        }
    }
}
