package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Colin.ColinRobotHardware;

@Autonomous(name="Red2(Park_in_warehouse)", group="Test")
public class Blue_1_Carousel_PkInWh extends LinearOpMode {

    //Calls the RobotHardware class
    ColinRobotHardware r = new ColinRobotHardware();
    private ElapsedTime runtime = new ElapsedTime();

    BNO055IMU             imu;
    Orientation           lastAngles = new Orientation();
    double                globalAngle, lastTargetAngle, power = .60, correction, rotation;

    //Declares some methods to compress and reduce tediousness of writing repetitive code.

    public void GetTelemetry()
    {
        telemetry.addData("1 imu heading", lastAngles.firstAngle);
        telemetry.addData("2 global heading", globalAngle);
        telemetry.addData("3 correction", correction);
        telemetry.addData("4 turn rotation", rotation);
        telemetry.addData("runtime", getRuntime());
        telemetry.update();
    }

    //Basic Methods
    public void StopDriving(){
        r.m1.setPower(0);
        r.m2.setPower(0);
        r.m3.setPower(0);
        r.m4.setPower(0);
        sleep(250);
    }
    public void DriveForward(long time, double otherPower) {
        r.m1.setPower(otherPower);
        r.m2.setPower(otherPower);
        r.m3.setPower(otherPower);
        r.m4.setPower(otherPower);
        sleep(time);
        StopDriving();
    }
    public void DriveBackwards(long time, double otherPower) {
        r.m1.setPower(-otherPower);
        r.m2.setPower(-otherPower);
        r.m3.setPower(-otherPower);
        r.m4.setPower(-otherPower);
        sleep(time);
        StopDriving();
    }
    public void StrafeRight(long time, double otherPower){
        r.m1.setPower(otherPower);
        r.m2.setPower(-otherPower);
        r.m3.setPower(-otherPower);
        r.m4.setPower(otherPower);
        sleep(time);
        StopDriving();
    }
    public void StrafeLeft(long time, double otherPower){
        r.m1.setPower(-otherPower);
        r.m2.setPower(otherPower);
        r.m3.setPower(otherPower);
        r.m4.setPower(-otherPower);
        sleep(time);
        StopDriving();
    }

    //Acceleration Methods
    public void AccelerateForward(double travelTime) {

        //Resets the run time once the method has been called on
        runtime.reset();

        //ACCELERATE  When going for more than 1.5 seconds & accelerate to desired power
        while (opModeIsActive() && (runtime.seconds() < travelTime) && (travelTime > 1.5) && ((travelTime - runtime.seconds()) > 1)) {

            //correction value is constantly updated in this while loop
            correction = checkDirection();
            r.m1.setPower(power * (1.5 * runtime.seconds()) + correction);
            r.m2.setPower(power * (1.5 * runtime.seconds()) - correction);
            r.m3.setPower(power * (1.5 * runtime.seconds()) + correction);
            r.m4.setPower(power * (1.5 * runtime.seconds()) - correction);

            GetTelemetry();

            //Once the power has increased to the regular power, leave the power at that level
            if (Math.abs(power * 1.5 * runtime.seconds()) >= power) {
                r.m1.setPower(power);
                r.m2.setPower(power);
                r.m3.setPower(power);
                r.m4.setPower(power);

            }
        }

        //DECELERATE
        while (opModeIsActive() && (runtime.seconds() < travelTime) && (travelTime > 1.5) && ((travelTime - runtime.seconds()) < 1)) {

            //correction value is constantly updated in this while loop
            correction = checkDirection();
            r.m1.setPower(power*(travelTime - runtime.seconds()) + correction);
            r.m2.setPower(power*(travelTime - runtime.seconds()) - correction);
            r.m3.setPower(power*(travelTime - runtime.seconds()) + correction);
            r.m4.setPower(power*(travelTime - runtime.seconds()) - correction);

            GetTelemetry();
        }

        //When We're not traveling for a long time, just set the motors to a low power
        while (opModeIsActive() && (runtime.seconds() < travelTime) && (travelTime <= 1.5)) {

            correction = checkDirection();
            r.m1.setPower(0.2 + correction);
            r.m2.setPower(0.2 - correction);
            r.m3.setPower(0.2 + correction);
            r.m4.setPower(0.2 - correction);

            GetTelemetry();

        }
        StopDriving();
        sleep(250);
    }

    public void AccelerateBackwards(double travelTime) {

        //Resets the run time once the method has been called on
        runtime.reset();

        //ACCELERATE  When going for more than 2 seconds & accelerate to desired power
        while (opModeIsActive() && (runtime.seconds() < travelTime) && (travelTime > 1.5) && ((travelTime - runtime.seconds()) > 1)) {
            //correction value is constantly updated in this while loop
            correction = checkDirection();
            r.m1.setPower(-power * (1.5 * runtime.seconds()) + correction);
            r.m2.setPower(-power * (1.5 * runtime.seconds()) - correction);
            r.m3.setPower(-power * (1.5 * runtime.seconds()) + correction);
            r.m4.setPower(-power * (1.5 * runtime.seconds()) - correction);

            GetTelemetry();

            //Once the power has increased to/reached to the regular power, cap the power at that level
            if (Math.abs(power * 1.5 * runtime.seconds()) >= power) {
                r.m1.setPower(-power);
                r.m2.setPower(-power);
                r.m3.setPower(-power);
                r.m4.setPower(-power);
            }
        }

        //DECELERATE
        while (opModeIsActive() && (runtime.seconds()< travelTime) && (travelTime > 1.5) && ((travelTime - runtime.seconds()) < 1)) {

            correction = checkDirection();
            r.m1.setPower(-power*(travelTime - runtime.seconds()) + correction);
            r.m2.setPower(-power*(travelTime - runtime.seconds()) - correction);
            r.m3.setPower(-power*(travelTime - runtime.seconds()) + correction);
            r.m4.setPower(-power*(travelTime - runtime.seconds()) - correction);

            GetTelemetry();
        }

        //When We're not traveling for a long time, just set the motors to a low power
        while (opModeIsActive() && (runtime.seconds() < travelTime) && (travelTime <= 1.5)) {

            correction = checkDirection();
            r.m1.setPower(-0.2 + correction);
            r.m2.setPower(-0.2 - correction);
            r.m3.setPower(-0.2 + correction);
            r.m4.setPower(-0.2 - correction);

            GetTelemetry();

        }

        StopDriving();
        sleep (250);
    }

    public void AccelerateStrafeRight(double travelTime) {

        //Resets the run time once the method has been called on
        runtime.reset();

        //ACCELERATE  When going for more than 2 seconds & accelerate to desired power
        while (opModeIsActive() && (runtime.seconds() < travelTime) && (travelTime > 1.5) && ((travelTime - runtime.seconds()) > 1)) {

            //correction value is constantly updated in this while loop
            correction = checkDirection();
            r.m1.setPower(power * (1.5 * runtime.seconds()) + correction);
            r.m2.setPower(-power * (1.5 * runtime.seconds()) - correction);
            r.m3.setPower(-power * (1.5 * runtime.seconds()) + correction);
            r.m4.setPower(power * (1.5 * runtime.seconds()) - correction);

            GetTelemetry();

            //Once the power has increased to the regular power, leave the power at that level
            if (Math.abs(power * 1.5 * runtime.seconds()) >= power) {

                r.m1.setPower(-power + correction);
                r.m2.setPower(-power - correction);
                r.m3.setPower(-power + correction);
                r.m4.setPower(-power - correction);
            }
        }

        //DECELERATE
        while (opModeIsActive() && (runtime.seconds()< travelTime) && (travelTime > 1.5) && ((travelTime - runtime.seconds()) < 1)) {

            //correction value is constantly updated in this while loop
            correction = checkDirection();
            r.m1.setPower(power*(travelTime - runtime.seconds()) + correction);
            r.m2.setPower(-power*(travelTime - runtime.seconds()) - correction);
            r.m3.setPower(-power*(travelTime - runtime.seconds()) + correction);
            r.m4.setPower(power*(travelTime - runtime.seconds()) - correction);

            GetTelemetry();
        }

        //When We're not traveling for a long time, just set the motors to a low power
        while (opModeIsActive() && (runtime.seconds() < travelTime) && (travelTime <= 1.5)) {

            correction = checkDirection();
            r.m1.setPower(0.2 + correction);
            r.m2.setPower(-0.2 - correction);
            r.m3.setPower(-0.2 + correction);
            r.m4.setPower(0.2 - correction);

            GetTelemetry();

        }
        StopDriving();
        sleep(250);
    }

    public void AccelerateStrafeLeft(double travelTime) {

        //Resets the run time once the method has been called on
        runtime.reset();

        //ACCELERATE  When going for more than 2 seconds & accelerate to desired power
        while (opModeIsActive() && (runtime.seconds() < travelTime) && (travelTime > 1.5) && ((travelTime - runtime.seconds()) > 1)) {

            //correction value is constantly updated in this while loop
            correction = checkDirection();
            r.m1.setPower(-power * (1.5 * runtime.seconds()) + correction);
            r.m2.setPower(power * (1.5 * runtime.seconds()) - correction);
            r.m3.setPower(power * (1.5 * runtime.seconds()) + correction);
            r.m4.setPower(-power * (1.5 * runtime.seconds()) - correction);

            GetTelemetry();
            // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
            // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
            // and named "imu".

            //Once the power has increased to the regular power, leave the power at that level
            if (Math.abs(power * 1.5 * runtime.seconds()) >= power) {
                r.m1.setPower(-power);
                r.m2.setPower(power);
                r.m3.setPower(power);
                r.m4.setPower(-power);

            }
        }

        //Once the past loop finishes and the IMU is calibrated, the rest of the code continues.
        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calibration status", r.imu.getCalibrationStatus().toString());
        telemetry.update();

        //DECELERATE
        while (opModeIsActive() && (runtime.seconds()< travelTime) && (travelTime > 1.5) && ((travelTime - runtime.seconds()) < 1)) {

            //correction value is constantly updated in this while loop
            correction = checkDirection();
            r.m1.setPower(-power*(travelTime - runtime.seconds()) + correction);
            r.m2.setPower(power*(travelTime - runtime.seconds()) - correction);
            r.m3.setPower(power*(travelTime - runtime.seconds()) + correction);
            r.m4.setPower(-power*(travelTime - runtime.seconds()) - correction);

            GetTelemetry();
        }

        //When We're not traveling for a long time, just set the motors to a low power
        while (opModeIsActive() && (runtime.seconds() < travelTime) && (travelTime <= 1.5)) {

            correction = checkDirection();
            r.m1.setPower(-0.2 + correction);
            r.m2.setPower(0.2 - correction);
            r.m3.setPower(0.2 + correction);
            r.m4.setPower(-0.2 - correction);

            GetTelemetry();

        }
        StopDriving();
        sleep(250);
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

        if (angle == lastTargetAngle)
            correction = 0;             // no adjustment.
        else
            correction = (lastTargetAngle - angle);        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    private double rotate(int degrees, double travelTime) {

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
                    r.m1.setPower(-Math.abs(0.00555 * (degrees - getAngle())));
                    r.m2.setPower(Math.abs(0.00555 * (degrees - getAngle())));
                    r.m3.setPower(-Math.abs(0.00555 * (degrees - getAngle())));
                    r.m4.setPower(Math.abs(0.00555 * (degrees - getAngle())));

                    //else, keep it at this power so the robot has enough power to finish its rotation
                    if (Math.abs(r.m1.getPower()) < minTurnPower && Math.abs(r.m4.getPower()) < minTurnPower) {
                        r.m1.setPower(-minTurnPower);
                        r.m2.setPower(minTurnPower);
                        r.m3.setPower(-minTurnPower);
                        r.m4.setPower(minTurnPower);
                    }
                }
                while (opModeIsActive() && getAngle() < degrees) {
                    r.m1.setPower(Math.abs(0.00555 * (degrees - getAngle())));
                    r.m2.setPower(-Math.abs(0.00555 * (degrees - getAngle())));
                    r.m3.setPower(Math.abs(0.00555 * (degrees - getAngle())));
                    r.m4.setPower(-Math.abs(0.00555 * (degrees - getAngle())));

                    if (Math.abs(r.m1.getPower()) < minTurnPower && Math.abs(r.m4.getPower()) < minTurnPower) {
                        r.m1.setPower(minTurnPower);
                        r.m2.setPower(-minTurnPower);
                        r.m3.setPower(minTurnPower);
                        r.m4.setPower(-minTurnPower);
                    }
                }
            }
            else {   // left turn.
                while (opModeIsActive() && getAngle() < degrees) {
                    r.m1.setPower(Math.abs(0.00555 * (degrees - getAngle())));
                    r.m2.setPower(-Math.abs(0.00555 * (degrees - getAngle())));
                    r.m3.setPower(Math.abs(0.00555 * (degrees - getAngle())));
                    r.m4.setPower(-Math.abs(0.00555 * (degrees - getAngle())));

                    if (Math.abs(r.m1.getPower()) < minTurnPower && Math.abs(r.m4.getPower()) < minTurnPower) {
                        r.m1.setPower(minTurnPower);
                        r.m2.setPower(-minTurnPower);
                        r.m3.setPower(minTurnPower);
                        r.m4.setPower(-minTurnPower);
                    }
                }
                while (opModeIsActive() && getAngle() > degrees) {

                    r.m1.setPower(-Math.abs(0.00555 * (degrees - getAngle())));
                    r.m2.setPower(Math.abs(0.00555 * (degrees - getAngle())));
                    r.m3.setPower(-Math.abs(0.00555 * (degrees - getAngle())));
                    r.m4.setPower(Math.abs(0.00555 * (degrees - getAngle())));

                    if (Math.abs(r.m1.getPower()) < minTurnPower && Math.abs(r.m4.getPower()) < minTurnPower) {
                        r.m1.setPower(-minTurnPower);
                        r.m2.setPower(minTurnPower);
                        r.m3.setPower(-minTurnPower);
                        r.m4.setPower(minTurnPower);
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
        //putting all my IMU stuff here, recreating it basically
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        //Makes new methods for naming simplification purposes
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = true;

        imu.initialize(parameters);

        //Gives info about what the IMU is doing on the phone
        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // When the stop button isn't pushed and the gyro (IMU) isn't calibrated, wait (! means not). This is a loop.
        while (!isStopRequested() && !r.imu.isGyroCalibrated()) {
            //do nothing for 50 milliseconds
            sleep(50);
            //idle(); allows the program to perform other necessary tasks in between iterations of the loop.
            idle();
        }
        //Once the past loop finishes and the IMU is calibrated, the rest of the code continues.
        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", r.imu.getCalibrationStatus().toString());
        telemetry.update();

        // The program will wait for the start button to continue.
        waitForStart();

        //(the last direction we rotated to)
        lastTargetAngle=0;



        r.m1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT); //...ZeroPowerBehavior.FLOAT
        r.m2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        r.m3.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        r.m4.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        AccelerateStrafeLeft(1.4);

        r.m1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE); //...ZeroPowerBehavior.FLOAT
        r.m2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        r.m3.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        r.m4.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        DriveBackwards(500,0.1);

        //correct for if we bounced
        StrafeLeft(400,0.1);

        //turn the carousel
        r.m6.setPower(0.3);
        sleep(4000);

        AccelerateForward(0.75);

        AccelerateStrafeRight(2.3);

        AccelerateBackwards(0.75);

        rotate(90,2);

        //Park!
        AccelerateBackwards(0.5);
        stop();









    }
}

