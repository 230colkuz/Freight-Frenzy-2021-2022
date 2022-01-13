// Simple autonomous program that uses an IMU to drive in a straight line.
// Uses REV Hub's built in IMU in place of a gyro.

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Colin.ColinRobotHardware;

@Autonomous(name="EncoderOpMode", group="Testing")
public class EncoderOpMode extends LinearOpMode
{
    //Calls the RobotHardware class
    ColinRobotHardware r = new ColinRobotHardware();
    private ElapsedTime runtime = new ElapsedTime();

    BNO055IMU             imu;
    Orientation           lastAngles = new Orientation();
    double                globalAngle, lastTargetAngle, velocity = 60, correction, rotation;

    //Declares some methods to compress and reduce tediousness of writing repetitive code.

    //Stops all 4 motors
    public void StopDriving()
    {
        r.m1.setVelocity(0);
        r.m2.setVelocity(0);
        r.m3.setVelocity(0);
        r.m4.setVelocity(0);
    }
    public void GetTelemetry()
    {
        telemetry.addData("1 imu heading", lastAngles.firstAngle);
        telemetry.addData("2 global heading", globalAngle);
        telemetry.addData("3 correction", correction);
        telemetry.addData("4 turn rotation", rotation);
        telemetry.addData("runtime", getRuntime());
        telemetry.update();
    }

    //Acceleration Methods
    public void AccelerateForward(double travelTime) {

        //Resets the run time once the method has been called on
        runtime.reset();

        //ACCELERATE  When going for more than 2 seconds & accelerate to desired velocity
        while (opModeIsActive() && (runtime.seconds() < travelTime) && (travelTime > 1.5) && ((travelTime - runtime.seconds()) > 1)) {

            //correction value is constantly updated in this while loop
            correction = checkDirection();
            r.m1.setVelocity(velocity * (1.5 * runtime.seconds()) + correction);
            r.m2.setVelocity(velocity * (1.5 * runtime.seconds()) - correction);
            r.m3.setVelocity(velocity * (1.5 * runtime.seconds()) + correction);
            r.m4.setVelocity(velocity * (1.5 * runtime.seconds()) - correction);

            GetTelemetry();

            //Once the velocity has increased to the regular velocity, leave the velocity at that level
            if (Math.abs(velocity * 1.5 * runtime.seconds()) >= velocity) {
                r.m1.setVelocity(velocity);
                r.m2.setVelocity(velocity);
                r.m3.setVelocity(velocity);
                r.m4.setVelocity(velocity);

            }
        }

        //DECELERATE
        while (opModeIsActive() && (runtime.seconds()< travelTime) && (travelTime > 1.5) && ((travelTime - runtime.seconds()) < 1)) {

            //correction value is constantly updated in this while loop
            correction = checkDirection();
            r.m1.setVelocity(velocity*(travelTime - runtime.seconds()) + correction);
            r.m2.setVelocity(velocity*(travelTime - runtime.seconds()) - correction);
            r.m3.setVelocity(velocity*(travelTime - runtime.seconds()) + correction);
            r.m4.setVelocity(velocity*(travelTime - runtime.seconds()) - correction);

            GetTelemetry();
        }

        //When We're not traveling for a long time, just set the motors to a low velocity
        while (opModeIsActive() && (runtime.seconds() < travelTime) && (travelTime <= 1.5)) {

            correction = checkDirection();
            r.m1.setVelocity(20 + correction);
            r.m2.setVelocity(20 - correction);
            r.m3.setVelocity(20 + correction);
            r.m4.setVelocity(20 - correction);

            GetTelemetry();

        }
        StopDriving();
        sleep(250);
    }


    public void AccelerateBackwards(double travelTime) {

        //Resets the run time once the method has been called on
        runtime.reset();

        //ACCELERATE  When going for more than 2 seconds & accelerate to desired velocity
            while (opModeIsActive() && (runtime.seconds() < travelTime) && (travelTime > 1.5) && ((travelTime - runtime.seconds()) > 1)) {
                //correction value is constantly updated in this while loop
                correction = checkDirection();
                r.m1.setVelocity(-velocity * (1.5 * runtime.seconds()) + correction);
                r.m2.setVelocity(-velocity * (1.5 * runtime.seconds()) - correction);
                r.m3.setVelocity(-velocity * (1.5 * runtime.seconds()) + correction);
                r.m4.setVelocity(-velocity * (1.5 * runtime.seconds()) - correction);

                GetTelemetry();

                //Once the velocity has increased to/reached to the regular velocity, cap the velocity at that level
                if (Math.abs(velocity * 1.5 * runtime.seconds()) >= velocity) {
                    r.m1.setVelocity(-velocity);
                    r.m2.setVelocity(-velocity);
                    r.m3.setVelocity(-velocity);
                    r.m4.setVelocity(-velocity);
                }
            }

        //DECELERATE
        while (opModeIsActive() && (runtime.seconds()< travelTime) && (travelTime > 1.5) && ((travelTime - runtime.seconds()) < 1)) {

            correction = checkDirection();
            r.m1.setVelocity(-velocity*(travelTime - runtime.seconds()) + correction);
            r.m2.setVelocity(-velocity*(travelTime - runtime.seconds()) - correction);
            r.m3.setVelocity(-velocity*(travelTime - runtime.seconds()) + correction);
            r.m4.setVelocity(-velocity*(travelTime - runtime.seconds()) - correction);

            GetTelemetry();
        }

        //When We're not traveling for a long time, just set the motors to a low velocity
        while (opModeIsActive() && (runtime.seconds() < travelTime) && (travelTime <= 1.5)) {

            correction = checkDirection();
            r.m1.setVelocity(-20 + correction);
            r.m2.setVelocity(-20 - correction);
            r.m3.setVelocity(-20 + correction);
            r.m4.setVelocity(-20 - correction);

            GetTelemetry();

        }

        StopDriving();
        sleep (250);
    }

    public void AccelerateStrafeRight(double travelTime) {

        //Resets the run time once the method has been called on
        runtime.reset();

        //ACCELERATE  When going for more than 2 seconds & accelerate to desired velocity
        while (opModeIsActive() && (runtime.seconds() < travelTime) && (travelTime > 1.5) && ((travelTime - runtime.seconds()) > 1)) {

            //correction value is constantly updated in this while loop
            correction = checkDirection();
            r.m1.setVelocity(velocity * (1.5 * runtime.seconds()) + correction);
            r.m2.setVelocity(-velocity * (1.5 * runtime.seconds()) - correction);
            r.m3.setVelocity(-velocity * (1.5 * runtime.seconds()) + correction);
            r.m4.setVelocity(velocity * (1.5 * runtime.seconds()) - correction);

            GetTelemetry();

            //Once the velocity has increased to the regular velocity, leave the velocity at that level
            if (Math.abs(velocity * 1.5 * runtime.seconds()) >= velocity) {

                r.m1.setVelocity(-velocity + correction);
                r.m2.setVelocity(-velocity - correction);
                r.m3.setVelocity(-velocity + correction);
                r.m4.setVelocity(-velocity - correction);
            }
        }

        //DECELERATE
        while (opModeIsActive() && (runtime.seconds()< travelTime) && (travelTime > 1.5) && ((travelTime - runtime.seconds()) < 1)) {

            //correction value is constantly updated in this while loop
            correction = checkDirection();
            r.m1.setVelocity(velocity*(travelTime - runtime.seconds()) + correction);
            r.m2.setVelocity(-velocity*(travelTime - runtime.seconds()) - correction);
            r.m3.setVelocity(-velocity*(travelTime - runtime.seconds()) + correction);
            r.m4.setVelocity(velocity*(travelTime - runtime.seconds()) - correction);

            GetTelemetry();
        }

        //When We're not traveling for a long time, just set the motors to a low velocity
        while (opModeIsActive() && (runtime.seconds() < travelTime) && (travelTime <= 1.5)) {

            correction = checkDirection();
            r.m1.setVelocity(20 + correction);
            r.m2.setVelocity(-20 - correction);
            r.m3.setVelocity(-20 + correction);
            r.m4.setVelocity(20 - correction);

            GetTelemetry();

        }
        StopDriving();
        sleep(250);
    }

    public void AccelerateStrafeLeft(double travelTime) {

        //Resets the run time once the method has been called on
        runtime.reset();

        //ACCELERATE  When going for more than 2 seconds & accelerate to desired velocity
        while (opModeIsActive() && (runtime.seconds() < travelTime) && (travelTime > 1.5) && ((travelTime - runtime.seconds()) > 1)) {

            //correction value is constantly updated in this while loop
            correction = checkDirection();
            r.m1.setVelocity(-velocity * (1.5 * runtime.seconds()) + correction);
            r.m2.setVelocity(velocity * (1.5 * runtime.seconds()) - correction);
            r.m3.setVelocity(velocity * (1.5 * runtime.seconds()) + correction);
            r.m4.setVelocity(-velocity * (1.5 * runtime.seconds()) - correction);

            GetTelemetry();
        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".

            //Once the velocity has increased to the regular velocity, leave the velocity at that level
            if (Math.abs(velocity * 1.5 * runtime.seconds()) >= velocity) {
                r.m1.setVelocity(-velocity);
                r.m2.setVelocity(velocity);
                r.m3.setVelocity(velocity);
                r.m4.setVelocity(-velocity);

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
            r.m1.setVelocity(-velocity*(travelTime - runtime.seconds()) + correction);
            r.m2.setVelocity(velocity*(travelTime - runtime.seconds()) - correction);
            r.m3.setVelocity(velocity*(travelTime - runtime.seconds()) + correction);
            r.m4.setVelocity(-velocity*(travelTime - runtime.seconds()) - correction);

            GetTelemetry();
        }

        //When We're not traveling for a long time, just set the motors to a low velocity
        while (opModeIsActive() && (runtime.seconds() < travelTime) && (travelTime <= 1.5)) {

            correction = checkDirection();
            r.m1.setVelocity(-20 + correction);
            r.m2.setVelocity(20 - correction);
            r.m3.setVelocity(20 + correction);
            r.m4.setVelocity(-20 - correction);

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
     * See if we are moving in a straight line and if not return a velocity correction value.
     * @return velocity adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = 5;

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

        if (opModeIsActive()) ;
            //Resets the run time once the method has been called on
            runtime.reset();
            double minTurnVelocity = 13.3;

            while (opModeIsActive() && (runtime.seconds() < travelTime)) {
                // getAngle() returns + when rotating counter clockwise (left) and - when rotating clockwise (right).
                // rotate until turn is completed.
                GetTelemetry();
                if (degrees < 0) {
                    // On right turn we have to get off zero first.
                    while (opModeIsActive() && (getAngle() > degrees || getAngle() == 0)) {
                        //do this as long as our velocity will be above the amount specified
                        //sets velocity as a relationship of the difference between our target angle and current angle
                        r.m1.setVelocity(-Math.abs(0.555 * (degrees - getAngle())));
                        r.m2.setVelocity(Math.abs(0.555 * (degrees - getAngle())));
                        r.m3.setVelocity(-Math.abs(0.555 * (degrees - getAngle())));
                        r.m4.setVelocity(Math.abs(0.555 * (degrees - getAngle())));

                        //else, keep it at this velocity so the robot has enough velocity to finish its rotation
                        if (Math.abs(r.m1.getVelocity()) < minTurnVelocity && Math.abs(r.m4.getVelocity()) < minTurnVelocity) {
                            r.m1.setVelocity(-minTurnVelocity);
                            r.m2.setVelocity(minTurnVelocity);
                            r.m3.setVelocity(-minTurnVelocity);
                            r.m4.setVelocity(minTurnVelocity);
                        }
                    }
                    while (opModeIsActive() && getAngle() < degrees) {
                        r.m1.setVelocity(Math.abs(0.555 * (degrees - getAngle())));
                        r.m2.setVelocity(-Math.abs(0.555 * (degrees - getAngle())));
                        r.m3.setVelocity(Math.abs(0.555 * (degrees - getAngle())));
                        r.m4.setVelocity(-Math.abs(0.555 * (degrees - getAngle())));

                        if (Math.abs(r.m1.getVelocity()) < minTurnVelocity && Math.abs(r.m4.getVelocity()) < minTurnVelocity) {
                            r.m1.setVelocity(minTurnVelocity);
                            r.m2.setVelocity(-minTurnVelocity);
                            r.m3.setVelocity(minTurnVelocity);
                            r.m4.setVelocity(-minTurnVelocity);
                        }
                    }
                }
                else {   // left turn.
                    while (opModeIsActive() && getAngle() < degrees) {
                        r.m1.setVelocity(Math.abs(0.555 * (degrees - getAngle())));
                        r.m2.setVelocity(-Math.abs(0.555 * (degrees - getAngle())));
                        r.m3.setVelocity(Math.abs(0.555 * (degrees - getAngle())));
                        r.m4.setVelocity(-Math.abs(0.555 * (degrees - getAngle())));

                        if (Math.abs(r.m1.getVelocity()) < minTurnVelocity && Math.abs(r.m4.getVelocity()) < minTurnVelocity) {
                            r.m1.setVelocity(minTurnVelocity);
                            r.m2.setVelocity(-minTurnVelocity);
                            r.m3.setVelocity(minTurnVelocity);
                            r.m4.setVelocity(-minTurnVelocity);
                        }
                    }
                    while (opModeIsActive() && getAngle() > degrees) {

                        r.m1.setVelocity(-Math.abs(0.555 * (degrees - getAngle())));
                        r.m2.setVelocity(Math.abs(0.555 * (degrees - getAngle())));
                        r.m3.setVelocity(-Math.abs(0.555 * (degrees - getAngle())));
                        r.m4.setVelocity(Math.abs(0.555 * (degrees - getAngle())));

                        if (Math.abs(r.m1.getVelocity()) < minTurnVelocity && Math.abs(r.m4.getVelocity()) < minTurnVelocity) {
                            r.m1.setVelocity(-minTurnVelocity);
                            r.m2.setVelocity(minTurnVelocity);
                            r.m3.setVelocity(-minTurnVelocity);
                            r.m4.setVelocity(minTurnVelocity);
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

    //This is what happens when the init button is pushed.
    @Override
    public void runOpMode() throws InterruptedException {
        // Initializes hardware when init is pressed on the phone
        r.init(hardwareMap);

        r.m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        r.m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        r.m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        r.m4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        r.m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        r.m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        r.m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        r.m4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                r.m1.getCurrentPosition(),
                r.m2.getCurrentPosition(),
                r.m3.getCurrentPosition(),
                r.m4.getCurrentPosition());
        telemetry.update();

        // The program will wait for the start button to continue.
        waitForStart();

            //last heading (the last direction we rotated to) will update after every turn, but since we haven't turned at start, default to 0 degrees
            lastTargetAngle=0;

            AccelerateForward(1.6);
            //rotate(-90, 5);
            AccelerateBackwards(1.6);
            AccelerateStrafeRight(1.6);
            AccelerateStrafeLeft(1.6);
    }


}