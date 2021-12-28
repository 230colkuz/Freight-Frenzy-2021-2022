// Simple autonomous program that uses an IMU to drive in a straight line.
// Uses REV Hub's built in IMU in place of a gyro.

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Colin.ColinRobotHardware;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

@Autonomous(name="IMU Test", group="Testing")
public class IMUTest extends LinearOpMode
{
    //Calls the RobotHardware class
    ColinRobotHardware robot = new ColinRobotHardware();
    private ElapsedTime runtime = new ElapsedTime();

    BNO055IMU             imu;
    Orientation           lastAngles = new Orientation();
    double                globalAngle, power = .60, correction, rotation;
    double newPower;
/*  double targetPower = power; //power, as defined earlier in the code, will now be named targetPower
    double currentPower;*/

    //Declares some methods to compress and reduce tediousness of writing repetitive code.
    //Every time the method DriveForward() is called,  it will do the instructions within the method
    public void DriveForward()
    {
        robot.m1.setPower(newPower + correction); //sets the motors power
        robot.m2.setPower(newPower - correction);
        robot.m3.setPower(newPower + correction);
        robot.m4.setPower(newPower - correction);
    }
    //Same as DriveForward() but in reverse
    public void DriveBackward()
    {
        robot.m1.setPower(-power + correction);
        robot.m2.setPower(-power - correction);
        robot.m3.setPower(-power + correction);
        robot.m4.setPower(-power - correction);
    }
    public void StrafeLeft()
    {
        robot.m1.setPower(-power + correction);
        robot.m2.setPower(power - correction);
        robot.m3.setPower(power + correction);
        robot.m4.setPower(-power - correction);
    }
    public void StrafeRight()
    {
        robot.m1.setPower(power + correction);
        robot.m2.setPower(-power - correction);
        robot.m3.setPower(-power + correction);
        robot.m4.setPower(power - correction);
    }
    public void TurnLeft()
    {
        robot.m1.setPower(.2);
        robot.m2.setPower(-.2);
        robot.m3.setPower(.2);
        robot.m4.setPower(-.2);
    }
    public void TurnRight()
    {
        robot.m1.setPower(-.2);
        robot.m2.setPower(.2);
        robot.m3.setPower(-.2);
        robot.m4.setPower(.2);
    }
    //Stops all 4 motors
    public void StopDriving()
    {
        robot.m1.setPower(0);
        robot.m2.setPower(0);
        robot.m3.setPower(0);
        robot.m4.setPower(0);
    }

    /*public void CalculatePower()
    {
        if (targetPower > currentPower) {
            newPower = currentPower + .5*Math.pow(runtime.seconds(),2); //The main equation: adds a power-curve to whatever the current power was
        }
        else if (targetPower < currentPower) {
            newPower = currentPower + -.5*Math.pow(runtime.seconds(),2); //The main equation: adds a power-curve to whatever the current power was
        }
        else {
            newPower = targetPower;
        }

        if (newPower > targetPower && targetPower > currentPower) {
            newPower = targetPower; //caps the power added by the graph to the power we set
        }

        if (newPower < targetPower && targetPower < currentPower) {
            newPower = targetPower; //caps the power added by the graph to the power we set
        }
    }*/

    //This is what happens when the init button is pushed.
    @Override
    public void runOpMode() throws InterruptedException
    {
        // Initializes hardware when init is pressed on the phone
        robot.init(hardwareMap);

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
        while (!isStopRequested() && !robot.imu.isGyroCalibrated())
        {
            //do nothing for 50 milliseconds
            sleep(50);
            //idle(); allows the program to perform other necessary tasks in between iterations of the loop.
            idle();
        }
        //Once the past loop finishes and the IMU is calibrated, the rest of the code continues.
        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", robot.imu.getCalibrationStatus().toString());
        telemetry.update();

        // The program will wait for the start button to continue.
        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        sleep(1000);


        //captures System.currentTimeMillis and saves it as startTime. Subtract the later time from this time to get the change in time.
        boolean started = false;
        long startTime = 0;
        long elapsedTime;


        //Proceeds with the following code as long as the mode is active (returns false when stop button is pushed or power is disconnected).
        while (opModeIsActive())
        {
            // simple switch that sets the time as soon as op mode is started (activates only once and at the beginning of the loop)
            if (!started) {
                started = true;
                startTime = System.currentTimeMillis();
            }

            elapsedTime = System.currentTimeMillis() - startTime;
            correction = checkDirection();
            //Displays the realtime heading information on the phone.
            telemetry.addData("1 imu heading", lastAngles.firstAngle);
            telemetry.addData("2 global heading", globalAngle);
            telemetry.addData("3 correction", correction);
            telemetry.addData("4 turn rotation", rotation);
            telemetry.update();

            //The series of instructions the robot will do.
            //Drive Forward
            if(elapsedTime > 0 && elapsedTime < 2000) {
                /*while (opModeIsActive()) {
                    currentPower = robot.m1.getPower();
                    CalculatePower();
                }*/
                DriveBackward();
            }
            //Stop Driving
            else if(elapsedTime > 2000 && elapsedTime < 2500) {
                StopDriving();
            }
            //Rotate Left
            else if(elapsedTime > 2500 && elapsedTime < 7500) {
                rotate(90);
            }
            //Stop Driving
            else if(elapsedTime > 7500 && elapsedTime < 8000) {
                StopDriving();
            }
            //Rotate Left
            else if(elapsedTime > 7500 && elapsedTime < 12500) {
                rotate(-90);
            }

        }
        //Turn the motors off (this will happen once when "While opModeIsActive" loop is finished).
        StopDriving();
    }

    // Resets the cumulative angle tracking to zero.
    private void resetAngle() {
        //ZYX
        lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
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

        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

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
                TurnRight();
            }

            while (opModeIsActive() && getAngle() > degrees) {
                TurnRight();
            }
            while (opModeIsActive() && getAngle() < degrees) {
                TurnLeft();
            }
        }
        else {   // left turn.
            while (opModeIsActive() && getAngle() < degrees) {
                TurnLeft();
            }
            while (opModeIsActive() && getAngle() > degrees) {
                TurnRight();
            }
        }
        /*if (getAngle() == degrees) {
            resetAngle();
            while(opModeIsActive() && degrees > 0) {
                robot.m1.setPower(0 - 5*correction);
                robot.m2.setPower(0 - 5*correction);
                robot.m3.setPower(0 - 5*correction);
                robot.m4.setPower(0 - 5*correction);
            }
            while(opModeIsActive() && degrees < 0) {
                robot.m1.setPower(0 + 5*correction);
                robot.m2.setPower(0 + 5*correction);
                robot.m3.setPower(0 + 5*correction);
                robot.m4.setPower(0 + 5*correction);
            }
        }*/

    // turn the motors off.
    StopDriving();

    rotation = getAngle();

    // wait for rotation to stop.
    sleep(500);

    }
}