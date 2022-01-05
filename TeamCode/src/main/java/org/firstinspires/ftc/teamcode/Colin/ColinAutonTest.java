package org.firstinspires.ftc.teamcode.Colin;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="ColinAutonTest", group="Test")

public class ColinAutonTest extends LinearOpMode {

    //Calls the RobotHardware class
    ColinRobotHardware r = new ColinRobotHardware(); //r for robot

    double power=0.6;
    private ElapsedTime runtime = new ElapsedTime();

    //Basic Methods
    public void StopDriving(){
        r.m1.setPower(0);
        r.m2.setPower(0);
        r.m3.setPower(0);
        r.m4.setPower(0);
        sleep(250);
    }
    public void DriveForward(long time) {
        r.m1.setPower(power);
        r.m2.setPower(power);
        r.m3.setPower(power);
        r.m4.setPower(power);
        sleep(time);
        StopDriving();
    }
    public void DriveBackwards(long time) {
        r.m1.setPower(-power);
        r.m2.setPower(-power);
        r.m3.setPower(-power);
        r.m4.setPower(-power);
        sleep(time);
        StopDriving();
    }
    public void StrafeRight(long time){
        r.m1.setPower(power);
        r.m2.setPower(-power);
        r.m3.setPower(-power);
        r.m4.setPower(power);
        sleep(time);
        StopDriving();
    }
    public void StrafeLeft(long time){
        r.m1.setPower(-power);
        r.m2.setPower(power);
        r.m3.setPower(power);
        r.m4.setPower(-power);
        sleep(time);
        StopDriving();
    }

    public void TurnRight(double turnPower, long time){
        r.m1.setPower(turnPower);
        r.m2.setPower(-turnPower);
        r.m3.setPower(turnPower);
        r.m4.setPower(-turnPower);
        sleep(time);
        StopDriving();
    }

    public void TurnLeft(double turnPower, long time){
        r.m1.setPower(-turnPower);
        r.m2.setPower(turnPower);
        r.m3.setPower(-turnPower);
        r.m4.setPower(turnPower);
        sleep(time);
        StopDriving();
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

    //Acceleration Methods
    public void AccelerateForward(double travelTime) {

        if (opModeIsActive()) ;
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
        while (opModeIsActive() && (runtime.seconds()< travelTime) && (travelTime > 2) && ((travelTime - runtime.seconds()) < 1)) {

            r.m1.setPower(power*(travelTime - runtime.seconds()));
            r.m2.setPower(power*(travelTime - runtime.seconds()));
            r.m3.setPower(power*(travelTime - runtime.seconds()));
            r.m4.setPower(power*(travelTime - runtime.seconds()));

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

    public void AccelerateBackwards(double travelTime) {

        if (opModeIsActive()) ;
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
        while (opModeIsActive() && (runtime.seconds()< travelTime) && (travelTime > 2) && ((travelTime - runtime.seconds()) < 1)) {

            r.m1.setPower(-power*(travelTime - runtime.seconds()));
            r.m2.setPower(-power*(travelTime - runtime.seconds()));
            r.m3.setPower(-power*(travelTime - runtime.seconds()));
            r.m4.setPower(-power*(travelTime - runtime.seconds()));

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

    public void AccelerateRightStrafe(double travelTime){
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


    @Override
    public void runOpMode() {

        r.init(hardwareMap);

        waitForStart();

        TurnRight(0.4,1000);
        TurnLeft(0.4,1000);
        AccelerateForward(3);

        AccelerateLeftStrafe(2);
        AccelerateRightStrafe(2);

        AccelerateBackwards(3);


    }
}

