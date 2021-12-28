package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

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

//import virtual_robot.util.AngleUtils;

/**
 * My Custom Autonomous Opmode
 */
//@Disabled
@Autonomous(name = "Dylan's Speed Up and Slow Down", group = "Testing")
public class Dylans_SpeedUpAndSlowDown extends LinearOpMode {

    double power=0.6;
    private ElapsedTime     runtime = new ElapsedTime();
    DcMotor m1, m2, m3, m4;
    //GyroSensor gyro;
    BNO055IMU imu;
    ColorSensor colorSensor;
    Servo backServo;

    public void Stop(){
        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);
    }

    //Method to print telemetry to phone
    public void PrintPowerAndRuntime() {
        telemetry.addData("motor1 Power", m1.getPower());
        telemetry.addData("motor2 Power", m2.getPower());
        telemetry.addData("motor3 Power", m3.getPower());
        telemetry.addData("motor4 Power", m4.getPower());
        telemetry.addData("runtime", getRuntime());
        telemetry.update();
    }

    public void DriveForward(double moveTime) {

        if (opModeIsActive()) {

            //Resets runtime only once when DriveForward is called
            runtime.reset();

            //This while loop loops continuously
            /*This loop is the speed up portion of speed up/slow down. (runtime.seconds() < moveTime) means it will terminate after we've reached our target move time.
            (moveTime>2) means it will do this smoothing function only if we have a target move time longer than 2. Later in the code, we tell it what to do if the move
            time is less than 2 (no smoothing function, 0.2 power). (moveTime-runtime.seconds()) > 1) means that it will speed up until there's 1 second left between our
            target move time and our current runtime.
            */
            while (opModeIsActive() && (runtime.seconds() < moveTime) && (moveTime > 2) && ((moveTime-runtime.seconds()) > 1)) {

                m1.setPower(power*(0.5*runtime.seconds()));
                m2.setPower(power*(0.5*runtime.seconds()));
                m3.setPower(power*(0.5*runtime.seconds()));
                m4.setPower(power*(0.5*runtime.seconds()));

                /*If the absolute power that the motors are targeted to go to exceed the power levels we specified (listed in the beginning as double power),
                set the power to the power we specified
                 */
                if((Math.abs(m1.getPower())>=power) && (Math.abs(m2.getPower())>=power)) {
                    m1.setPower(power);
                    m2.setPower(power);
                    m3.setPower(power);
                    m4.setPower(power);
                }
                PrintPowerAndRuntime();
            }

            //This loop is the slow down portion of speed up/slow down
            while (opModeIsActive() && (runtime.seconds() < moveTime) && (moveTime > 2) && ((moveTime-runtime.seconds()) <= 1)) {

                m1.setPower(power*(moveTime-runtime.seconds()));
                m2.setPower(power*(moveTime-runtime.seconds()));
                m3.setPower(power*(moveTime-runtime.seconds()));
                m4.setPower(power*(moveTime-runtime.seconds()));

                PrintPowerAndRuntime();
            }

            //If we're moving for less than or equal to 2 seconds, do this instead
            while (opModeIsActive() && (runtime.seconds() < moveTime) && (moveTime <= 2)) {
                m1.setPower(.2);
                m2.setPower(.2);
                m3.setPower(.2);
                m4.setPower(.2);

                PrintPowerAndRuntime();
            }
            Stop();
            sleep(250);
        }
    }
    public void DriveBackward(double moveTime) {

        if (opModeIsActive()) {

            runtime.reset();

            while (opModeIsActive() && (runtime.seconds() < moveTime) && (moveTime > 2) && ((moveTime-runtime.seconds()) > 1)) {

                m1.setPower(-power*(0.5*runtime.seconds()));
                m2.setPower(-power*(0.5*runtime.seconds()));
                m3.setPower(-power*(0.5*runtime.seconds()));
                m4.setPower(-power*(0.5*runtime.seconds()));

                /*If the absolute power that the motors are targeted to go to exceed the power levels we specified (listed in the beginning as double power),
                set the power to the power we specified (in this case, we will multiply power by -1 to go backwards)
                 */
                if((Math.abs(m1.getPower())>=power) && (Math.abs(m2.getPower())>=power)) {
                    m1.setPower(-power);
                    m2.setPower(-power);
                    m3.setPower(-power);
                    m4.setPower(-power);
                }

                PrintPowerAndRuntime();
            }
            while (opModeIsActive() && (runtime.seconds() < moveTime) && (moveTime > 2) && ((moveTime-runtime.seconds()) <= 1)) {

                m1.setPower(-power*(moveTime-runtime.seconds()));
                m2.setPower(-power*(moveTime-runtime.seconds()));
                m3.setPower(-power*(moveTime-runtime.seconds()));
                m4.setPower(-power*(moveTime-runtime.seconds()));

                PrintPowerAndRuntime();
            }
            while (opModeIsActive() && (runtime.seconds() < moveTime) && (moveTime <= 2)) {
                m1.setPower(-.2);
                m2.setPower(-.2);
                m3.setPower(-.2);
                m4.setPower(-.2);

                PrintPowerAndRuntime();
            }
            Stop();
            sleep(250);
        }
    }

    public void StrafeRight(double moveTime) {

        if (opModeIsActive()) {

            runtime.reset();

            while (opModeIsActive() && (runtime.seconds() < moveTime) && (moveTime > 2) && ((moveTime-runtime.seconds()) > 1)) {

                m1.setPower(-power*(0.5*runtime.seconds()));
                m2.setPower(power*(0.5*runtime.seconds()));
                m3.setPower(-power*(0.5*runtime.seconds()));
                m4.setPower(power*(0.5*runtime.seconds()));

                if((Math.abs(m1.getPower())>=power) && (Math.abs(m2.getPower())>=power)) {
                    m1.setPower(-power);
                    m2.setPower(power);
                    m3.setPower(-power);
                    m4.setPower(power);
                }

                PrintPowerAndRuntime();
            }
            while (opModeIsActive() && (runtime.seconds() < moveTime) && (moveTime > 2) && ((moveTime-runtime.seconds()) <= 1)) {

                m1.setPower(-power*(moveTime-runtime.seconds()));
                m2.setPower(power*(moveTime-runtime.seconds()));
                m3.setPower(-power*(moveTime-runtime.seconds()));
                m4.setPower(power*(moveTime-runtime.seconds()));

                PrintPowerAndRuntime();
            }
            while (opModeIsActive() && (runtime.seconds() < moveTime) && (moveTime <= 2)) {
                m1.setPower(-.2);
                m2.setPower(.2);
                m3.setPower(-.2);
                m4.setPower(.2);

                PrintPowerAndRuntime();
            }
            Stop();
            sleep(250);
        }
    }

    public void StrafeLeft(double moveTime) {

        if (opModeIsActive()) {

            runtime.reset();

            while (opModeIsActive() && (runtime.seconds() < moveTime) && (moveTime > 2) && ((moveTime-runtime.seconds()) > 1)) {

                m1.setPower(power*(0.5*runtime.seconds()));
                m2.setPower(-power*(0.5*runtime.seconds()));
                m3.setPower(power*(0.5*runtime.seconds()));
                m4.setPower(-power*(0.5*runtime.seconds()));

                if((Math.abs(m1.getPower())>=power) && (Math.abs(m2.getPower())>=power)) {
                    m1.setPower(power);
                    m2.setPower(-power);
                    m3.setPower(power);
                    m4.setPower(-power);
                }

                PrintPowerAndRuntime();
            }
            while (opModeIsActive() && (runtime.seconds() < moveTime) && (moveTime > 2) && ((moveTime-runtime.seconds()) <= 1)) {

                m1.setPower(power*(moveTime-runtime.seconds()));
                m2.setPower(-power*(moveTime-runtime.seconds()));
                m3.setPower(power*(moveTime-runtime.seconds()));
                m4.setPower(-power*(moveTime-runtime.seconds()));

                PrintPowerAndRuntime();
            }
            while (opModeIsActive() && (runtime.seconds() < moveTime) && (moveTime <= 2)) {
                m1.setPower(.2);
                m2.setPower(-.2);
                m3.setPower(.2);
                m4.setPower(-.2);

                PrintPowerAndRuntime();
            }
            Stop();
            sleep(250);
        }
    }

    public void runOpMode(){

        m1 = hardwareMap.dcMotor.get("back_left_motor");
        m2 = hardwareMap.dcMotor.get("front_left_motor");
        m3 = hardwareMap.dcMotor.get("front_right_motor");
        m4 = hardwareMap.dcMotor.get("back_right_motor");
        m1.setDirection(DcMotor.Direction.REVERSE);
        m2.setDirection(DcMotor.Direction.REVERSE);
        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ElapsedTime waitTime = new ElapsedTime();
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Seconds since init","%d. Press start when ready.", (int)waitTime.seconds());
            telemetry.update();
        }

        DriveForward(5);
        DriveBackward(4);
        StrafeRight(5);
        StrafeLeft(1.5);
        DriveForward(3);
    }
}
