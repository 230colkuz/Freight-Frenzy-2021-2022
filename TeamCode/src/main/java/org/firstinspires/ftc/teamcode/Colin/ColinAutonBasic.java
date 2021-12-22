package org.firstinspires.ftc.teamcode.Colin;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.InstantiableUserConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Colin.ColinRobotHardware;

import java.lang.reflect.Array;
import java.util.Arrays;
import java.util.ConcurrentModificationException;
import java.util.Set;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import java.util.ArrayList;


@Autonomous(name="ColinAutonTest", group="Test")

public class ColinAutonBasic extends LinearOpMode {

    //Calls the RobotHardware class
    ColinRobotHardware robot = new ColinRobotHardware();

    double power=0.6;
    double PowerMultiplyer=1;
    private ElapsedTime     runtime = new ElapsedTime();

    public void DriveForward() {
        robot.motor1.setPower(power*PowerMultiplyer);
        robot.motor2.setPower(power*PowerMultiplyer);
        robot.motor3.setPower(power*PowerMultiplyer);
        robot.motor4.setPower(power*PowerMultiplyer);
    }
    public void DriveBackwards() {
        robot.motor1.setPower(-power*PowerMultiplyer);
        robot.motor2.setPower(-power*PowerMultiplyer);
        robot.motor3.setPower(-power*PowerMultiplyer);
        robot.motor4.setPower(-power*PowerMultiplyer);
    }

    public void StrafeRight(){
        robot.motor1.setPower(power*PowerMultiplyer);
        robot.motor2.setPower(-power*PowerMultiplyer);
        robot.motor3.setPower(-power*PowerMultiplyer);
        robot.motor4.setPower(power*PowerMultiplyer);
    }

    public void StrafeLeft(){
        robot.motor1.setPower(-power*PowerMultiplyer);
        robot.motor2.setPower(power*PowerMultiplyer);
        robot.motor3.setPower(power*PowerMultiplyer);
        robot.motor4.setPower(-power*PowerMultiplyer);
    }

    public void Stop(){
        robot.motor1.setPower(0);
        robot.motor2.setPower(0);
        robot.motor3.setPower(0);
        robot.motor4.setPower(0);
    }

    public void something(){
        /*robot.motor1.setPower(power/runtime);
        robot.motor2.setPower(-power);
        robot.motor3.setPower(-power);
        robot.motor4.setPower(power);*/
    }

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        waitForStart();

        runtime.reset();

        DriveForward();
        sleep(250);
        DriveBackwards();
        sleep(250);
        StrafeLeft();
        sleep(250);
        StrafeRight();
        sleep(250);
        Stop();
        sleep(500);
        //StrafeRight();
        //sleep(500);
        //runtime.reset();
        //something();

        stop();

    }
}
