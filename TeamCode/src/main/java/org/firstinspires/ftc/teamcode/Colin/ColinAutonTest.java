package org.firstinspires.ftc.teamcode.Colin;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name="ColinAutonTest", group="Test")

public class ColinAutonTest extends LinearOpMode {

    //Calls the RobotHardware class and autonomous instructions
    Hardware_and_Auton r = new Hardware_and_Auton(); //r for robot




    @Override
    public void runOpMode() {

        //Makes new methods for naming simplification purposes
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = true;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        r.imu = hardwareMap.get(BNO055IMU.class, "imu");

        r.imu.initialize(parameters);

        r.init(hardwareMap);

        waitForStart();

        r.TurnRight(0.4,1000);
        r.TurnLeft(0.4,1000);
        r.AccelerateForward(3);

        r.AccelerateLeftStrafe(2);
        r.AccelerateRightStrafe(2);

        r.AccelerateBackwards(3);

        r.rotate(30,5);

        stop();

    }
}

