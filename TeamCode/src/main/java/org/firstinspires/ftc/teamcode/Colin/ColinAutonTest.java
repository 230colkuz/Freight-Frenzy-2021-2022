package org.firstinspires.ftc.teamcode.Colin;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name="ColinAutonTest", group="Test")

public class ColinAutonTest extends LinearOpMode {

    //Calls the RobotHardware class
    ColinRobotHardware r = new ColinRobotHardware(); //r for robot
    private ElapsedTime runtime = new ElapsedTime();
    Autonomous_Instructions A = new Autonomous_Instructions();


    BNO055IMU             imu;
    Orientation lastAngles = new Orientation();
    double                 power = .60, lastTargetAngle, correction, rotation;





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
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        r.init(hardwareMap);

        waitForStart();

        A.TurnRight(0.4,1000);
        A.TurnLeft(0.4,1000);
        A.AccelerateForward(3);

        A.AccelerateLeftStrafe(2);
        A.AccelerateRightStrafe(2);

        A.AccelerateBackwards(3);

        A.rotate(30,5);


    }
}

