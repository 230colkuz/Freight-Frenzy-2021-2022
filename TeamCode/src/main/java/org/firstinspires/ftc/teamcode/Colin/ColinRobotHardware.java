package org.firstinspires.ftc.teamcode.Colin;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;

public class ColinRobotHardware {
    //  INSTANTIATE MOTORS AND SERVOS
    public DcMotor m1;
    public DcMotor m2;
    public DcMotor m3;
    public DcMotor m4;
    public DcMotor m5;
    public DcMotor m6;
    /*public DcMotor motor7;
     DcMotor motor8;*/
    public Servo s1;
    public Servo s2;


    //INSTANTIATE SENSORS
    public BNO055IMU imu;
    /*//public GyroSensor gyroSensor;
    //public ColorSensor colorSensor;
    public DcMotor LeftEncoder, RightEncoder, BackEncoder;*/

    //CREATE THE HARDWARE MAP
    HardwareMap hardwareMap;

    public void init(HardwareMap hardwareMap) {

        // DEFINE MOTORS AND SERVOS
        m1 = hardwareMap.get(DcMotor.class, "motor1"); // back right
        m2 = hardwareMap.get(DcMotor.class, "motor2"); // back left
        m3 = hardwareMap.get(DcMotor.class, "motor3"); // front right
        m4 = hardwareMap.get(DcMotor.class, "motor4"); // front left
        m5 = hardwareMap.get(DcMotor.class, "motor5"); // arm motor
        m6 = hardwareMap.get(DcMotor.class, "motor6"); // Carousel motor
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
        s1.setPosition(1);
        s2.setPosition(0.0);

        //SET MOTOR zeroPowerBehavior (Brake for the Wheels and Float for anything else
            m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //...ZeroPowerBehavior.FLOAT
            m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        m5.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        m6.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);



        //SET MOTOR DIRECTIONS
        m1.setDirection(DcMotor.Direction.FORWARD); //motor1.setDirection(DcMotor.Direction.REVERSE)
        m2.setDirection(DcMotor.Direction.REVERSE); //motor2.setDirection(DcMotor.Direction.FORWARD)
        m3.setDirection(DcMotor.Direction.FORWARD); //motor3.setDirection(DcMotor.Direction.REVERSE)
        m4.setDirection(DcMotor.Direction.REVERSE); //motor4.setDirection(DcMotor.Direction.FORWARD)
        m5.setDirection(DcMotor.Direction.REVERSE); //motor5.setDirection(DcMotor.Direction.REVERSE)
        m6.setDirection(DcMotor.Direction.FORWARD); //motor6.setDirection(DcMotor.Direction.FORWARD)


        //CALIBRATE SENSORS
        //gyroSensor.calibrate();

    }
}
