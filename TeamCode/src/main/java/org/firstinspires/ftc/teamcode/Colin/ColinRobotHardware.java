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
    public DcMotor motor1;
    public DcMotor motor2;
    public DcMotor motor3;
    public DcMotor motor4;
    public DcMotor motor5;
    public DcMotor motor6;
    /*public DcMotor motor7;
     DcMotor motor8;*/
    public Servo servo1;
    public Servo servo2;


    /*//INSTANTIATE SENSORS
    public BNO055IMU imu;
    //public GyroSensor gyroSensor;
    //public ColorSensor colorSensor;
    public DcMotor LeftEncoder, RightEncoder, BackEncoder;*/

    //CREATE THE HARDWARE MAP
    HardwareMap hardwareMap;

    public void init(HardwareMap hardwareMap) {

        // DEFINE MOTORS AND SERVOS
        motor1 = hardwareMap.get(DcMotorEx.class, "motor1"); // drive motor
        motor2 = hardwareMap.get(DcMotorEx.class, "motor2"); // drive motor
        motor3 = hardwareMap.get(DcMotorEx.class, "motor3"); // drive motor
        motor4 = hardwareMap.get(DcMotorEx.class, "motor4"); // drive motor
        motor5 = hardwareMap.get(DcMotorEx.class, "motor5"); // carousel motor
        motor6 = hardwareMap.get(DcMotor.class, "motor6"); // arm motor
        servo1 = hardwareMap.get(Servo.class, "servo1"); // wrist
        servo2 = hardwareMap.get(Servo.class, "servo2"); //claw

        //SET MOTOR POWERS
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);
        motor5.setPower(0);
        motor6.setPower(0);


        //SET SERVO POSITION
        servo1.setPosition(1);
        servo2.setPosition(0.0);

        //SET MOTOR zeroPowerBehavior
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); //...ZeroPowerBehavior.FLOAT
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor5.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor5.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor6.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);



        //SET MOTOR DIRECTIONS
        motor1.setDirection(DcMotor.Direction.FORWARD); //motor1.setDirection(DcMotor.Direction.REVERSE)
        motor2.setDirection(DcMotor.Direction.REVERSE); //motor2.setDirection(DcMotor.Direction.FORWARD)
        motor3.setDirection(DcMotor.Direction.FORWARD); //motor3.setDirection(DcMotor.Direction.REVERSE)
        motor4.setDirection(DcMotor.Direction.REVERSE); //motor4.setDirection(DcMotor.Direction.FORWARD)
        motor5.setDirection(DcMotor.Direction.REVERSE); //motor5.setDirection(DcMotor.Direction.REVERSE)
        motor6.setDirection(DcMotor.Direction.FORWARD); //motor6.setDirection(DcMotor.Direction.FORWARD)


        //CALIBRATE SENSORS
        //gyroSensor.calibrate();

    }
}
