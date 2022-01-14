// Simple autonomous program that uses an IMU to drive in a straight line.
// Uses REV Hub's built in IMU in place of a gyro.

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Colin.ColinRobotHardware;

@Autonomous(name="Velocity Test", group="Testing")
public class VelocityTest extends LinearOpMode {
    //Calls the RobotHardware class
    ColinRobotHardware r = new ColinRobotHardware();
    private ElapsedTime runtime = new ElapsedTime();
    double TicksPerSecond;

public double GetTicksPerSecond() {
    if (getRuntime() > 1) {
        runtime.reset();
        TicksPerSecond = r.m1.getCurrentPosition();
    }
    return TicksPerSecond;
}

    public void StopDriving()
    {
        r.m1.setPower(0);
        r.m2.setPower(0);
        r.m3.setPower(0);
        r.m4.setPower(0);
    }

    //This is what happens when the init button is pushed.
    @Override
    public void runOpMode() {
        r.init(hardwareMap);
        runtime.reset();

        r.m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        r.m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        r.m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        r.m4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        r.m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        r.m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        r.m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        r.m4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        /*r.s2.setPosition (.6);
        sleep(2000);
        r.s2.setPosition (0);
        sleep(2000);
        r.s2.setPosition(0.6);
        sleep(1000);*/

        while (opModeIsActive()) {
            r.m1.setPower(1);
            r.m2.setPower(1);
            r.m3.setPower(1);
            r.m4.setPower(1);
            GetTicksPerSecond();
            telemetry.addData("position", r.m1.getCurrentPosition());
            telemetry.addData("Velocity", TicksPerSecond);
            telemetry.update();
        }
    }
}