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

@Autonomous(name="Velocity Test", group="Testing")
public class VelocityTest extends LinearOpMode {
    //Calls the RobotHardware class
    ColinRobotHardware r = new ColinRobotHardware();
    private ElapsedTime runtime = new ElapsedTime();
    double TicksPerSecond;

/*public double GetTicksPerSecond() {
    if (getRuntime() > 1) {
        runtime.reset();
        TicksPerSecond = r.m1.getCurrentPosition();
    }
    return TicksPerSecond;
}*/

    //This is what happens when the init button is pushed.
    @Override
    public void runOpMode() {
        r.init(hardwareMap);
        runtime.reset();

        waitForStart();

        r.s2.setPosition (.6);
        sleep(2000);
        r.s2.setPosition (0);
        sleep(2000);
        r.s2.setPosition(0.6);
        sleep(1000);

        /*r.m1.setPower(1);
        r.m2.setPower(1);
        r.m3.setPower(1);
        r.m4.setPower(1);

        while (opModeIsActive() && r.m1.isBusy()) {
            GetTicksPerSecond();
            telemetry.addData("Velocity", TicksPerSecond);
            telemetry.update();
        }*/
    }
}