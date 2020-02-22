package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Robot;

/**
 * Created by Dravid-C on 10/7/2018.
 *
 */
@Autonomous(name="Autonomous_test", group="Testing")
public class Autonomous_test extends LinearOpMode {
    private Robot robot = null;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this, hardwareMap, telemetry);

        waitForStart();
        robot.driveTrain.strafeRight(30, 0.5, 10.0);
    }

}