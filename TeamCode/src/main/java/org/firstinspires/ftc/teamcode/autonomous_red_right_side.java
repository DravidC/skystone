package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Robot;

@Autonomous(name="autnomous red_right_side", group="Testing")
public class autonomous_red_right_side extends LinearOpMode {
    private Robot robot = null;

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        waitForStart();

        moveForward();

        moveLeft();

        putGrippersDown();

        moveBackward();

        putGrippersUp();

        moveRight();

        moveForward();

        strafeRight();

    }

    private void initialize() {
        robot = new Robot(this, hardwareMap, telemetry);

    }

    private void moveForward () {
        robot.driveTrain.driveForwardEncoder(29, 0.5, 10.0);
    }

    private void moveLeft () {
        robot.driveTrain.strafeRight(70, 0.5, 10.0);
    }

    private void putGrippersDown () {
        robot.intake.putBumpersDown();
    }

    private void moveBackward () {
        robot.driveTrain.driveBackwardEncoder(35, 0.5, 10.0);
    }
    private void putGrippersUp () {
        robot.intake.putBumpersUp();
    }

    private void moveRight () {
        robot.driveTrain.strafeLeft(29.5, 0.5, 10.0);
    }

    private void strafeRight () {
        robot.driveTrain.strafeLeft(15, 0.5, 10.0);
    }

}