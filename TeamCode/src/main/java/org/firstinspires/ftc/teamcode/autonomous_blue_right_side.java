package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Robot;

@Autonomous(name="autnomous blue_right_side", group="Testing")
public class autonomous_blue_right_side extends LinearOpMode {
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
        robot.driveTrain.strafeLeft(70, 0.5, 10.0);
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
        robot.driveTrain.strafeRight(29.5, 0.5, 10.0);
    }

    private void strafeRight () {
        robot.driveTrain.strafeRight(15, 0.5, 10.0);
    }

}