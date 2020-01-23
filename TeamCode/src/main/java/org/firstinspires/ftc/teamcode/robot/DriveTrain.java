package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Dravid-C on 10/7/2018.
 *
 */
public class DriveTrain {
    private Robot robot = null;
    private ElapsedTime runtime = new ElapsedTime();

    public DcMotor leftDrive = null;
    public DcMotor leftDrive1 = null;
    public DcMotor rightDrive1 = null;
    public DcMotor rightDrive = null;

    public static final int PULSES_PER_MTR_ROTATION = 384;
    public static final double INCHES_PER_WHEEL_ROTATION = 12.566;
    public static final double GEAR_RATIO = 2.0; // Motor to Wheel
    private static final boolean INVERT_DIRECTION_SIGN = false;

    private static final double GYRO_TURN_KP = 12.5 / 1000.0;
    private static final double GYRO_TURN_KI = 0.35 / 1000.0;
    private static final double GYRO_TURN_KD = 0.0 / 1000.0;

    private static final double PIXY_TURN_KP = 2000.0 / 1000.0;
    private static final double PIXY_TURN_KI = 0.0 / 1000.0;
    private static final double PIXY_TURN_KD = 0.0 / 1000.0;

    //kp = 7.0, ki = 0.3, kd =0.5
    public DriveTrain(Robot inRobot){
        robot = inRobot;

        leftDrive = robot.hardwareMap.get(DcMotor.class,"leftFront");
        rightDrive = robot.hardwareMap.get(DcMotor.class, "rightFront");
        leftDrive1 = robot.hardwareMap.get(DcMotor.class, "leftRear");
        rightDrive1 = robot.hardwareMap.get(DcMotor.class, "rightRear");

        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftDrive1.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDrive1.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    /**
     * This is the message about what the funciton does.
     * You can have multiple lines and it won't care, though
     * you should have all of your parameters and your return value
     * listed below.
     *
     * @param targetAngle This is something
     * @param timeoutSec This is something else
     */

    /**
     * Uses encoders on both wheels to drive a certain distance in inches
     * @param distance IN INCHES
     * @param power 0 to 1
     * @param timeoutSec
     */
    public void driveForwardEncoder(double distance, double power, double timeoutSec) {
    double numMtrRotations = distance * (GEAR_RATIO / INCHES_PER_WHEEL_ROTATION);
        int pulses =  (int) (numMtrRotations * PULSES_PER_MTR_ROTATION);

        if (INVERT_DIRECTION_SIGN) {
            pulses = -pulses;
        }

        DcMotor.RunMode prevLeftMode = leftDrive.getMode();
        DcMotor.RunMode prevRightMode = rightDrive.getMode();
        DcMotor.RunMode prevleftMode1 = leftDrive1.getMode();
        DcMotor.RunMode prevRightMode1 = rightDrive1.getMode();

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftDrive.setTargetPosition(pulses);
        rightDrive.setTargetPosition(pulses);
        leftDrive1.setTargetPosition(pulses);
        rightDrive1.setTargetPosition(pulses);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDrive1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();
        leftDrive.setPower(Math.abs(power));
        rightDrive.setPower(Math.abs(power));
        leftDrive1.setPower(Math.abs(power));
        rightDrive1.setPower(Math.abs(power));

        while (robot.opModeIsActive() &&
                (runtime.seconds() < timeoutSec) &&
                (leftDrive.isBusy() && rightDrive.isBusy())) {
            robot.telemetry.addData("Autonomous",  "Autonomous: %s", Boolean.toString(robot.isRunningAutonomous()));
            robot.telemetry.addData("Path1",  "Running to %7d :%7d", pulses,  pulses);
            robot.telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d",
                    leftDrive.getCurrentPosition(),
                    rightDrive.getCurrentPosition(),
                    leftDrive1.getCurrentPosition(),
                    rightDrive1.getCurrentPosition());
            robot.telemetry.update();
        }

        leftDrive.setPower(0);
        leftDrive1.setPower(0);
        rightDrive.setPower(0);
        rightDrive1.setPower(0);

        leftDrive.setMode(prevLeftMode);
        leftDrive1.setMode(prevleftMode1);
        rightDrive.setMode(prevRightMode);
        rightDrive1.setMode(prevRightMode1);
    }

    public void driveBackwardEncoder(double distance, double power, double timeoutSec) {
        driveForwardEncoder(-distance, power, timeoutSec);
    }

    public void strafeLeft(double distance, double power, double timeoutSec) {
        double numMtrRotations = (distance + 1) * (GEAR_RATIO / INCHES_PER_WHEEL_ROTATION);
        int pulses =  (int) (numMtrRotations * PULSES_PER_MTR_ROTATION);

        if (INVERT_DIRECTION_SIGN) {
            pulses = -pulses;
        }

        DcMotor.RunMode prevLeftMode = leftDrive.getMode();
        DcMotor.RunMode prevRightMode = rightDrive.getMode();
        DcMotor.RunMode prevleftMode1 = leftDrive1.getMode();
        DcMotor.RunMode prevRightMode1 = rightDrive1.getMode();

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftDrive.setTargetPosition(-pulses);
        rightDrive.setTargetPosition(pulses);
        leftDrive1.setTargetPosition(pulses);
        rightDrive1.setTargetPosition(-pulses);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDrive1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();
        leftDrive.setPower(Math.abs(power));
        rightDrive.setPower(Math.abs(power));
        leftDrive1.setPower(Math.abs(power));
        rightDrive1.setPower(Math.abs(power));

        while (robot.opModeIsActive() &&
                (runtime.seconds() < timeoutSec) &&
                (leftDrive.isBusy() && rightDrive.isBusy())) {
            robot.telemetry.addData("Autonomous",  "Autonomous: %s", Boolean.toString(robot.isRunningAutonomous()));
            robot.telemetry.addData("Path1",  "Running to %7d :%7d", pulses,  pulses);
            robot.telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d",
                    leftDrive.getCurrentPosition(),
                    rightDrive.getCurrentPosition(),
                    leftDrive1.getCurrentPosition(),
                    rightDrive1.getCurrentPosition());
            robot.telemetry.update();
        }

        leftDrive.setPower(0);
        leftDrive1.setPower(0);
        rightDrive.setPower(0);
        rightDrive1.setPower(0);

        leftDrive.setMode(prevLeftMode);
        leftDrive1.setMode(prevleftMode1);
        rightDrive.setMode(prevRightMode);
        rightDrive1.setMode(prevRightMode1);
    }

    public void strafeRight(double distance, double power, double timeoutSec) {
        double numMtrRotations = (distance + 1) * (GEAR_RATIO / INCHES_PER_WHEEL_ROTATION);
        int pulses =  (int) (numMtrRotations * PULSES_PER_MTR_ROTATION);

        if (INVERT_DIRECTION_SIGN) {
            pulses = -pulses;
        }

        DcMotor.RunMode prevLeftMode = leftDrive.getMode();
        DcMotor.RunMode prevRightMode = rightDrive.getMode();
        DcMotor.RunMode prevleftMode1 = leftDrive1.getMode();
        DcMotor.RunMode prevRightMode1 = rightDrive1.getMode();

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftDrive.setTargetPosition(pulses);
        rightDrive.setTargetPosition(-pulses);
        leftDrive1.setTargetPosition(-pulses);
        rightDrive1.setTargetPosition(pulses);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDrive1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();
        leftDrive.setPower(Math.abs(power));
        rightDrive.setPower(Math.abs(power));
        leftDrive1.setPower(Math.abs(power));
        rightDrive1.setPower(Math.abs(power));

        while (robot.opModeIsActive() &&
                (runtime.seconds() < timeoutSec) &&
                (leftDrive.isBusy() && rightDrive.isBusy())) {
            robot.telemetry.addData("Autonomous",  "Autonomous: %s", Boolean.toString(robot.isRunningAutonomous()));
            robot.telemetry.addData("Path1",  "Running to %7d :%7d", pulses,  pulses);
            robot.telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d",
                    leftDrive.getCurrentPosition(),
                    rightDrive.getCurrentPosition(),
                    leftDrive1.getCurrentPosition(),
                    rightDrive1.getCurrentPosition());
            robot.telemetry.update();
        }

        leftDrive.setPower(0);
        leftDrive1.setPower(0);
        rightDrive.setPower(0);
        rightDrive1.setPower(0);

        leftDrive.setMode(prevLeftMode);
        leftDrive1.setMode(prevleftMode1);
        rightDrive.setMode(prevRightMode);
        rightDrive1.setMode(prevRightMode1);
    }
}