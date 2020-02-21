package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Autonomous(name = "color_sensor_test (Blocks to Java)", group = "")
public class color_sensor_test extends LinearOpMode {
    private Robot robot = null;

    private ColorSensor sensor_color_distance;


    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        sensor_color_distance = hardwareMap.colorSensor.get("sensor_color_distance");
        robot.driveTrain.rightDrive = hardwareMap.dcMotor.get("rightFront");
        robot.driveTrain.rightDrive1 = hardwareMap.dcMotor.get("rightRear");
        robot.driveTrain.leftDrive = hardwareMap.dcMotor.get("leftFront");
        robot.driveTrain.leftDrive1 = hardwareMap.dcMotor.get("leftRear");

        // Put initialization blocks here.
        robot.driveTrain.leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.driveTrain.leftDrive1.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.driveTrain.rightDrive1.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.driveTrain.rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        waitForStart();
        robot.driveTrain.rightDrive1.setPower(0);
        robot.driveTrain.rightDrive.setPower(0);
        robot.driveTrain.leftDrive.setPower(0);
        robot.driveTrain.leftDrive1.setPower(0);
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.
                int CurrentColor = Color.rgb(sensor_color_distance.red(), sensor_color_distance.green(), sensor_color_distance.blue());
                if (JavaUtil.colorToSaturation(CurrentColor) >= 0.6 && JavaUtil.colorToHue(CurrentColor) > 60 && JavaUtil.colorToHue(CurrentColor) < 120) {
                    robot.driveTrain.leftDrive.setPower(0.5);
                    robot.driveTrain.rightDrive.setPower(0.5);
                    robot.driveTrain.leftDrive1.setPower(0.5);
                    robot.driveTrain.rightDrive1.setPower(0.5);
                } else {
                    robot.driveTrain.leftDrive.setPower(0);
                    robot.driveTrain.leftDrive1.setPower(0);
                    robot.driveTrain.rightDrive.setPower(0);
                    robot.driveTrain.rightDrive1.setPower(0);
                }
                telemetry.update();
            }
        }
    }
}