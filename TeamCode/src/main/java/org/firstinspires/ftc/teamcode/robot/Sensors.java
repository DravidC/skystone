package org.firstinspires.ftc.teamcode.robot;


import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

public class Sensors {

    private Robot robot = null;

    public ColorSensor sensor_color_distance = null;

    public Sensors(Robot inRobot) {

        robot = inRobot;

        sensor_color_distance = robot.hardwareMap.get(ColorSensor.class, "sensor_color_distance");


    }

    public void checkStones(double timeoutSec){
        int CurrentColor = Color.rgb(robot.sensors.sensor_color_distance.red(), robot.sensors.sensor_color_distance.green(), robot.sensors.sensor_color_distance.blue());
        if (JavaUtil.colorToSaturation(CurrentColor) >= 0.6 && JavaUtil.colorToHue(CurrentColor) > 20 && JavaUtil.colorToHue(CurrentColor) < 60) {
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

    }

}