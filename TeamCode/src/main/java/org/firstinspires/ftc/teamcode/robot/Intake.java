package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.Servo;
import static android.os.SystemClock.sleep;

public class Intake {

    private Robot robot = null;
    Servo bumperServo1 = null;
    Servo bumperServo2 = null;

    public Intake(Robot inRobot) {

        robot = inRobot;
        bumperServo1 = robot.hardwareMap.servo.get("bs1");
        bumperServo2 = robot.hardwareMap.servo.get("bs2");

    }

    public void putBumpersDown () {
        bumperServo1.setPosition(1.0);
        bumperServo2.setPosition(0.0);
        sleep(2000);
    }

    public void putBumpersUp () {
        bumperServo1.setPosition(0.0);
        bumperServo2.setPosition(1.0);
        sleep(2000);
    }
}
