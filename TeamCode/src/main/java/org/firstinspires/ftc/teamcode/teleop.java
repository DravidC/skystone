package org.firstinspires.ftc.teamcode;

import android.provider.Settings;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "teleop", group = "TeleOp")
public class teleop extends OpMode {

    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftRear;
    DcMotor rightRear;
    DcMotor intakeMotor;
    DcMotor intakeMotor1;
    DcMotor armMotor;
    DcMotor armExtensionMotor;
    Servo capStone;
    Servo bumperServo3;
    Servo bumperServo4;
    Servo intakeServo;
    Servo rotateIntakeServo;

    double mil1startticks;
    double mil2startticks;
    double liftMotorstartticks;
    double speedmodifier = 1;


    @Override
    public void init() {
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftRear = hardwareMap.dcMotor.get("leftRear");
        rightRear = hardwareMap.dcMotor.get("rightRear");
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor1");
        intakeMotor1 = hardwareMap.dcMotor.get("intakeMotor2");
        armMotor = hardwareMap.dcMotor.get("armMotor");
        armExtensionMotor = hardwareMap.dcMotor.get("armExtensionMotor");
        capStone = hardwareMap.servo.get("capStone");
        bumperServo3 = hardwareMap.servo.get("bs1");
        bumperServo4 = hardwareMap.servo.get("bs2");
        intakeServo = hardwareMap.servo.get("intakeServo");
        rotateIntakeServo = hardwareMap.servo.get("rotateServo");


        mil1startticks = 0; //mil1.getCurrentPosition();
        mil2startticks = 0; //mil2.getCurrentPosition();
        liftMotorstartticks = 0; //liftMotor.getCurrentPosition();

        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        //Driving
        double leftstickx = 0;
        double leftsticky = 0;
        double rightstickx = 0;
        double wheelpower;
        double stickangleradians;
        double rightX;
        double leftfrontpower;
        double rightfrontpower;
        double leftrearpower;
        double rightrearpower;
        double dpadpower = .2;
        double dpadturningpower = .4;
        // braking adjustment
        if (gamepad1.right_bumper) {
            speedmodifier = .5;
        }
        if (gamepad1.left_bumper) {
            speedmodifier = 1;
        }

        if (gamepad1.dpad_up) {
            leftsticky = dpadpower;
        } else if (gamepad1.dpad_right) {
            leftstickx = dpadturningpower;
        } else if (gamepad1.dpad_down) {
            leftsticky = -dpadpower;
        } else if (gamepad1.dpad_left) {
            leftstickx = -dpadturningpower;
        } else {
            leftstickx = gamepad1.left_stick_x;
            leftsticky = -gamepad1.left_stick_y;
            rightstickx = gamepad1.right_stick_x * speedmodifier;
        }
        if (Math.abs(leftsticky) <= .15) {
            leftsticky = 0;
        }

        wheelpower = Math.hypot(leftstickx, leftsticky);
        stickangleradians = Math.atan2(leftsticky, leftstickx);

        stickangleradians = stickangleradians - Math.PI / 4; //adjust by 45 degrees

        rightX = rightstickx * .5;
        leftfrontpower = wheelpower * Math.cos(stickangleradians) + rightX;
        rightfrontpower = wheelpower * Math.sin(stickangleradians) - rightX;
        leftrearpower = wheelpower * Math.sin(stickangleradians) + rightX;
        rightrearpower = wheelpower * Math.cos(stickangleradians) - rightX;

        leftFront.setPower(leftfrontpower);
        rightFront.setPower(rightfrontpower);
        leftRear.setPower(leftrearpower);
        rightRear.setPower(rightrearpower);



        if(gamepad2.right_bumper){
            intakeMotor.setPower(-1.0);
            intakeMotor1.setPower(1.0);
        } else if(gamepad2.left_bumper){
            intakeMotor.setPower(1.0);
            intakeMotor1.setPower(-1.0);
        } else if(gamepad2.b){
            intakeMotor1.setPower(0.0);
            intakeMotor.setPower(0.0);
        }

        armMotor.setPower(gamepad2.left_stick_y);

        armExtensionMotor.setPower(gamepad2.right_stick_y);

        if(gamepad2.a){
            rotateIntakeServo.setPosition(1.0);
        } else if (gamepad2.y) {
            rotateIntakeServo.setPosition(0.0);
        }

        if(gamepad1.a){
            bumperServo3.setPosition(0.0);
            bumperServo4.setPosition(1.0);
        } else if (gamepad1.b) {
            bumperServo3.setPosition(1.0);
            bumperServo4.setPosition(0.0);
        }

        if(gamepad2.dpad_left){
            intakeServo.setPosition(0.0);
        } else if (gamepad2.dpad_right){
            intakeServo.setPosition(1.0);
        }

        if(gamepad1.x){
            capStone.setPosition(0.0);
        } else if (gamepad1.y){
            capStone.setPosition(1.0);
        }

    }


}