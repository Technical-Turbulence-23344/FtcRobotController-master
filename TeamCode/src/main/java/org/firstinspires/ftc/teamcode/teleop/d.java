package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.LinearSlidePosition;

@TeleOp
public class d extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        boolean use =false;

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        CRServo pixelIn = hardwareMap.crservo.get("pixelIn");
        Servo pixelOut = hardwareMap.servo.get("pixelOut");
        Servo droneLauncher = hardwareMap.servo.get("droneLauncher");
        CRServo intakeRotate = hardwareMap.crservo.get("intakeRotate");
        int ticker = 0;
        int ticker2 = 0;
        int count = 2;
        double seeDistance2 = 10;
        double seeDistance = 10;
        double x = 0;
        double rightLinearSlidePos = -288.87;
        double leftLinearSlidePos = -288.87;
        double currentRuntime = 0.0;
        boolean isOn = false;
        boolean laOn = false;
        boolean lsGo = false;
        boolean servoTighten = false;
        boolean servoLossen = false;;
        boolean toes = false;
        boolean fingers = false;
        double y = 0;
        int numberInOne;
        int numberInTwo;
        boolean closed = false;
        double colorRuntime = 0.0;
        double colorWaittime = 0.0;
        double aRunTime = 0.0;
        double bRunTime = 0.0;
        double mRunTime = 0.0;
        double nRunTime = 0.0;

        pixelIn.setDirection(DcMotorSimple.Direction.REVERSE);
        droneLauncher.setDirection(Servo.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double currentPositionLin = (-1)*LinearSlidePosition.pos +100;
        boolean linearSlideMode = false;
        boolean prevState = false;
        String intendedColor = "red";
        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Set your initial pose to x: 10, y: 10, facing 90 degrees
        drive.setPoseEstimate(new Pose2d(10, 10, Math.toRadians(90)));
        boolean hello = false;
        waitForStart();


        if (isStopRequested()) return;

        while (opModeIsActive()) {







            double rightOdoPos = (backRightMotor.getCurrentPosition() * -1);
            double leftOdoPos = (frontLeftMotor.getCurrentPosition());
            double sideOdoPos = (backLeftMotor.getCurrentPosition());



            double turbo = 0.8 + 0.2 * gamepad1.right_trigger - 0.6 * gamepad1.left_trigger;
            //Y button controller 1
            if (gamepad1.right_stick_y > 0.3 || gamepad1.right_stick_y < -0.3) {
                y = -gamepad1.right_stick_y * turbo; // Remember, Y stick value is reversed
            } else {
                y = 0;
            }
            //X button controller 1
            if (gamepad1.right_stick_x > 0.5 || gamepad1.right_stick_x < -0.5) {
                x = gamepad1.right_stick_x * 1.3 * turbo; // Counteract imperfect strafing
            } else {
                x = 0;
            }
            double rx = gamepad1.left_stick_x * turbo;


            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;


            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            //Linear SLide button control 2





            if (gamepad2.x) {
                closed = true;
                aRunTime = getRuntime();
            }
            if (toes && !gamepad2.y) {
                nRunTime = getRuntime() - mRunTime;
                if (nRunTime < 0.5) {
                    pixelOut.setPosition(0);
                } else if (nRunTime < 1) {
                    pixelIn.setPower(0.77);
                }  else {
                    pixelIn.setPower(0);
                    toes = false;
                }
                if(gamepad2.x){
                    toes =false;
                }
            }
            if (closed && !gamepad2.x) {
                bRunTime = getRuntime() - aRunTime;
                if (bRunTime < 0.2) {
                    pixelOut.setPosition(0);
                } else if (bRunTime < 0.4) {
                    pixelOut.setPosition(0.8);
                } else if (bRunTime < 0.7) {
                    pixelIn.setPower(0.77);
                } else {
                    pixelIn.setPower(0);
                    if (gamepad2.y){
                        pixelOut.setPosition(0);
                    }
                    closed = false;
                }
            }
            if (gamepad2.y) {
                toes = true;
                mRunTime = getRuntime();
            }
            if (gamepad1.start) {
                droneLauncher.setPosition(0.8);
            }

            if (gamepad2.left_stick_y>0) {
                intakeRotate.setPower(-1);

            } else if (gamepad2.left_stick_y<0) {
                intakeRotate.setPower(1);




            }
        }
    }
}

