package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp
@Disabled

public class Drivetrain5 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        CRServo linActServo = hardwareMap.crservo.get("linActServo");
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        DcMotor linearSlideLeft = hardwareMap.dcMotor.get("linearSlideLeft");
        DcMotor linearSlideRight = hardwareMap.dcMotor.get("linearSlideRight");
        DcMotor intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        CRServo pixelIn = hardwareMap.crservo.get("pixelIn");
        Servo pixelOut = hardwareMap.servo.get("pixelOut");
        Servo droneLauncher = hardwareMap.servo.get("droneLauncher");
        CRServo intakeMove = hardwareMap.crservo.get("intakeMove");
        CRServo intakeRotate = hardwareMap.crservo.get("intakeRotate");
        DcMotor linearActuator = hardwareMap.dcMotor.get("linearActuator");
        DistanceSensor dist = hardwareMap.get(DistanceSensor.class, "dist");
        DistanceSensor dist2 = hardwareMap.get(DistanceSensor.class, "dist2");
        RevBlinkinLedDriver lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
        int ticker = 0;
        int ticker2 = 0;
        int count = 2;
        double seeDistance2 = dist.getDistance(DistanceUnit.INCH);
        double seeDistance = dist.getDistance(DistanceUnit.INCH);
        double x = 0;
        double rightLinearSlidePos = -288.87;
        double leftLinearSlidePos = -288.87;
        double currentRuntime = 0.0;
        boolean isOn = false;
        boolean laOn = false;
        boolean lsGo = false;
        boolean servoTighten = false;
        boolean servoLossen = false;;

        double y = 0;
        boolean closed = false;
        double aRunTime = 0.0;
        double bRunTime = 0.0;
        linearSlideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMove.setDirection(DcMotorSimple.Direction.REVERSE);
        pixelIn.setDirection(DcMotorSimple.Direction.REVERSE);
        droneLauncher.setDirection(Servo.Direction.REVERSE);
        linearSlideLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        linearSlideRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        boolean linearSlideMode = false;
        boolean prevState = false;
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
            drive.update();

            // Retrieve your pose
            Pose2d myPose = drive.getPoseEstimate();
            telemetry.addData("x", myPose.getX());
            telemetry.addData("y", myPose.getY());
            telemetry.addData("heading", Math.toDegrees(myPose.getHeading()));


            telemetry.addData("x", myPose.getX());
            telemetry.addData("y", myPose.getY());
            telemetry.addData("heading", myPose.getHeading());
            double rightOdoPos = (backRightMotor.getCurrentPosition() * -1);
            double leftOdoPos = (frontLeftMotor.getCurrentPosition());
            double sideOdoPos = (backLeftMotor.getCurrentPosition());
            telemetry.addData("FWDDistance", (rightOdoPos + leftOdoPos) / 2);
            telemetry.addData("LeftOdoPos", (leftOdoPos));
            telemetry.addData("RightOdoPos", (rightOdoPos));
            telemetry.addData("SideOdoPos", (sideOdoPos));


            double turbo = 0.58 + 0.42 * gamepad1.right_trigger - 0.42 * gamepad1.left_trigger;

            if (gamepad1.right_stick_y > 0.3 || gamepad1.right_stick_y < -0.3) {
                y = -gamepad1.right_stick_y * turbo; // Remember, Y stick value is reversed
            } else {
                y = 0;
            }

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
            if (gamepad2.start) {
                linearSlideMode = true;
            }
            if (gamepad2.back) {
                linearSlideMode = false;
            }

            telemetry.addData("right", rightLinearSlidePos);
            telemetry.addData("left", rightLinearSlidePos);
            telemetry.addData("rightPos", linearSlideRight.getCurrentPosition());
            telemetry.addData("leftPos", linearSlideLeft.getCurrentPosition());
            if (gamepad2.right_stick_button) {
                rightLinearSlidePos = linearSlideRight.getCurrentPosition();
                leftLinearSlidePos = linearSlideLeft.getCurrentPosition();
            }
            if (gamepad2.left_stick_button) {
                rightLinearSlidePos = -288.87;
                leftLinearSlidePos = -288.87;
            }
            if (gamepad2.right_trigger != 0) {
                linearSlideLeft.setPower(gamepad2.right_trigger);
                linearSlideRight.setPower(gamepad2.right_trigger);
//
            } else if (gamepad2.left_trigger != 0 && ((linearSlideRight.getCurrentPosition() < rightLinearSlidePos) || (rightLinearSlidePos == -288.87)) && ((linearSlideLeft.getCurrentPosition() < leftLinearSlidePos) || (leftLinearSlidePos == -288.87))) {
                linearSlideLeft.setPower(-gamepad2.left_trigger);
                linearSlideRight.setPower(-gamepad2.left_trigger);
            } else if (linearSlideMode) {
                linearSlideLeft.setPower(0.1);
                linearSlideRight.setPower(0.1);
            } else {
                linearSlideLeft.setPower(0);
                linearSlideRight.setPower(0);
            }
            telemetry.update();
            if (gamepad2.a) {
                intakeMotor.setPower(0.5);
                pixelIn.setPower(1);
                pixelOut.setPosition(.8);
                intakeRotate.setPower(-0.5);
                intakeMove.setPower(0);
            } else if (gamepad2.b) {
                intakeMotor.setPower(-0.8);
                pixelIn.setPower(-1);
                intakeRotate.setPower(-.5);
                intakeMove.setPower(-.4);
            } else {
                intakeMotor.setPower(0);
                pixelIn.setPower(0);
            }
            if (gamepad2.x) {
                closed = true;
                aRunTime = getRuntime();
            }
            if (closed && !gamepad2.x) {
                bRunTime = getRuntime() - aRunTime;
                if (bRunTime < 0.5) {
                    pixelOut.setPosition(0);
                } else if (bRunTime < 1) {
                    pixelOut.setPosition(0.8);
                } else if (bRunTime < 1.5) {
                    pixelIn.setPower(0.77);
                } else {
                    pixelIn.setPower(0);
                    closed = false;
                }
            }
            if (gamepad2.y) {
                pixelIn.setPower(1);
                pixelOut.setPosition(0);
            }
            if (gamepad1.start) {
                droneLauncher.setPosition(0.8);
            }

            if (gamepad2.right_stick_y>0) {
                intakeRotate.setPower(1);
                intakeMove.setPower(gamepad2.right_stick_y);
            } else if (gamepad2.left_stick_y<0) {
                intakeRotate.setPower(-1);
                intakeMove.setPower(gamepad2.right_stick_y);
            } else if (gamepad2.dpad_up) {
                intakeRotate.setPower(0.6);
                intakeMove.setPower(0.3);
            } else if (gamepad2.dpad_down) {
                intakeRotate.setPower(-0.6);
                intakeMove.setPower(0.3);
            } else {
                intakeRotate.setPower(0);
                intakeMove.setPower(0);
            }


            if (gamepad1.left_bumper) {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            } else if (gamepad2.right_stick_x >= 0.5) {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
            } else if (gamepad2.right_stick_x <= -0.5) {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
            } else if (gamepad2.right_stick_y >= 0.5) {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            } else if (gamepad2.right_stick_y <= -0.5) {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
            } else {
                if (seeDistance < (0.5 + 1.85)) {
                    if (seeDistance2 < (0.5 + 1.85)) {
                        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                    }
                } else {
                    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_PARTY_PALETTE);
                }
            }
            ticker++;
            if (ticker % 5 == 0) {
                seeDistance = dist.getDistance(DistanceUnit.INCH);
            }
            ticker2++;
            if (ticker2 % 5 == 0) {
                seeDistance2 = dist2.getDistance(DistanceUnit.INCH);
            }


            if (gamepad1.back) {
                resetRuntime();
                isOn = true;

            }
            if (isOn && !gamepad1.back) {
                currentRuntime = getRuntime();
                if (currentRuntime < 3) {
                    linActServo.setPower(1);
                } else if (currentRuntime < 4) {
                    linActServo.setPower(0);
                    linearActuator.setPower(1);
                } else if (currentRuntime < 9.5) {
                    linearActuator.setPower(0);
                    linActServo.setPower(-1);
                } else if (currentRuntime < 10.5) {
                    linActServo.setPower(1);
                } else {
                    linActServo.setPower(0);
                    linearActuator.setPower(0);
                }


                if (gamepad1.x) {
                    linActServo.setPower(-1);
                    if(!gamepad1.x){
                        linActServo.setPower(0);
                    }
                }
                if (gamepad1.b) {
                    linActServo.setPower(1);
                    if (!gamepad1.b) {
                        linActServo.setPower(0);
                    }
                }
                if (gamepad1.y) {
                    linearActuator.setPower(1);
                    if (!gamepad1.y) {
                        linearActuator.setPower(0);
                    }
                }
                if (gamepad1.a) {
                    linearActuator.setPower(-1);
                    if (!gamepad1.a) {
                        linearActuator.setPower(0);
                    }

                }


            }
        }
    }
}

