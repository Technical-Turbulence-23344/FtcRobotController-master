package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
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
public class Drivetrain4 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        boolean use =false;
        CRServo linActServo = hardwareMap.crservo.get("linActServo");
        CRServo mosaicMover = hardwareMap.crservo.get("mosaicMover");
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
        NormalizedColorSensor color1 = hardwareMap.get(NormalizedColorSensor.class, "color");
        NormalizedColorSensor color2 = hardwareMap.get(NormalizedColorSensor.class, "color2");
        CRServo intakeMove = hardwareMap.crservo.get("intakeMove");
        CRServo intakeRotate = hardwareMap.crservo.get("intakeRotate");
        DcMotor linearActuator = hardwareMap.dcMotor.get("linearActuator");
        DistanceSensor dist = hardwareMap.get(DistanceSensor.class, "dist");
        DistanceSensor dist2 = hardwareMap.get(DistanceSensor.class, "dist2");
        RevBlinkinLedDriver lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
        CRServo stackKnocker = hardwareMap.crservo.get("stackKnocker");
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
            NormalizedRGBA colors = color1.getNormalizedColors();
            NormalizedRGBA colors2 = color2.getNormalizedColors();
            String colorIn1 = "Undefined";
            String colorIn2 = "Undefined";
            double colorSum = colors.red + colors.green + colors.blue;
            double colorSum2 = colors2.red + colors2.green + colors2.blue;
            boolean isPixelIn1 = false;
            boolean isPixelIn2 = false;
            double redPercentage =(100*colors.red)/colorSum;
            double greenPercentage =(100*colors.green)/colorSum;
            double bluePercentage =(100*colors.blue)/colorSum;
            double redPercentage2 =(100*colors2.red)/colorSum2;
            double greenPercentage2 =(100*colors2.green)/colorSum2;
            double bluePercentage2 =(100*colors2.blue)/colorSum2;
            if (((DistanceSensor) color1).getDistance(DistanceUnit.INCH)<0.7){
                isPixelIn1 = true;
            }
            if ((isPixelIn1)&&(greenPercentage>39)&&(greenPercentage<43)&&(redPercentage>20)&&(redPercentage<24)&&(bluePercentage>34)&&(bluePercentage<38)){
                colorIn1 = "white!";
            }
            if ((isPixelIn1)&&(greenPercentage>45)&&(greenPercentage<51)&&(redPercentage>28)&&(redPercentage<34)&&(bluePercentage>16)&&(bluePercentage<22)){
                colorIn1 = "yellow!";
            }
            if ((isPixelIn1)&&(greenPercentage>31)&&(greenPercentage<37)&&(redPercentage>19)&&(redPercentage<25)&&(bluePercentage>41)&&(bluePercentage<47)){
                colorIn1 = "purple!";
            }
            if ((isPixelIn1)&&(greenPercentage>49)&&(greenPercentage<55)&&(redPercentage>16)&&(redPercentage<22)&&(bluePercentage>25)&&(bluePercentage<31)){
                colorIn1 = "green!";
            }
            //Color sensor 2
            if (((DistanceSensor) color2).getDistance(DistanceUnit.INCH)<1.0){
                isPixelIn2 = true;
            }
            if ((isPixelIn2)&&(greenPercentage2>39)&&(greenPercentage2<43)&&(redPercentage2>20)&&(redPercentage2<24)&&(bluePercentage2>34)&&(bluePercentage2<38)){
                colorIn2 = "white!";
            }
            if ((isPixelIn2)&&(greenPercentage2>48)&&(greenPercentage2<54)&&(redPercentage2>32)&&(redPercentage2<37)&&(bluePercentage2>11)&&(bluePercentage2<16)){
                colorIn2 = "yellow!";
            }
            if ((isPixelIn2)&&(greenPercentage2>29)&&(greenPercentage2<34)&&(redPercentage2>19)&&(redPercentage2<23)&&(bluePercentage2>44)&&(bluePercentage2<48)){
                colorIn2 = "purple!";
            }
            if ((isPixelIn2)&&(greenPercentage2>56)&&(greenPercentage2<62)&&(redPercentage2>14)&&(redPercentage2<20)&&(bluePercentage2>21)&&(bluePercentage2<26)){
                colorIn2 = "green!";
            }
            if (isPixelIn1==true){
                numberInOne =1;
            } else {
                numberInOne = 0;
            }

            if (isPixelIn2==true){
                numberInTwo =1;
            } else {
                numberInTwo = 0;
            }
            if (numberInTwo+numberInOne==0){
                intendedColor = "red";
            }
            if (numberInTwo+numberInOne==1){
                intendedColor = "yellow";
            }
            if (numberInTwo+numberInOne==2){
                intendedColor = "green";
            }



            telemetry.addData("Pixel in Slot 1: ",isPixelIn1);
            telemetry.addData("Pixel Color in Slot 1: ",colorIn1);
            telemetry.addData("distance",((DistanceSensor) color1).getDistance(DistanceUnit.INCH));
            telemetry.addLine()
                    .addData("Red", "%.3f", redPercentage)
                    .addData("Green", "%.3f", greenPercentage)
                    .addData("Blue", "%.3f", bluePercentage);
            //COlor senosr 2
            telemetry.addData("Pixel in Slot 2: ",isPixelIn2);
            telemetry.addData("Pixel Color in Slot 2: ",colorIn2);
            telemetry.addData("distance",((DistanceSensor) color2).getDistance(DistanceUnit.INCH));
            telemetry.addLine()
                    .addData("Red", "%.3f", redPercentage2)
                    .addData("Green", "%.3f", greenPercentage2)
                    .addData("Blue", "%.3f", bluePercentage2);
            telemetry.update();
            drive.update();

            // Retrieve your pose
            Pose2d myPose = drive.getPoseEstimate();




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
            if (linearSlideLeft.getCurrentPosition()>currentPositionLin) {
                linearSlideMode = true;
            } else {
                linearSlideMode = false;
            }


            //Linear Slide joystick controller 2
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
            } else if (gamepad2.left_trigger != 0) {
                linearSlideLeft.setPower(-gamepad2.left_trigger);
                linearSlideRight.setPower(-gamepad2.left_trigger);
            } else if (linearSlideMode) {
                linearSlideLeft.setPower(0.1);
                linearSlideRight.setPower(0.1);
            } else {
                linearSlideLeft.setPower(0);
                linearSlideRight.setPower(0);
            }

            if (gamepad2.a) {
                colorRuntime = getRuntime();
                if(isPixelIn1&&isPixelIn2){
                    intakeMotor.setPower(-0.42);
                } else {
                    intakeMotor.setPower(0.9);
                }
                pixelIn.setPower(1);
                pixelOut.setPosition(.8);
                intakeRotate.setPower(-0.5);
                intakeMove.setPower(0);
            } else if (gamepad2.b) {
                intakeMotor.setPower(-0.8);
                pixelIn.setPower(1);
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
            if (gamepad2.start){
                mosaicMover.setPower(1);
            } else if (gamepad2.back){
                mosaicMover.setPower(-1);
            } else{
                mosaicMover.setPower(0);
            }

            if (gamepad2.left_stick_y>0) {
                intakeRotate.setPower(-1);
                intakeMove.setPower(gamepad2.left_stick_y*0.3);
            } else if (gamepad2.left_stick_y<0) {
                intakeRotate.setPower(1);
                intakeMove.setPower(gamepad2.left_stick_y*0.35);
            } else if (gamepad2.dpad_up) {
                intakeRotate.setPower(0.6);
                intakeMove.setPower(-0.13);
            } else if (gamepad2.dpad_down) {
                intakeRotate.setPower(-0.6);
                intakeMove.setPower(-0.1);
            } else {
                intakeRotate.setPower(0);
                intakeMove.setPower(0);
            }
            if (gamepad2.right_bumper){
                stackKnocker.setPower(0.4);
            } else if (gamepad2.left_bumper){
                stackKnocker.setPower(-0.4);
            } else {
                stackKnocker.setPower(0);
            }
            if (getRuntime()-colorRuntime<=1.5){
                fingers = true;
            } else {
                fingers = false;
            }





            if (gamepad1.right_bumper) {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            } else if (gamepad2.right_stick_x >= 0.5) {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
            } else if (gamepad2.right_stick_x <= -0.5) {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
            } else if (gamepad2.right_stick_y >= 0.5) {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            } else if (gamepad2.right_stick_y <= -0.5) {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
            } else if (fingers) {

                if (intendedColor == "red"){
                    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_RED);
                } else if (intendedColor == "yellow"){
                    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                } else if (intendedColor == "green"){
                    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.SKY_BLUE);
                }
            } else if (colorIn1 == "white!") {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
            }else if (colorIn1 == "yellow!") {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
            } else if (colorIn1 == "purple!") {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
            } else if (colorIn1 == "green!") {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            }
            else {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_PARTY_PALETTE);
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

