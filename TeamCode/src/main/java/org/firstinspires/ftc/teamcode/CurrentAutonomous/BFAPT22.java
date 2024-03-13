/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.CurrentAutonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.LinearSlidePosition;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

/*
 * This OpMode illustrates the basics of TensorFlow Object Detection,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@Autonomous(name = "BFAPT22")

public class BFAPT22 extends LinearOpMode {


    int auto =3;
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    public IMU imu;

    private static NormalizedColorSensor color1;
    private static NormalizedColorSensor color2;
    int numberInOne;
    int numberInTwo;

    RevBlinkinLedDriver.BlinkinPattern help;


    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal myVisionPortal;
    private static final String TFOD_MODEL_ASSET = "Meet1Blue.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/myCustomModel.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
       "blue",
    };

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */


    /**
     * The variable to store our instance of the vision portal.
     */


    @Override
    public void runOpMode() {
        initDoubleVision();
        CRServo intakeMove = hardwareMap.crservo.get("intakeMove");
        imu = hardwareMap.get(IMU.class, "imu");
        CRServo intakeRotate = hardwareMap.crservo.get("intakeRotate");
        color1 = hardwareMap.get(NormalizedColorSensor.class, "color");
        color2 = hardwareMap.get(NormalizedColorSensor.class, "color2");
        RevBlinkinLedDriver lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
        Servo pixelOut = hardwareMap.servo.get("pixelOut");
        CRServo pixelIn = hardwareMap.crservo.get("pixelIn");
        DcMotor intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        NormalizedColorSensor color1 = hardwareMap.get(NormalizedColorSensor.class, "color");
        NormalizedColorSensor color2 = hardwareMap.get(NormalizedColorSensor.class, "color2");
        CRServo stackKnocker = hardwareMap.crservo.get("stackKnocker");
        DcMotor linearSlideLeft = hardwareMap.dcMotor.get("linearSlideLeft");
        DcMotor linearSlideRight = hardwareMap.dcMotor.get("linearSlideRight");
        linearSlideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlideLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        linearSlideRight.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMove.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        imu.initialize(
                new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,RevHubOrientationOnRobot.UsbFacingDirection.LEFT)
                ));
        double xforlast = 0;
        double yforlast = 0;

        int idforapril = 0;
        boolean hello = false;


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose =new Pose2d(0, 0, Math.toRadians(0));
        Pose2d backPose =new Pose2d(96,26, Math.toRadians(0));
        Pose2d backPose1 =new Pose2d(96,20, Math.toRadians(0));
        Pose2d backPose3 =new Pose2d(96,32, Math.toRadians(0));

        drive.setPoseEstimate(startPose);
        Trajectory t3 = drive.trajectoryBuilder(new Pose2d())
                .lineToConstantHeading(new Vector2d(-12,0))

                .build();
        Trajectory t2 = drive.trajectoryBuilder(new Pose2d())
                .lineToConstantHeading(new Vector2d(-14,0))

                .build();
        Trajectory t1 = drive.trajectoryBuilder(new Pose2d())
                .lineToConstantHeading(new Vector2d(-14,0))

                .build();
        TrajectorySequence trajSeq2back =drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(12,26))
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> intakeMove.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> intakeRotate.setPower(-0.1))
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> intakeMove.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> intakeRotate.setPower(-0.1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> pixelIn.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeRotate.setPower(-0.1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMove.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMove.setPower(1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMove.setPower(1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(0.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(0.5))
                .lineToConstantHeading(new Vector2d(110,26))
                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> stackKnocker.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(-0.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(-0.5))
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> stackKnocker.setPower(-0.5))
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> stackKnocker.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeRotate.setPower(0.1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeRotate.setPower(0.1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(-0.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(-0.5))
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> stackKnocker.setPower(-0.5))
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> intakeMotor.setPower(0.9))
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> intakeMotor.setPower(0.9))
                .lineToConstantHeading(new Vector2d(115.6,26))
                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> intakeMotor.setPower(0.9))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMotor.setPower(0.9))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeRotate.setPower(0.1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeRotate.setPower(0.1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> pixelIn.setPower(-1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMove.setPower(0.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMove.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeRotate.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMotor.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> pixelIn.setPower(-1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> pixelIn.setPower(-1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMotor.setPower(0.9))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMotor.setPower(0.9))
                .lineToConstantHeading(new Vector2d(96,26))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> checkForColor())
                .UNSTABLE_addTemporalMarkerOffset(0, () ->  lights.setPattern(help))
                .build();
        TrajectorySequence getPixel = drive.trajectorySequenceBuilder(backPose)
                .splineToConstantHeading((new Vector2d(112,16)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMotor.setPower(0.9))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeRotate.setPower(-0.3))//-0.1
                .UNSTABLE_addTemporalMarkerOffset(0, () -> pixelIn.setPower(-1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMove.setPower(0.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMove.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeRotate.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMotor.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> pixelIn.setPower(-1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> pixelIn.setPower(-1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMotor.setPower(0.9))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMotor.setPower(0.9))
                .waitSeconds(0.5)
                .splineToConstantHeading((new Vector2d(96,26)), Math.toRadians(0))
                .build();
        TrajectorySequence trajSeq2Score =drive.trajectorySequenceBuilder(backPose)
                .lineToConstantHeading(new Vector2d(7.6,26))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(-1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMotor.setPower(-1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMotor.setPower(-1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMotor.setPower(-1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMove.setPower(-0.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> pixelIn.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> pixelIn.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeRotate.setPower(-0.1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> linearSlideLeft.setPower(0.6))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> linearSlideRight.setPower(0.6))
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> linearSlideLeft.setPower(0.1))
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> linearSlideRight.setPower(0.1)).UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMove.setPower(-0.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> pixelIn.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> pixelIn.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeRotate.setPower(-0.1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> linearSlideLeft.setPower(0.6))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> linearSlideRight.setPower(0.6))
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> linearSlideLeft.setPower(0.1))
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> linearSlideRight.setPower(0.1))
                .lineToConstantHeading(new Vector2d(0,6))
                .build();

        //Position 3 RFAPT 2+2 Back
        TrajectorySequence trajSeq3back =drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(12,32))
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> intakeMove.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> intakeRotate.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> intakeMove.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> intakeRotate.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> pixelIn.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeRotate.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMove.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMove.setPower(1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMove.setPower(1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(0.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(0.5))
                .lineToConstantHeading(new Vector2d(110,32))
                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> stackKnocker.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(-0.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(-0.5))
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> stackKnocker.setPower(-0.5))
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> stackKnocker.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeRotate.setPower(0.1))//0.5 or 0.1
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeRotate.setPower(0.1))//0.5 or 0.1
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(-0.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(-0.5))
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> stackKnocker.setPower(-0.5))
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> intakeMotor.setPower(0.9))
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> intakeMotor.setPower(0.9))
                .lineToConstantHeading(new Vector2d(115.6,32))
                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> intakeMotor.setPower(0.9))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMotor.setPower(0.9))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeRotate.setPower(0.1))//0.5 or 0.1
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeRotate.setPower(0.1))//0.5 or 0.1
                .UNSTABLE_addTemporalMarkerOffset(0, () -> pixelIn.setPower(-1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMove.setPower(0.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMove.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeRotate.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMotor.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> pixelIn.setPower(-1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> pixelIn.setPower(-1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMotor.setPower(0.9))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMotor.setPower(0.9))
                .lineToConstantHeading(new Vector2d(96,32))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> checkForColor())
                .UNSTABLE_addTemporalMarkerOffset(0, () ->  lights.setPattern(help))
                .build();
        TrajectorySequence getPixel3 = drive.trajectorySequenceBuilder(backPose3)
                .splineToConstantHeading((new Vector2d(112,22)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMotor.setPower(0.9))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeRotate.setPower(-0.3))//0.3 or 0.1
                .UNSTABLE_addTemporalMarkerOffset(0, () -> pixelIn.setPower(-1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMove.setPower(0.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMove.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeRotate.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMotor.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> pixelIn.setPower(-1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> pixelIn.setPower(-1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMotor.setPower(0.9))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMotor.setPower(0.9))
                .waitSeconds(0.5)
                .splineToConstantHeading((new Vector2d(96,32)), Math.toRadians(0))
                .build();
        TrajectorySequence trajSeq3Score =drive.trajectorySequenceBuilder(backPose3)
                .lineToConstantHeading(new Vector2d(7.6,32))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(-1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMotor.setPower(-1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMotor.setPower(-1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMotor.setPower(-1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMove.setPower(-0.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> pixelIn.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> pixelIn.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeRotate.setPower(-0.1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> linearSlideLeft.setPower(0.6))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> linearSlideRight.setPower(0.6))
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> linearSlideLeft.setPower(0.1))
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> linearSlideRight.setPower(0.1)).UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMove.setPower(-0.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> pixelIn.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> pixelIn.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeRotate.setPower(-0.1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> linearSlideLeft.setPower(0.6))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> linearSlideRight.setPower(0.6))
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> linearSlideLeft.setPower(0.1))
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> linearSlideRight.setPower(0.1))
                .lineToConstantHeading(new Vector2d(0,9.5))
                .build();

        //Position 1 RFAPT 2+2 Back
        TrajectorySequence trajSeq1back =drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(12,20))
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> intakeMove.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> intakeRotate.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> intakeMove.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> intakeRotate.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> pixelIn.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeRotate.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMove.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMove.setPower(1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMove.setPower(1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(0.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(0.5))
                .lineToConstantHeading(new Vector2d(110,20))
                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> stackKnocker.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(-0.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(-0.5))
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> stackKnocker.setPower(-0.5))
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> stackKnocker.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeRotate.setPower(0.1)) // 0.1
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeRotate.setPower(0.1)) //0.1
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(-0.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(-0.5))
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> stackKnocker.setPower(-0.5))
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> intakeMotor.setPower(0.9))
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> intakeMotor.setPower(0.9))
                .lineToConstantHeading(new Vector2d(115.6,20))
                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> intakeMotor.setPower(0.9))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMotor.setPower(0.9))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeRotate.setPower(0.1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeRotate.setPower(0.1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> pixelIn.setPower(-1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMove.setPower(0.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMove.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeRotate.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMotor.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> pixelIn.setPower(-1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> pixelIn.setPower(-1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMotor.setPower(0.9))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMotor.setPower(0.9))
                .lineToConstantHeading(new Vector2d(96,20))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> checkForColor())
                .UNSTABLE_addTemporalMarkerOffset(0, () ->  lights.setPattern(help))
                .build();
        TrajectorySequence getPixel1 = drive.trajectorySequenceBuilder(backPose1)
                .splineToConstantHeading((new Vector2d(112,10)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMotor.setPower(0.9))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeRotate.setPower(0.1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> pixelIn.setPower(-1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMove.setPower(0.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMove.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeRotate.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMotor.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> pixelIn.setPower(-1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> pixelIn.setPower(-1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMotor.setPower(0.9))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMotor.setPower(0.9))
                .waitSeconds(0.5)
                .splineToConstantHeading((new Vector2d(96,20)), Math.toRadians(0))
                .build();
        TrajectorySequence trajSeq1Score =drive.trajectorySequenceBuilder(backPose1)
                .lineToConstantHeading(new Vector2d(7.6,20))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(-1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMotor.setPower(-1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMotor.setPower(-1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMotor.setPower(-1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMove.setPower(-0.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> pixelIn.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> pixelIn.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeRotate.setPower(-0.1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> linearSlideLeft.setPower(0.6))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> linearSlideRight.setPower(0.6))
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> linearSlideLeft.setPower(0.1))
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> linearSlideRight.setPower(0.1)).UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMove.setPower(-0.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> pixelIn.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> pixelIn.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeRotate.setPower(-0.1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> linearSlideLeft.setPower(0.6))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> linearSlideRight.setPower(0.6))
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> linearSlideLeft.setPower(0.1))
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> linearSlideRight.setPower(0.1))
                .lineToConstantHeading(new Vector2d(0,-6))
                .build();
        TrajectorySequence getStack1 = drive.trajectorySequenceBuilder(backPose1)
                .lineToConstantHeading(new Vector2d(110,16))
                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> stackKnocker.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(-0.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(-0.5))
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> stackKnocker.setPower(-0.5))
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> stackKnocker.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeRotate.setPower(0.1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeRotate.setPower(0.1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(-0.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(-0.5))
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> stackKnocker.setPower(-0.5))
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> intakeMotor.setPower(0.9))
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> intakeMotor.setPower(0.9))
                .lineToConstantHeading(new Vector2d(114.6,16))
                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> intakeMotor.setPower(0.9))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMotor.setPower(0.9))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeRotate.setPower(0.1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeRotate.setPower(0.1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> pixelIn.setPower(-1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMove.setPower(0.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMove.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeRotate.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMotor.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> pixelIn.setPower(-1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> pixelIn.setPower(-1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMotor.setPower(0.9))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMotor.setPower(0.9))
                .lineToConstantHeading(new Vector2d(96,20))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> checkForColor())
                .UNSTABLE_addTemporalMarkerOffset(0, () ->  lights.setPattern(help))
                .build();

        TrajectorySequence trajSeq2 =drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-37.5,0, Math.toRadians(-90)))
                .forward(6)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMove.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> intakeMove.setPower(-0.9))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> intakeRotate.setPower(-0.1))
                .lineToSplineHeading(new Pose2d(-24,-24, Math.toRadians(90)))
                .build();
        TrajectorySequence trajSeq1 =drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-28.5,-12, Math.toRadians(-90)))
                .forward(6)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMove.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> intakeMove.setPower(-0.9))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> intakeRotate.setPower(-0.1))
                .lineToSplineHeading(new Pose2d(-18,-24, Math.toRadians(90)))
                .build();
        TrajectorySequence trajSeq3 =drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-28.5,10, Math.toRadians(-90)))
                .forward(6)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMove.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> intakeMove.setPower(-0.9))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> intakeRotate.setPower(-0.1))
                .lineToSplineHeading(new Pose2d(-30,-24, Math.toRadians(90)))
                .build();


        Trajectory traj1a = drive.trajectoryBuilder(new Pose2d())
                .lineToConstantHeading(new Vector2d(-22,-6.6))

                .build();
        Trajectory trajddd = drive.trajectoryBuilder(new Pose2d())

                .lineToConstantHeading (new Vector2d(4,0))

                .build();
        Trajectory traj1b = drive.trajectoryBuilder(new Pose2d())
                .lineToConstantHeading(new Vector2d(10,0))
                .build();
        Trajectory traj1c = drive.trajectoryBuilder(new Pose2d())

                .lineToSplineHeading (new Pose2d(0,-10,Math.toRadians(90)))

                .build();
        Trajectory traj1d = drive.trajectoryBuilder(new Pose2d())

                .lineToConstantHeading (new Vector2d(-8,9))

                .build();

        Trajectory traj1e = drive.trajectoryBuilder(new Pose2d())

                .lineToSplineHeading (new Pose2d(11,0,Math.toRadians(1)))

                .build();
        Trajectory traj3e = drive.trajectoryBuilder(new Pose2d())

                .lineToSplineHeading (new Pose2d(11,0,Math.toRadians(1)))

                .build();
        Trajectory trajd = drive.trajectoryBuilder(new Pose2d())

                .lineToConstantHeading (new Vector2d(-3,0))

                .build();
        Trajectory traj1f = drive.trajectoryBuilder(new Pose2d())

                .lineToConstantHeading (new Vector2d(0,-16.5))

                .build();
        Trajectory traj2f = drive.trajectoryBuilder(new Pose2d())

                .lineToConstantHeading (new Vector2d(0,-23))

                .build();
        Trajectory traj3f = drive.trajectoryBuilder(new Pose2d())

                .lineToConstantHeading (new Vector2d(0,-29))

                .build();
        Trajectory traj1g = drive.trajectoryBuilder(new Pose2d())

                .lineToConstantHeading (new Vector2d(-10,0))

                .build();
        Trajectory traj3g = drive.trajectoryBuilder(new Pose2d())

                .lineToConstantHeading (new Vector2d(-15,0))

                .build();
        Trajectory traj2a = drive.trajectoryBuilder(new Pose2d())
                .lineToConstantHeading(new Vector2d(-31,0))
                .build();
        Trajectory traj2b = drive.trajectoryBuilder(new Pose2d())
                .lineToConstantHeading(new Vector2d(15,0))
                .build();
        Trajectory traj2c = drive.trajectoryBuilder(new Pose2d())

                .lineToSplineHeading (new Pose2d(0,-10,Math.toRadians(90)))

                .build();
        Trajectory traj2d = drive.trajectoryBuilder(new Pose2d())

                .lineToConstantHeading (new Vector2d(-15,9))

                .build();
        Trajectory traj3a = drive.trajectoryBuilder(new Pose2d())
                .lineToSplineHeading(new Pose2d(-27,7.3,Math.toRadians(-90)))

                .build();
        Trajectory traj3b = drive.trajectoryBuilder(new Pose2d())

                .lineToConstantHeading (new Vector2d(27,-3.5))

                .build();
        Trajectory traj3c = drive.trajectoryBuilder(new Pose2d())

                .lineToSplineHeading (new Pose2d(10,1.5,Math.toRadians(-177)))

                .build();

        Trajectory trajApril = drive.trajectoryBuilder(new Pose2d())
                .lineToConstantHeading(new Vector2d(1,0))

                .build();
        boolean d= true;
        while (!opModeIsActive()) {
            telemetryTfod();
        }




        waitForStart();

        if (opModeIsActive()) {

            if (auto == 2 || auto == 1) {
                telemetry.addData("auto", auto);
                telemetry.update();
            } else {
                resetRuntime();
                while (getRuntime() < 1 && auto == 3) {
                    telemetryTfod();

                    telemetry.update();
                }
            }
        }
        telemetry.addData("auto",auto);

        telemetry.update();
        resetRuntime();


        if (auto ==1){

            telemetry.update();
            drive.followTrajectorySequence(trajSeq1);
            drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
            resetRuntime();
            while (getRuntime()<1) {
                if (aprilTag.getDetections().size() > 0) {
                    for (int i = 0; i < aprilTag.getDetections().size(); i++) {
                        AprilTagDetection tag = aprilTag.getDetections().get(i);
                        idforapril = tag.id;
                        xforlast = -1 * tag.ftcPose.y;
                        yforlast = tag.ftcPose.x;

                        if (idforapril == 1) {
                            trajApril = drive.trajectoryBuilder(new Pose2d())
                                    .lineToConstantHeading(new Vector2d(xforlast-3.5,yforlast))

                                    .build();
                            hello = true;
                        }
                    }


                    if (hello) {
                        drive.followTrajectory(trajApril);
                    }

                }
            }
            if(!hello){
                drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
                sleep(200);
                drive.followTrajectory(t1);
                drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
                sleep(200);
                drive.followTrajectory(traj1d);
            }

            pixelOut.setPosition(0);
            pixelIn.setPower(-1);
            intakeRotate.setPower(-0.1);
            sleep(200);
            pixelIn.setPower(0);
            intakeMove.setPower(0);
            intakeRotate.setPower(0);
            pixelOut.setPosition(0.8);
            sleep(100);
            intakeMove.setPower(0);
            sleep(100);
            intakeMove.setPower(1);
            intakeMove.setPower(1);
            sleep(100);
            hello = false;
            drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
            drive.followTrajectorySequence(trajSeq3back);
            checkForColor();
            lights.setPattern(help);
            if(help != RevBlinkinLedDriver.BlinkinPattern.GREEN){
                drive.followTrajectorySequence(getPixel3);
            }

            checkForColor();
            lights.setPattern(help);
            resetRuntime();
            intakeMotor.setPower(-0.3-getRuntime()*0.7);
            drive.followTrajectorySequence(trajSeq3Score);
            frontLeftMotor.setPower(-0.2);
            frontRightMotor.setPower(-0.2);
            backLeftMotor.setPower(-0.2);
            backRightMotor.setPower(-0.2);
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            sleep(500);
            pixelOut.setPosition(0);
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            intakeRotate.setPower(-0.1);
            sleep(100);
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            sleep(200);
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            pixelIn.setPower(-1);
            drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
            sleep(800);
            drive.followTrajectory(trajddd);
            pixelOut.setPosition(0.8);
            linearSlideLeft.setPower(-0.5);
            linearSlideRight.setPower(-0.5);
            sleep(600);
            linearSlideLeft.setPower(0);
            linearSlideRight.setPower(0);
            LinearSlidePosition.pos = linearSlideLeft.getCurrentPosition();
            intakeMove.setPower(0);
            sleep(500);
            intakeMove.setPower(1);
            sleep(1000);
            intakeMove.setPower(0);


        }
        if (auto ==2){

            telemetry.update();
            drive.followTrajectorySequence(trajSeq2);
            drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
            resetRuntime();
            while (getRuntime()<1) {
                if (aprilTag.getDetections().size() > 0) {
                    for (int i = 0; i < aprilTag.getDetections().size(); i++) {
                        AprilTagDetection tag = aprilTag.getDetections().get(i);
                        idforapril = tag.id;
                        xforlast = -1 * tag.ftcPose.y;
                        yforlast = tag.ftcPose.x;

                        if (idforapril == 2) {
                            trajApril = drive.trajectoryBuilder(new Pose2d())
                                    .lineToConstantHeading(new Vector2d(xforlast-3.5,yforlast))

                                    .build();
                            hello = true;
                        }
                    }


                    if (hello) {
                        drive.followTrajectory(trajApril);
                    }

                }
            }
            if(!hello){
                drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
                sleep(200);
                drive.followTrajectory(t2);
                drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
                sleep(200);
                drive.followTrajectory(traj1d);
            }

            pixelOut.setPosition(0);
            pixelIn.setPower(-1);
            intakeRotate.setPower(-0.1);
            sleep(200);
            pixelIn.setPower(0);
            intakeMove.setPower(0);
            intakeRotate.setPower(0);
            pixelOut.setPosition(0.8);
            sleep(100);
            intakeMove.setPower(0);
            sleep(100);
            intakeMove.setPower(1);
            intakeMove.setPower(1);
            sleep(100);
            hello = false;
            drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
            drive.followTrajectorySequence(trajSeq2back);
            checkForColor();
            lights.setPattern(help);
            if(help != RevBlinkinLedDriver.BlinkinPattern.GREEN){
                drive.followTrajectorySequence(getPixel);
            }
            checkForColor();
            lights.setPattern(help);
            resetRuntime();
            intakeMotor.setPower(-0.3-getRuntime()*0.7);
            drive.followTrajectorySequence(trajSeq2Score);
            frontLeftMotor.setPower(-0.2);
            frontRightMotor.setPower(-0.2);
            backLeftMotor.setPower(-0.2);
            backRightMotor.setPower(-0.2);
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            sleep(500);
            pixelOut.setPosition(0);
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            intakeRotate.setPower(-0.1);
            sleep(100);
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            sleep(200);
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            pixelIn.setPower(-1);
            drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
            sleep(800);
            drive.followTrajectory(trajddd);
            pixelOut.setPosition(0.8);
            linearSlideLeft.setPower(-0.5);
            linearSlideRight.setPower(-0.5);
            sleep(600);
            linearSlideLeft.setPower(0);
            linearSlideRight.setPower(0);
            LinearSlidePosition.pos = linearSlideLeft.getCurrentPosition();
            intakeMove.setPower(0);
            intakeMove.setPower(1);
            sleep(1000);
            intakeMove.setPower(0);





        } if (auto ==3){
            telemetry.update();
            drive.followTrajectorySequence(trajSeq3);
            drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
            resetRuntime();
            while (getRuntime()<1) {
                if (aprilTag.getDetections().size() > 0) {
                    for (int i = 0; i < aprilTag.getDetections().size(); i++) {
                        AprilTagDetection tag = aprilTag.getDetections().get(i);
                        idforapril = tag.id;
                        xforlast = -1 * tag.ftcPose.y;
                        yforlast = tag.ftcPose.x;

                        if (idforapril == 3) {
                            trajApril = drive.trajectoryBuilder(new Pose2d())
                                    .lineToConstantHeading(new Vector2d(xforlast-3.5,yforlast))

                                    .build();
                            hello = true;
                        }
                    }


                    if (hello) {
                        drive.followTrajectory(trajApril);
                    }

                }
            }
            if(!hello){
                drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
                sleep(200);
                drive.followTrajectory(t3);
                drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
                sleep(200);
                drive.followTrajectory(traj1d);
            }

            pixelOut.setPosition(0);
            pixelIn.setPower(-1);
            intakeRotate.setPower(-0.1);
            sleep(200);
            pixelIn.setPower(0);
            intakeMove.setPower(0);
            intakeRotate.setPower(0);
            pixelOut.setPosition(0.8);
            sleep(100);
            intakeMove.setPower(0);
            sleep(100);
            intakeMove.setPower(1);
            intakeMove.setPower(1);
            sleep(100);
            hello = false;
            drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
            drive.followTrajectorySequence(trajSeq1back);
            checkForColor();
            lights.setPattern(help);
            if(help != RevBlinkinLedDriver.BlinkinPattern.GREEN){
                drive.followTrajectorySequence(getPixel1);
            }
            checkForColor();
            lights.setPattern(help);
            resetRuntime();
            intakeMotor.setPower(-0.3-getRuntime()*0.7);
            drive.followTrajectorySequence(trajSeq1Score);
            frontLeftMotor.setPower(-0.2);
            frontRightMotor.setPower(-0.2);
            backLeftMotor.setPower(-0.2);
            backRightMotor.setPower(-0.2);
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            sleep(500);
            pixelOut.setPosition(0);
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            intakeRotate.setPower(-0.1);
            sleep(100);
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            sleep(200);
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            pixelIn.setPower(-1);
            drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
            sleep(800);
            drive.followTrajectory(trajddd);
            pixelOut.setPosition(0.8);
            linearSlideLeft.setPower(-0.5);
            linearSlideRight.setPower(-0.5);
            sleep(600);
            linearSlideLeft.setPower(0);
            linearSlideRight.setPower(0);
            LinearSlidePosition.pos = linearSlideLeft.getCurrentPosition();
            intakeMove.setPower(0);
            sleep(500);
            intakeMove.setPower(1);
            sleep(1000);
            intakeMove.setPower(0);
        }

        // Save more CPU resources when camera is no longer needed.



    }   // end runOpMode()

    /**
     * Initialize the TensorFlow Object Detection processor.
     */


    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private void telemetryTfod() {
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop
        if (currentRecognitions.size()!=0){
            for (Recognition recognition : currentRecognitions) {
                        if((recognition.getImageHeight()<=1.5* recognition.getImageWidth())&&(recognition.getImageWidth()<=1.5* recognition.getImageHeight())&&(recognition.getConfidence()>0.87)){
                            if (((recognition.getLeft()+ recognition.getRight())/2)<200){
                                auto=1;
                            } else {
                                auto = 2;
                            }

                        } else {
                            idle();
                        }

                telemetry.addData("auto",auto);
                telemetry.addData("confonf",recognition.getConfidence());
                telemetry.update();
            }   // end for() loop

        }else {
            auto =3;
            telemetry.addData("auto",auto);
            telemetry.update();
        }

    }   // end method telemetryTfod()
    private void initDoubleVision() {
        // -----------------------------------------------------------------------------------------
        // AprilTag Configuration
        // -----------------------------------------------------------------------------------------

        aprilTag = new AprilTagProcessor.Builder()
                .build();

        // -----------------------------------------------------------------------------------------
        // TFOD Configuration
        // -----------------------------------------------------------------------------------------

        tfod = new TfodProcessor.Builder()
                .setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                .build();

        // -----------------------------------------------------------------------------------------
        // Camera Configuration
        // -----------------------------------------------------------------------------------------

        if (USE_WEBCAM) {
            myVisionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessors(tfod, aprilTag)
                    .build();
        } else {
            myVisionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                    .addProcessors(tfod, aprilTag)
                    .build();
        }
    }   // end initDoubleVision()
    private void checkForColor(){
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
            help =  RevBlinkinLedDriver.BlinkinPattern.DARK_RED;
        }
        if (numberInTwo+numberInOne==1){
            help =  RevBlinkinLedDriver.BlinkinPattern.YELLOW;

        }
        if (numberInTwo+numberInOne==2){
            help =  RevBlinkinLedDriver.BlinkinPattern.GREEN;

        }
    }
}   // end class
