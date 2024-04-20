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

package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
@Autonomous(name = "BBAPT_SPEED")

public class BBAPT_SPEED extends LinearOpMode {


    int auto =1;
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

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
            CRServo intakeRotate = hardwareMap.crservo.get("intakeRotate");
            Servo pixelOut = hardwareMap.servo.get("pixelOut");
            CRServo pixelIn = hardwareMap.crservo.get("pixelIn");
            DcMotor intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
            DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
            DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
            DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
            DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
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



        double xforlast = 0;
        double yforlast = 0;
        int idforapril = 0;
        boolean hello = false;


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose =new Pose2d(0, 0, Math.toRadians(0));
        Pose2d speedPose =new Pose2d(-16.849, 1.112, Math.toRadians(0));


        TrajectorySequence trajSeq1 =drive.trajectorySequenceBuilder(startPose)
                .back(6)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMove.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(1))
                .lineToSplineHeading(new Pose2d(-28.5,-11, Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(-25.5,16))
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> stackKnocker.setPower(-0.4))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMotor.setPower(0.9))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeRotate.setPower(0.1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> pixelIn.setPower(-1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMove.setPower(0.5))
                .forward(3.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMove.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeRotate.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> pixelIn.setPower(-1))
                .back(16)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMotor.setPower(-1))
                .strafeLeft(26)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMotor.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> pixelIn.setPower(0))
                .back(86.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(-0.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMove.setPower(-0.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeRotate.setPower(-0.1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> linearSlideLeft.setPower(0.4))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> linearSlideRight.setPower(0.4))
                //.waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> linearSlideLeft.setPower(0.1))
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> linearSlideRight.setPower(0.1))
                .waitSeconds(1)
                .strafeRight(34)
                .build();
        TrajectorySequence trajSeq3 =drive.trajectorySequenceBuilder(startPose)
                .back(6)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMove.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(1))
                .lineToSplineHeading(new Pose2d(-28.5,11, Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(-25.5,16))
                .waitSeconds(0.7)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> stackKnocker.setPower(-0.35))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMotor.setPower(0.9))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeRotate.setPower(0.1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> pixelIn.setPower(-1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMove.setPower(0.5))
                .forward(3.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMove.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeRotate.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> pixelIn.setPower(-1))
                .back(5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMotor.setPower(-1))
                .lineToConstantHeading(new Vector2d(-51.5,7.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMotor.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> pixelIn.setPower(0))
                .back(84.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMove.setPower(-0.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeRotate.setPower(-0.1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> linearSlideLeft.setPower(0.4))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> linearSlideRight.setPower(0.4))
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> linearSlideLeft.setPower(0.1))
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> linearSlideRight.setPower(0.1))
                .strafeRight(22)
                .build();
        TrajectorySequence trajSeq2 =drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-37.5,0, Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMove.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(1))
                .lineToConstantHeading(new Vector2d(-37.5,16))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(-1))
                .waitSeconds(0.7)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMotor.setPower(0.9))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeRotate.setPower(0.1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> pixelIn.setPower(-1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMove.setPower(0.5))
                .forward(3.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMove.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeRotate.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> pixelIn.setPower(-1))
                .back(7)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMotor.setPower(-1))
                .strafeLeft(14)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMotor.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> pixelIn.setPower(0))
                .back(91.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMove.setPower(-0.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeRotate.setPower(-0.1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> linearSlideLeft.setPower(0.4))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> linearSlideRight.setPower(0.4))
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> linearSlideLeft.setPower(0.1))
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> linearSlideRight.setPower(0.1))
                .strafeRight(28)
                .build();
        TrajectorySequence trajSeq2back =drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(12+16.849,24.5-1.112))
                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> linearSlideLeft.setPower(-1))
                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> linearSlideRight.setPower(-1))
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> intakeMove.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> intakeRotate.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> intakeMove.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> intakeRotate.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> pixelIn.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> pixelOut.setPosition(0.8))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeRotate.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMove.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMove.setPower(1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMove.setPower(1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> linearSlideLeft.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> linearSlideRight.setPower(0))
                .forward(83)
                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> stackKnocker.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(0.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(0.5))
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> stackKnocker.setPower(0.5))
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> stackKnocker.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(0.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(0.5))
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> stackKnocker.setPower(0.5))
                .forward(3.1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMotor.setPower(0.9))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeRotate.setPower(-0.3))
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
                .strafeRight(10)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMotor.setPower(0.9))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMotor.setPower(0.9))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> pixelIn.setPower(-1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> pixelIn.setPower(-1))
                .strafeLeft(10)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMotor.setPower(0.9))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMotor.setPower(0.9))
                .back(92)
                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> stackKnocker.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(-0.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(-0.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMotor.setPower(-1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMotor.setPower(-1))
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> intakeMotor.setPower(-1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMove.setPower(-0.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> pixelIn.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> pixelIn.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeRotate.setPower(-0.1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> linearSlideLeft.setPower(0.4))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> linearSlideRight.setPower(0.4))
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> linearSlideLeft.setPower(0.1))
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> linearSlideRight.setPower(0.1))
                .lineToConstantHeading(new Vector2d(16.849,6-1.112))
                .build();
        TrajectorySequence trajSeq1back =drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> linearSlideLeft.setPower(-1))
                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> linearSlideRight.setPower(-1))
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(11+16.849,35.2-1.112))//12 //36
                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> linearSlideLeft.setPower(-1))
                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> linearSlideRight.setPower(-1))
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> intakeMove.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> intakeRotate.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> intakeMove.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> intakeRotate.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> pixelIn.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> pixelOut.setPosition(0.8))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeRotate.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMove.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMove.setPower(1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMove.setPower(1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> linearSlideLeft.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> linearSlideRight.setPower(0))
                .forward(82)
                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> stackKnocker.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(0.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(0.5))
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> stackKnocker.setPower(0.5))
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> stackKnocker.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(0.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(0.5))
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> stackKnocker.setPower(0.5))
                .forward(4.2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMotor.setPower(0.9))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeRotate.setPower(-0.3))
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
                .back(100)
                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> stackKnocker.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(-0.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(-0.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMotor.setPower(-1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMotor.setPower(-1))
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> intakeMotor.setPower(-1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMove.setPower(-0.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> pixelIn.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> pixelIn.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeRotate.setPower(-0.1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> linearSlideRight.setPower(0.4))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> linearSlideRight.setPower(0.4))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> linearSlideLeft.setPower(0.1))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> linearSlideRight.setPower(0.1))
                .lineToConstantHeading(new Vector2d(16.849,12-1.112))
                .build();
        TrajectorySequence trajSeq3back =drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(12+16.849,19-1.112))
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> intakeMove.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> intakeRotate.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> intakeMove.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> intakeRotate.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> pixelIn.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> pixelOut.setPosition(0.8))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeRotate.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMove.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMove.setPower(1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMove.setPower(1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> linearSlideLeft.setPower(-0.4))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> linearSlideRight.setPower(-0.4))
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> linearSlideLeft.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> linearSlideRight.setPower(0))
                .forward(82)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> intakeMotor.setPower(0.9))
                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> stackKnocker.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(0.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(0.5))
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> stackKnocker.setPower(0.5))
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> stackKnocker.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(0.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(0.5))
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> stackKnocker.setPower(0.5))
                .forward(3.8)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMotor.setPower(0.9))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeRotate.setPower(-0.3))
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
                .strafeRight(10)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMotor.setPower(0.9))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMotor.setPower(0.9))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> pixelIn.setPower(-1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> pixelIn.setPower(-1))
                .strafeLeft(10)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMotor.setPower(0.9))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMotor.setPower(0.9))
                .back(92)
                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> stackKnocker.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(-0.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> stackKnocker.setPower(-0.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMotor.setPower(-1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMotor.setPower(-1))
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> intakeMotor.setPower(-1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeMove.setPower(-0.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> pixelIn.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> pixelIn.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intakeRotate.setPower(-0.1))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> linearSlideRight.setPower(0.4))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> linearSlideRight.setPower(0.4))
                .UNSTABLE_addTemporalMarkerOffset(0.45, () -> linearSlideLeft.setPower(0.1))
                .UNSTABLE_addTemporalMarkerOffset(0.45, () -> linearSlideRight.setPower(0.1))
                .lineToConstantHeading(new Vector2d(16.849,-1.112))
                .build();
        Trajectory traj1a = drive.trajectoryBuilder(new Pose2d())
                .lineToSplineHeading(new Pose2d(-28.5,-9,Math.toRadians(90)))

                .build();
        Trajectory traj1b = drive.trajectoryBuilder(new Pose2d())

                .lineToConstantHeading (new Vector2d(9,0))

                .build();
        Trajectory trajz = drive.trajectoryBuilder(new Pose2d())
                .lineToConstantHeading (new Vector2d(0,-3.2),
                        SampleMecanumDrive.getVelocityConstraint(13,2.5,10.69),
                        SampleMecanumDrive.getAccelerationConstraint(30)
                )
                .build();
        Trajectory traj1r = drive.trajectoryBuilder(new Pose2d())

                .lineToSplineHeading(new Pose2d(0,-25.5,Math.toRadians(-90)))

                .build();
        Trajectory trajddd = drive.trajectoryBuilder(new Pose2d())

                .lineToConstantHeading (new Vector2d(4,0))

                .build();
        Trajectory traj1d = drive.trajectoryBuilder(new Pose2d())

                .lineToConstantHeading(new Vector2d(4,0))
                .build();

        Trajectory traj1e = drive.trajectoryBuilder(new Pose2d())

                .lineToConstantHeading (new Vector2d(0,-6))

                .build();
        Trajectory trajeeeee = drive.trajectoryBuilder(new Pose2d())

                .lineToConstantHeading (new Vector2d(-22,-7))

                .build();
        Trajectory traj1c = drive.trajectoryBuilder(new Pose2d())

                .lineToSplineHeading (new Pose2d(-1,0,Math.toRadians(92)),
                        SampleMecanumDrive.getVelocityConstraint(20,6.7464,10.69),
                        SampleMecanumDrive.getAccelerationConstraint(50)

                )
                .build();
        Trajectory traj2c = drive.trajectoryBuilder(new Pose2d())

                .lineToSplineHeading (new Pose2d(-1,0,Math.toRadians(86)))

                .build();

        Trajectory trajd = drive.trajectoryBuilder(new Pose2d())

                .lineToConstantHeading(new Vector2d(-80,0))


                .build();

        Trajectory traj1x = drive.trajectoryBuilder(new Pose2d())

                .lineToSplineHeading(new Pose2d(0,15,Math.toRadians(1.3)))
                .build();
        Trajectory traj2x = drive.trajectoryBuilder(new Pose2d())

                .lineToSplineHeading(new Pose2d(0,22,Math.toRadians(1.6)))
                .build();
        Trajectory traj3x = drive.trajectoryBuilder(new Pose2d())

                .lineToSplineHeading(new Pose2d(0,28,Math.toRadians(2)))
                .build();
        Trajectory traj2a = drive.trajectoryBuilder(new Pose2d())

                .lineToConstantHeading (new Vector2d(-31,-3))

                .build();
        Trajectory traj2b = drive.trajectoryBuilder(new Pose2d())

                .lineToConstantHeading (new Vector2d(27.5,3))

                .build();
        Trajectory traj3a = drive.trajectoryBuilder(new Pose2d())

                .lineToConstantHeading(new Vector2d(-27,5.6))

                .build();
        Trajectory traj3b = drive.trajectoryBuilder(new Pose2d())

                .lineToConstantHeading (new Vector2d(23.5,-5.6))

                .build();
        Trajectory traj3e = drive.trajectoryBuilder(new Pose2d())

                .lineToSplineHeading (new Pose2d(11,0,Math.toRadians(1)))

                .build();

        Trajectory traj1f = drive.trajectoryBuilder(new Pose2d())

                .lineToConstantHeading (new Vector2d(0,30))

                .build();
        Trajectory traj2f = drive.trajectoryBuilder(new Pose2d())

                .lineToConstantHeading (new Vector2d(0,23))

                .build();
        Trajectory traj3f = drive.trajectoryBuilder(new Pose2d())

                .lineToConstantHeading (new Vector2d(0,19))

                .build();
        Trajectory traj1g = drive.trajectoryBuilder(new Pose2d())

                .lineToConstantHeading (new Vector2d(-10,0))

                .build();
        Trajectory traj3g = drive.trajectoryBuilder(new Pose2d())

                .lineToConstantHeading (new Vector2d(-15,0))

                .build();

        Trajectory traj2d = drive.trajectoryBuilder(new Pose2d())

                .lineToConstantHeading (new Vector2d(-10,-11))

                .build();

        Trajectory traj3c = drive.trajectoryBuilder(new Pose2d())

                .lineToSplineHeading (new Pose2d(0,10,Math.toRadians(-90)))

                .build();
        Trajectory traj3d = drive.trajectoryBuilder(new Pose2d())

                .lineToConstantHeading (new Vector2d(-8,-7))

                .build();

        Trajectory trajApril = drive.trajectoryBuilder(new Pose2d())
                .lineToConstantHeading(new Vector2d(1,0))

                .build();
        boolean d= true;
        boolean xxx = false;
        while (!opModeIsActive()) {
            telemetry.addData("DS preview on/off","3 dots, Camera Stream");
            telemetry.addLine();
            telemetry.addLine("----------------------------------------");
            telemetry.addData("auto",auto);
            telemetry.update();
        }



        waitForStart();

        if (opModeIsActive()) {
            xxx = true;

            if (auto == 2 || auto == 3) {
                telemetry.addData("auto", auto);
                telemetry.update();
            } else {
                resetRuntime();
                while (getRuntime() < 1 && auto == 1) {
                    telemetryTfod();

                    telemetry.update();
                }
            }
        }
        telemetry.addData("auto",auto);

        telemetry.update();
        resetRuntime();


        if (auto ==1){
            pixelOut.setPosition(0.8);
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
                                    .lineToConstantHeading(new Vector2d(xforlast-0.5, yforlast))

                                    .build();
                            hello = true;
                        }
                        if (hello) {
                            drive.followTrajectory(trajApril);
                        }
                    }




                }
            }

            if (!hello){
                sleep(300);
                drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
                drive.followTrajectory(traj1e);
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
                                        .lineToConstantHeading(new Vector2d(xforlast-0.5, yforlast))

                                        .build();
                                hello = true;
                            }
                            if (hello) {
                                drive.followTrajectory(trajApril);
                            }
                        }




                    }
                }
            }
            drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
            pixelOut.setPosition(0);
            intakeRotate.setPower(-0.1);
            sleep(1000);
            pixelIn.setPower(-1);
            sleep(200);
            hello = false;
            drive.followTrajectorySequence(trajSeq1back);
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
                                    .lineToConstantHeading(new Vector2d(xforlast-0.5, yforlast))

                                    .build();
                            hello = true;
                        }
                    }


                    if (hello) {
                        drive.followTrajectory(trajApril);
                    }

                }
            }
            frontLeftMotor.setPower(-0.2);
            frontRightMotor.setPower(-0.2);
            backLeftMotor.setPower(-0.2);
            backRightMotor.setPower(-0.2);
            sleep(200);
            pixelOut.setPosition(0);
            intakeRotate.setPower(-0.1);
            sleep(300);
            pixelIn.setPower(-1);
            drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
            sleep(300);
            drive.followTrajectory(trajddd);
            linearSlideLeft.setPower(-1);
            linearSlideRight.setPower(-1);
            sleep(300);
            linearSlideLeft.setPower(0);
            linearSlideRight.setPower(0);
            LinearSlidePosition.pos = linearSlideLeft.getCurrentPosition();

        }
        if (auto ==2) {
            pixelOut.setPosition(0.8);
            drive.followTrajectorySequence(trajSeq2);
            drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
            resetRuntime();
            while (getRuntime() < 0.3) {
                if (aprilTag.getDetections().size() > 0) {
                    for (int i = 0; i < aprilTag.getDetections().size(); i++) {
                        AprilTagDetection tag = aprilTag.getDetections().get(i);
                        idforapril = tag.id;
                        xforlast = -1 * tag.ftcPose.y;
                        yforlast = tag.ftcPose.x;

                        if (idforapril == 2) {
                            trajApril = drive.trajectoryBuilder(new Pose2d())
                                    .lineToConstantHeading(new Vector2d(xforlast -1, yforlast))

                                    .build();
                            hello = true;
                        }
                        if (hello) {
                            drive.followTrajectory(trajApril);
                        }
                    }


                }
            }
            if (!hello) {
                drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
                drive.followTrajectory(trajeeeee);
            }

            drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
            pixelOut.setPosition(0);
            intakeRotate.setPower(-0.1);
            sleep(300);
            pixelIn.setPower(-1);
            sleep(1000);
            hello = false;
            drive.followTrajectorySequence(trajSeq2back);
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
                                    .lineToConstantHeading(new Vector2d(xforlast-0.5, yforlast))

                                    .build();
                            hello = true;
                        }
                    }


                    if (hello) {
                        drive.followTrajectory(trajApril);
                    }

                }
            }
            frontLeftMotor.setPower(-0.2);
            frontRightMotor.setPower(-0.2);
            backLeftMotor.setPower(-0.2);
            backRightMotor.setPower(-0.2);
            sleep(500);
            pixelOut.setPosition(0);
            intakeRotate.setPower(-0.1);
            sleep(300);
            pixelIn.setPower(-1);
            drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
            sleep(800);
            drive.followTrajectory(trajddd);
            linearSlideLeft.setPower(-1);
            linearSlideRight.setPower(-1);
            sleep(1000);
            linearSlideLeft.setPower(0);
            linearSlideRight.setPower(0);
            LinearSlidePosition.pos = linearSlideLeft.getCurrentPosition();
            sleep(500);




        }
        if (auto ==3){
            pixelOut.setPosition(0.8);
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
                                    .lineToConstantHeading(new Vector2d(xforlast-1, yforlast))

                                    .build();
                            hello = true;
                        }
                        if (hello) {
                            drive.followTrajectory(trajApril);
                        }
                    }




                }
            }
            if (!hello) {
                drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
                drive.followTrajectory(trajeeeee);
            }

            drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
            pixelOut.setPosition(0);
            intakeRotate.setPower(-0.1);
            sleep(300);
            pixelIn.setPower(-1);
            sleep(1000);
            hello = false;
            drive.followTrajectorySequence(trajSeq3back);
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
                                    .lineToConstantHeading(new Vector2d(xforlast-0.5, yforlast))

                                    .build();
                            hello = true;
                        }
                    }


                    if (hello) {
                        drive.followTrajectory(trajApril);
                    }

                }
            }
            frontLeftMotor.setPower(-0.2);
            frontRightMotor.setPower(-0.2);
            backLeftMotor.setPower(-0.2);
            backRightMotor.setPower(-0.2);
            sleep(500);
            pixelOut.setPosition(0);
            intakeRotate.setPower(-0.1);
            sleep(300);
            pixelIn.setPower(-1);
            drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
            sleep(800);
            drive.followTrajectory(trajddd);
            linearSlideLeft.setPower(-1);
            linearSlideRight.setPower(-1);
            sleep(1000);
            linearSlideLeft.setPower(0);
            linearSlideRight.setPower(0);
            LinearSlidePosition.pos = linearSlideLeft.getCurrentPosition();
            sleep(500);


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
                if((recognition.getImageHeight()<=1.5* recognition.getImageWidth())&&(recognition.getImageWidth()<=1.5* recognition.getImageHeight())&&(recognition.getConfidence()>0.9)){
                    if (((recognition.getLeft()+ recognition.getRight())/2)<300){
                        auto=2;
                    } else {
                        auto = 3;
                    }

                } else {
                    idle();
                }

            telemetry.addData("auto",auto);
                telemetry.addData("confonf",recognition.getConfidence());
            }   // end for() loop
            telemetry.addData("auto",auto);
        }else {
            auto =1;
        }

        telemetry.addData("auto",auto);

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
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessors(tfod, aprilTag)
                    .build();
        }
    }   // end initDoubleVision()
}   // end class
