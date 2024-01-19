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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

/*
 * This OpMode illustrates the basics of TensorFlow Object Detection,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@Autonomous(name = "RedBack_2")
@Disabled

public class RedBack_2 extends LinearOpMode {


    int auto =3;
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "Meet1Red.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/myCustomModel.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "red",
    };

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        CRServo intakeMove = hardwareMap.crservo.get("intakeMove");
        CRServo intakeRotate = hardwareMap.crservo.get("intakeRotate");
        Servo pixelOut = hardwareMap.servo.get("pixelOut");
        CRServo pixelIn = hardwareMap.crservo.get("pixelIn");

        initTfod();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
        Trajectory traj1a = drive.trajectoryBuilder(new Pose2d())
                .lineToConstantHeading(new Vector2d(-27,-6.6))

                .build();
        Trajectory traj1b = drive.trajectoryBuilder(new Pose2d())
                .lineToSplineHeading (new Pose2d(-1.5,0,Math.toRadians(-90)))
                .build();
        Trajectory traj1c = drive.trajectoryBuilder(new Pose2d())

                .lineToConstantHeading(new Vector2d(-85,0))

                .build();
        Trajectory traj1d = drive.trajectoryBuilder(new Pose2d())

                .lineToConstantHeading(new Vector2d(23,6.6))
                .build();
        Trajectory traj1e = drive.trajectoryBuilder(new Pose2d())

                .lineToConstantHeading(new Vector2d(-9.7,-13))
                .build();
        Trajectory traj1j = drive.trajectoryBuilder(new Pose2d())
                .lineToConstantHeading(new Vector2d(0,3))
                .build();


        Trajectory traj2a = drive.trajectoryBuilder(new Pose2d())
                .lineToConstantHeading(new Vector2d(-31,0))
                .build();
        Trajectory traj2d = drive.trajectoryBuilder(new Pose2d())
                .lineToConstantHeading(new Vector2d(28,0))
                .build();
        Trajectory traj2c = drive.trajectoryBuilder(new Pose2d())

                .lineToConstantHeading(new Vector2d(100,0))

                .build();
        Trajectory traj2erd = drive.trajectoryBuilder(new Pose2d())

                .lineToConstantHeading (new Vector2d(-20,8))

                .build();
        Trajectory traj2e = drive.trajectoryBuilder(new Pose2d())

                .lineToConstantHeading (new Vector2d(-8.5,-24))

                .build();

        Trajectory traj3a = drive.trajectoryBuilder(new Pose2d())
                .lineToSplineHeading(new Pose2d(-27,7.3,Math.toRadians(-90)))

                .build();
        Trajectory traj3d = drive.trajectoryBuilder(new Pose2d())

                .lineToConstantHeading (new Vector2d(10,0))

                .build();
        Trajectory traj3c = drive.trajectoryBuilder(new Pose2d())

                .lineToConstantHeading (new Vector2d(0,23))

                .build();
        Trajectory traj3b = drive.trajectoryBuilder(new Pose2d())

                .lineToConstantHeading(new Vector2d(-89,0))
                .build();
        Trajectory traj3e = drive.trajectoryBuilder(new Pose2d())

                .lineToConstantHeading(new Vector2d(-8.5,-28.5))
                .build();
        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        while (!opModeIsActive()){
            telemetryTfod();
            telemetry.update();
        }
        boolean d= true;
        waitForStart();

        if (opModeIsActive()) {
            if (auto == 1 || auto == 2) {
                telemetry.addData("auto", auto);
                telemetry.update();
            } else {
                resetRuntime();
                while (getRuntime() < 3 && auto == 3) {
                    telemetryTfod();
                    telemetry.update();
                }
            }
        }
        telemetry.addData("auto",auto);
        telemetry.update();
        if (auto ==1){
            drive.followTrajectory(traj1a);
            sleep(1000);
            drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
            drive.followTrajectory(traj1d);
            drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
            sleep(1000);
            drive.followTrajectory(traj1b);
            sleep(1000);
            drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
            sleep(1000);
            drive.followTrajectory(traj1c);
            sleep(1000);
            drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
            drive.followTrajectory(traj1e);
            sleep(1000);
            intakeMove.setPower(-1);
            intakeRotate.setPower(1);
            sleep(4000);

            intakeMove.setPower(-1);
            intakeRotate.setPower(-0.6);
            sleep(2000);

            pixelOut.setPosition(0);
            pixelIn.setPower(-1);
            sleep(2000);

        }
        if (auto ==2){
            drive.followTrajectory(traj2a);
            sleep(1000);
            drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
            drive.followTrajectory(traj2d);
            drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
            sleep(1000);
            drive.followTrajectory(traj1b);
            sleep(1000);
            drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
            sleep(1000);
            drive.followTrajectory(traj1c);
            drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
            sleep(1000);
            drive.followTrajectory(traj2e);
            sleep(1000);

            intakeMove.setPower(-1);
            intakeRotate.setPower(1);
            sleep(4000);

            intakeMove.setPower(-1);
            intakeRotate.setPower(-0.6);
            sleep(2000);

            pixelOut.setPosition(0);
            pixelIn.setPower(-1);
            sleep(2000);



        } if (auto ==3){
            drive.followTrajectory(traj3a);
            sleep(1000);
            drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
            drive.followTrajectory(traj3d);
            drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
            sleep(1000);
            drive.followTrajectory(traj3c);
            sleep(1000);
            drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
            sleep(1000);
            drive.followTrajectory(traj3b);
            drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
            sleep(1000);
            drive.followTrajectory(traj3e);
            sleep(1000);

            intakeMove.setPower(-1);
            intakeRotate.setPower(1);
            sleep(4000);

            intakeMove.setPower(-1);
            intakeRotate.setPower(-0.6);
            sleep(2000);

            pixelOut.setPosition(0);
            pixelIn.setPower(-1);
            sleep(2000);

        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end runOpMode()

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

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
            }   // end for() loop
            telemetry.addData("auto",auto);
        }else {
            auto =3;
        }

        telemetry.addData("auto",auto);

    }   // end method telemetryTfod()

}   // end class
