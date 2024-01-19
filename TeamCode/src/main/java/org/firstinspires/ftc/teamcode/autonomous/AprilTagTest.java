package org.firstinspires.ftc.teamcode.autonomous;
import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@Autonomous

public class AprilTagTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();
        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                        .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640,480))
                                .build();
        //RR Code
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
        double x = 0;
        double y = 0;
        double z = 0;
        int id = 0;
            boolean hello = false;
            waitForStart();
            resetRuntime();
            Trajectory trajApril = drive.trajectoryBuilder(new Pose2d())
                    .lineToConstantHeading(new Vector2d(1,0))

                    .build();
            while (opModeIsActive()) {

                if (tagProcessor.getDetections().size() > 0) {
                    for (int i = 0; i<tagProcessor.getDetections().size();i++){
                        AprilTagDetection tag = tagProcessor.getDetections().get(i);
                        id = tag.id;
                     x = -1*tag.ftcPose.y;
                     y = tag.ftcPose.x;
                     z = tag.ftcPose.yaw;
                    telemetry.addData("x",x);
                    telemetry.addData("y",y);
                    telemetry.update();
                    if (id == 4){
                        trajApril = drive.trajectoryBuilder(new Pose2d())
                                .lineToSplineHeading(new Pose2d(x,y,z))

                                .build();
                        hello = true;
                    }
                }


                if (hello){
                    drive.followTrajectory(trajApril);
                }

            }


        }
    }
}

