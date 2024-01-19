package org.firstinspires.ftc.teamcode.autonomous;


import android.graphics.drawable.GradientDrawable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@Autonomous
public class testRR extends LinearOpMode {
    BNO055IMU imu;
    Orientation angles;
    @Override
    public void runOpMode() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        imu =hardwareMap.get(BNO055IMU.class,"imu");
        imu.initialize(parameters);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .splineToLinearHeading(new Pose2d(24,24, Math.toRadians(90)), Math.toRadians(0))


                .build();

        waitForStart();

        if(isStopRequested()) return;

        while (opModeIsActive()){
            angles =imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("heading",angles.firstAngle);
            telemetry.addData("hing",angles.secondAngle);
            telemetry.addData("ading",angles.thirdAngle);
            telemetry.update();
        }
    }
}