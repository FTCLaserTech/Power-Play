package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous(group = "a")
public class LeftHighJunction2 extends LinearOpMode
{

    @Override
    public void runOpMode() throws InterruptedException
    {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        //drive.IMUInit(hardwareMap);
        ExtraOpModeFunctions extras = new ExtraOpModeFunctions(hardwareMap, this);
        TrajectoryBook book = new TrajectoryBook(drive, extras);

        extras.clawClose();

        //Trajectory hub = null;
        //Trajectory hubBackup = null;
        Pose2d poseEstimate = drive.getPoseEstimate();

        telemetry.addLine("Initialized");
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.update();

        book.LeftHighJunction(drive.getPoseEstimate());
        book.LHJStacking(book.leftHighJunction.end());

        waitForStart();

        drive.followTrajectorySequence(book.leftHighJunction);
        drive.followTrajectorySequence(book.lHJStacking);
        drive.followTrajectorySequence(book.lHJStacking);
        drive.followTrajectorySequence(book.lHJStacking);
        drive.followTrajectorySequence(book.lHJStacking);
        drive.followTrajectorySequence(book.lHJStacking);

        sleep(10000);
    }
}