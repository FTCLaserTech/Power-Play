package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous(group = "a")
public class BlueParkOnly extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        ExtraOpModeFunctions extras = new ExtraOpModeFunctions(hardwareMap, this);
        TrajectoryBook book = new TrajectoryBook(drive, extras);

        extras.clawClose();

        Pose2d poseEstimate = drive.getPoseEstimate();

        telemetry.addLine("Initialized");
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.update();

        waitForStart();

        switch(extras.grabAndProcessImage(ExtraOpModeFunctions.FieldSide.BLUE))
        {
            case RED:
                // move to LEFT column of parking tiles
                drive.followTrajectorySequence(book.leftParkFromStart);
                break;

            case GREEN:
                // move to MIDDLE column of parking tiles
                drive.followTrajectorySequence(book.middleParkFromStart);
                break;

            case BLUE:
                // move to RIGHT colum of parking tiles
                drive.followTrajectorySequence(book.rightParkFromStart);
                break;
        }

        sleep(20000);
    }
}