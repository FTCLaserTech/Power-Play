package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous(group = "a")
public class ParkOnly extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.IMUInit(hardwareMap);
        ExtraOpModeFunctions extras = new ExtraOpModeFunctions(hardwareMap, this);
        TrajectoryBook book = new TrajectoryBook(drive, extras);

        extras.clawClose();
        extras.wristMiddle();
        extras.initElevator();

        Pose2d poseEstimate = drive.getPoseEstimate();

        telemetry.addLine("Initialized");
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.update();

        book.LeftParkFromStart(drive.getPoseEstimate());
        book.MiddleParkFromStart(drive.getPoseEstimate());
        book.RightParkFromStart(drive.getPoseEstimate());

        waitForStart();

        ExtraOpModeFunctions.Signal coneColor = extras.grabAndProcessImage(ExtraOpModeFunctions.FieldSide.RED);
        telemetry.addData("Cone Color: ", coneColor);
        telemetry.update();

        switch(coneColor)
        {
            case ONE:
                // move to LEFT column of parking tiles
                drive.followTrajectorySequence(book.leftParkFromStart);
                break;

            case TWO:
                // move to MIDDLE column of parking tiles
                drive.followTrajectorySequence(book.middleParkFromStart);
                break;

            case THREE:
                // move to RIGHT colum of parking tiles
                drive.followTrajectorySequence(book.rightParkFromStart);
                break;

        }




        sleep(20000);
    }
}