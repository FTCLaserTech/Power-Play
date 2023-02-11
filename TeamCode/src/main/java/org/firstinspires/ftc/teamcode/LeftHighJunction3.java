package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous(group = "a")
//@Disabled

public class LeftHighJunction3 extends LinearOpMode
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
        sleep(500);
        extras.initElevator();

        Pose2d poseEstimate = drive.getPoseEstimate();

        telemetry.addLine("Initialized");
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.update();

        book.LeftHighJunction(drive.getPoseEstimate());
        book.LHJFirstCone(book.leftHighJunction.end());
        book.LHJSecondCone(book.lHJFirstCone.end());
        book.LHJParkOne(book.lHJSecondCone.end());
        book.LHJParkTwo(book.lHJSecondCone.end());
        book.LHJParkThree(book.lHJSecondCone.end());

        waitForStart();
        ExtraOpModeFunctions.Signal Signal = extras.grabAndProcessImage(ExtraOpModeFunctions.FieldSide.RED);
        telemetry.addData("Signal Location: ", Signal);
        telemetry.update();

        drive.followTrajectorySequence(book.leftHighJunction);
        drive.followTrajectorySequence(book.lHJFirstCone);
        drive.followTrajectorySequence(book.lHJSecondCone);

        switch(Signal)
        {
            case ONE:
                // move to LEFT column of parking tiles
                drive.followTrajectorySequence(book.lHJParkOne);
                break;

            case TWO:
                // move to MIDDLE column of parking tiles
                drive.followTrajectorySequence(book.lHJParkTwo);
                break;

            case THREE:
                // move to RIGHT colum of parking tiles
                drive.followTrajectorySequence(book.lHJParkThree);
                break;

        }
    }
}