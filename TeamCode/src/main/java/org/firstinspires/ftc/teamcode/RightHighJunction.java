package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous(group = "a")
//@Disabled
public class RightHighJunction extends LinearOpMode
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

        book.RightHighJunction(drive.getPoseEstimate());
        book.RHJFirstCone(book.rightHighJunction.end());
        book.RHJSecondCone(book.rHJFirstCone.end());
        book.RHJParkOne(book.rHJSecondCone.end());
        book.RHJParkTwo(book.rHJSecondCone.end());
        book.RHJParkThree(book.rHJSecondCone.end());

        telemetry.addLine("Initialized");
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.update();

        waitForStart();
        ExtraOpModeFunctions.Signal Signal = extras.grabAndProcessImage(ExtraOpModeFunctions.FieldSide.RED);
        telemetry.addData("Signal Location: ", Signal);
        telemetry.update();

        drive.followTrajectorySequence(book.rightHighJunction);
        drive.followTrajectorySequence(book.rHJFirstCone);
        drive.followTrajectorySequence(book.rHJSecondCone);

        switch(Signal)
        {
            case ONE:
                // move to LEFT column of parking tiles
                drive.followTrajectorySequence(book.rHJParkOne);
                break;

            case TWO:
                // move to MIDDLE column of parking tiles
                drive.followTrajectorySequence(book.rHJParkTwo);
                break;

            case THREE:
                // move to RIGHT colum of parking tiles
                drive.followTrajectorySequence(book.rHJParkThree);
                break;

        }


        sleep(10000);
    }
}