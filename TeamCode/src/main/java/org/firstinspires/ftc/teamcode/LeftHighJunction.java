package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous(group = "a")
//@Disabled
public class LeftHighJunction extends LinearOpMode
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

        book.LeftHighJuncInitial(drive.getPoseEstimate());
        book.LHJStackToJunction(new Pose2d(0,0,Math.toRadians(90)));
        book.LHJJunctionToStack4(book.lHJStackToJunction.end());
        book.LHJJunctionToStack3(book.lHJStackToJunction.end());
        book.LHJJunctionToStack2(book.lHJStackToJunction.end());
        book.LHJJunctionToStack1(book.lHJStackToJunction.end());
        book.LHJLeftPark(book.lHJStackToJunction.end());
        book.LHJMiddlePark(book.lHJStackToJunction.end());
        book.LHJRightPark(book.lHJStackToJunction.end());

        telemetry.addLine("Initialized");
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.update();

        waitForStart();
        ExtraOpModeFunctions.Signal Signal = extras.grabAndProcessImage(ExtraOpModeFunctions.FieldSide.RED);
        telemetry.addData("Signal Location: ", Signal);
        telemetry.update();

        drive.followTrajectorySequence(book.leftHighJunction);
        /*
        //At the wall for cone 5
        drive.setPoseEstimate(new Pose2d(0,0, Math.toRadians(90)));
        drive.followTrajectorySequence(book.lHJStackToJunction);
        drive.followTrajectorySequence(book.lHJJunctionToStack4);
        //At the wall for cone 4
        drive.setPoseEstimate(new Pose2d(0,0, Math.toRadians(90)));
        drive.followTrajectorySequence(book.lHJStackToJunction);
        drive.followTrajectorySequence(book.lHJJunctionToStack3);
        //At the wall for cone 3
        drive.setPoseEstimate(new Pose2d(0,0, Math.toRadians(90)));
        drive.followTrajectorySequence(book.lHJStackToJunction);
        drive.followTrajectorySequence(book.lHJJunctionToStack2);
        //At the wall for cone 2
        drive.setPoseEstimate(new Pose2d(0,0, Math.toRadians(90)));
        drive.followTrajectorySequence(book.lHJStackToJunction);
        drive.followTrajectorySequence(book.lHJJunctionToStack1);
        //At the wall for cone 1
        drive.setPoseEstimate(new Pose2d(0,0, Math.toRadians(90)));
        drive.followTrajectorySequence(book.lHJStackToJunction);
        //End of stacking 1+5
        switch(Signal)
        {
            case ONE:
                // move to LEFT column of parking tiles
                drive.followTrajectorySequence(book.lHJLeftPark);
                break;

            case TWO:
                // move to MIDDLE column of parking tiles
                drive.followTrajectorySequence(book.lHJMiddlePark);
                break;

            case THREE:
                // move to RIGHT colum of parking tiles
                drive.followTrajectorySequence(book.lHJRightPark);
                break;

        }

         */

        sleep(10000);
    }
}