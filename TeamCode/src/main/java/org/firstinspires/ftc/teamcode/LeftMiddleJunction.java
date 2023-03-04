package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous(group = "a")
//@Disabled
public class LeftMiddleJunction extends LinearOpMode
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

        book.LeftMedJuncInitial(drive.getPoseEstimate());
        book.LMJStackToJunction(new Pose2d(0,0,Math.toRadians(90)));
        book.LMJJunctionToStack4(book.lMJStackToJunction.end());
        book.LMJJunctionToStack3(book.lMJStackToJunction.end());
        book.LMJJunctionToStack2(book.lMJStackToJunction.end());
        book.LMJJunctionToStack1(book.lMJStackToJunction.end());
        book.LMJLeftPark(book.lMJStackToJunction.end());
        book.LMJMiddlePark(book.lMJStackToJunction.end());
        book.LMJRightPark(book.lMJStackToJunction.end());

        telemetry.addLine("Initialized");
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.update();

        waitForStart();
        ExtraOpModeFunctions.Signal Signal = extras.grabAndProcessImage(ExtraOpModeFunctions.FieldSide.RED);
        telemetry.addData("Signal Location: ", Signal);
        telemetry.update();

        drive.followTrajectorySequence(book.leftMediumJunction);
        //At the wall for cone 5
        drive.setPoseEstimate(new Pose2d(0,0, Math.toRadians(90)));
        drive.followTrajectorySequence(book.lMJStackToJunction);
        drive.followTrajectorySequence(book.lMJJunctionToStack4);
        //At the wall for cone 4
        drive.setPoseEstimate(new Pose2d(0,0, Math.toRadians(90)));
        drive.followTrajectorySequence(book.lMJStackToJunction);
        drive.followTrajectorySequence(book.lMJJunctionToStack3);
        //At the wall for cone 3
        drive.setPoseEstimate(new Pose2d(0,0, Math.toRadians(90)));
        drive.followTrajectorySequence(book.lMJStackToJunction);
        drive.followTrajectorySequence(book.lMJJunctionToStack2);
        //At the wall for cone 2
        drive.setPoseEstimate(new Pose2d(0,0, Math.toRadians(90)));
        drive.followTrajectorySequence(book.lMJStackToJunction);
        drive.followTrajectorySequence(book.lMJJunctionToStack1);
        //At the wall for cone 1
        drive.setPoseEstimate(new Pose2d(0,0, Math.toRadians(90)));
        drive.followTrajectorySequence(book.lMJStackToJunction);
        //End of stacking 1+5
        switch(Signal)
        {
            case ONE:
                // move to LEFT column of parking tiles
                drive.followTrajectorySequence(book.lMJLeftPark);
                break;

            case TWO:
                // move to MIDDLE column of parking tiles
                drive.followTrajectorySequence(book.lMJMiddlePark);
                break;

            case THREE:
                // move to RIGHT colum of parking tiles
                drive.followTrajectorySequence(book.lMJRightPark);
                break;

        }

        sleep(10000);
    }
}