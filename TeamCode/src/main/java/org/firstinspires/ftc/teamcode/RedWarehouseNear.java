package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

//@Disabled
@Config
@Autonomous(group = "b")
public class RedWarehouseNear extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.IMUInit(hardwareMap);
        ExtraOpModeFunctions extras = new ExtraOpModeFunctions(hardwareMap, this);
        TrajectoryBook book = new TrajectoryBook(drive, extras);

        extras.clawClose();
        sleep(1000);
        extras.initArm();
        extras.wristClose();
        telemetry.addLine("Initialized");
        telemetry.update();

        waitForStart();

        // scan for capstone
        ExtraOpModeFunctions.MarkerPosition markerPosition = extras.grabAndProcessImage(ExtraOpModeFunctions.FieldSide.RED);

        //grab capstone

        // immediately move to hub and place block
        switch (markerPosition)
        {
            case RIGHT:
                telemetry.addLine("Location 1");
                telemetry.update();
                book.RedWarehouseRight(drive.getPoseEstimate());
                telemetry.addLine("Location 2");
                telemetry.update();
                drive.followTrajectorySequence(book.redCapstoneWarehouse);
                telemetry.addLine("Location 3");
                telemetry.update();
                break;
            case MIDDLE:
                book.RedWarehouseMiddle(drive.getPoseEstimate());
                drive.followTrajectorySequence(book.redCapstoneWarehouse);
                break;
            case LEFT:
                book.RedWarehouseLeft(drive.getPoseEstimate());
                drive.followTrajectorySequence(book.redCapstoneWarehouse);
                break;
        }
    }
}