package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

//@Disabled
@Config
@Autonomous(group = "b")
public class BlueWarehouseFar extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        //drive.IMUInit(hardwareMap);
        ExtraOpModeFunctions extras = new ExtraOpModeFunctions(hardwareMap, this);
        TrajectoryBook book = new TrajectoryBook(drive, extras);

        extras.clawClose();
        sleep(500);
        extras.initArm();
        extras.wristClose();
        telemetry.addLine("Initialized");
        telemetry.update();

        waitForStart();

        // scan for capstone
        ExtraOpModeFunctions.MarkerPosition markerPosition = extras.grabAndProcessImage(ExtraOpModeFunctions.FieldSide.BLUE);

        switch (markerPosition)
        {
            case RIGHT:
                book.BlueWarehouseRight(drive.getPoseEstimate());
                drive.followTrajectorySequence(book.blueCapstoneWarehouse);
                book.BlueWarehouseRightShift(drive.getPoseEstimate());
                drive.followTrajectorySequence(book.blueCapstoneWarehouseShift);
                break;
            case MIDDLE:
                book.BlueWarehouseMiddle(drive.getPoseEstimate());
                drive.followTrajectorySequence(book.blueCapstoneWarehouse);
                book.BlueWarehouseMiddleShift(drive.getPoseEstimate());
                drive.followTrajectorySequence(book.blueCapstoneWarehouseShift);
                break;
            case LEFT:
                book.BlueWarehouseLeft(drive.getPoseEstimate());
                drive.followTrajectorySequence(book.blueCapstoneWarehouse);
                book.BlueWarehouseLeftShift(drive.getPoseEstimate());
                drive.followTrajectorySequence(book.blueCapstoneWarehouseShift);
                break;
        }
    }
}