package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(group = "a")
public class BasicTeleOp extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        ExtraOpModeFunctions extras = new ExtraOpModeFunctions(hardwareMap, this);

        int IMUReset = 0;
        double stickForward;
        double stickSideways;
        double stickForwardRotated;
        double stickSidewaysRotated;
        double adjustedAngle;
        double powerMultiplier;

        boolean gamepad2_dpad_left_pressed = false;
        boolean gamepad2_dpad_right_pressed = false;
        boolean gamepad2_dpad_up_pressed = false;
        boolean gamepad2_dpad_down_pressed = false;
        boolean gamepad2_a_pressed = false;
        boolean gamepad2_b_pressed = false;

        int wristPosition = 1;
        int elevatorPosition = 0;

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();


        while (!isStopRequested())
        {
            if (gamepad1.right_bumper == true)
            {
                powerMultiplier = 0.6;
            }
            else if (gamepad1.left_bumper == true)
            {
                powerMultiplier = 0.4;
            }
            else
            {
                powerMultiplier = 0.75;
            }

            adjustedAngle = extras.adjustAngleForDriverPosition(drive.getRawExternalHeading(), ExtraOpModeFunctions.RobotStartPosition.STRAIGHT);
            stickForward = -gamepad1.left_stick_y * powerMultiplier;
            stickSideways = -gamepad1.left_stick_x * powerMultiplier;
            stickSidewaysRotated = (stickSideways * Math.cos(adjustedAngle)) - (stickForward * Math.sin(adjustedAngle));
            stickForwardRotated = (stickSideways * Math.sin(adjustedAngle)) + (stickForward * Math.cos(adjustedAngle));
            drive.setWeightedDrivePower(new Pose2d(stickForwardRotated, stickSidewaysRotated, -gamepad1.right_stick_x));
            drive.update();

            // RESET IMU
            if ((gamepad1.back == true) && (gamepad1.b == true))
            {
                IMUReset = 1;
            }
            else if (IMUReset == 1)
            {
                IMUReset = 0;
                telemetry.addLine("IMU Resetting...");
                telemetry.update();
                drive.IMUInit(hardwareMap);
            }

            NormalizedRGBA colors = extras.colorSensor.getNormalizedColors();

            if (getRuntime() >= 90 && getRuntime() <= 91)
            {
                extras.pattern = RevBlinkinLedDriver.BlinkinPattern.STROBE_RED;
                extras.displayPattern();
            }
            else
            {
                if (colors.alpha > 0.35)
                {
                    extras.pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
                    extras.displayPattern();
                }
                else
                {
                    extras.pattern = RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE;
                    extras.displayPattern();
                }
            }


            if (gamepad2.a)
            {
                gamepad2_a_pressed = true;
            }
            else if (!gamepad2.a && gamepad2_a_pressed)
            {
                extras.clawOpen();
            }

            if (gamepad2.b)
            {
                gamepad2_b_pressed = true;
            }
            else if (!gamepad2.b && gamepad2_b_pressed)
            {
                extras.clawClose();
            }


            // MANUAL ELEVATOR CONTROL- gamepad 2
            if(!extras.elevatorLimit.isPressed())
            {
                extras.elevator1.setPower(gamepad2.left_stick_y);
                extras.elevator2.setPower(gamepad2.left_stick_y);
            }
            else
            {
                extras.elevator1.setPower(0);
                extras.elevator2.setPower(0);
            }


            // wrist left movements
            if(gamepad2.dpad_left)
            {
                gamepad2_dpad_left_pressed = true;
            }
            else if (!gamepad2.dpad_left && gamepad2_dpad_left_pressed)
            {
                gamepad2_dpad_left_pressed = false;
                switch(wristPosition)
                {
                    case 0:
                        ;
                        break;

                    case 1:
                        extras.wristLeft();
                        wristPosition = 0;
                        break;

                    case 2:
                        extras.wristMiddle();
                        wristPosition = 1;
                        break;
                }
            }


            // wrist right movements
            if(gamepad2.dpad_right)
            {
                gamepad2_dpad_right_pressed = true;
            }
            else if (!gamepad2.dpad_right && gamepad2_dpad_right_pressed)
            {
                gamepad2_dpad_right_pressed = false;
                switch(wristPosition)
                {
                    case 0:
                        extras.wristMiddle();
                        wristPosition = 1;
                        break;

                    case 1:
                        extras.wristRight();
                        wristPosition = 2;
                        break;

                    case 2:
                        ;
                        break;
                }
            }


            // elevator up movements
            if(gamepad2.dpad_up)
            {
                gamepad2_dpad_up_pressed = true;
            }
            else if (!gamepad2.dpad_up && gamepad2_dpad_up_pressed)
            {
                gamepad2_dpad_up_pressed = false;
                switch(elevatorPosition)
                {
                    case 0:
                        extras.elevatorJunction();
                        elevatorPosition = 1;
                        break;

                    case 1:
                        extras.elevatorLow();
                        elevatorPosition = 2;
                        break;

                    case 2:
                        extras.elevatorMid();
                        elevatorPosition = 3;
                        break;

                    case 3:
                        extras.elevatorHigh();
                        elevatorPosition = 4;
                        break;

                    case 4:
                        ; // no movement
                        break;
                }
            }


            // elevator down movements
            if(gamepad2.dpad_down)
            {
                gamepad2_dpad_down_pressed = true;
            }
            else if (!gamepad2.dpad_down && gamepad2_dpad_down_pressed)
            {
                gamepad2_dpad_down_pressed = false;
                switch(elevatorPosition)
                {
                    case 0:
                        // no movement- already at lowest position
                        break;

                    case 1:
                        extras.elevatorGround();
                        elevatorPosition = 0;
                        break;

                    case 2:
                        extras.elevatorJunction();
                        elevatorPosition = 1;
                        break;

                    case 3:
                        extras.elevatorLow();
                        elevatorPosition = 2;
                        break;

                    case 4:
                        extras.elevatorMid();
                        elevatorPosition = 3;
                        break;
                }
            }



            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());

            telemetry.addData("Elapsed Time: ", getRuntime());

            telemetry.addLine()
                    .addData("Red", "%.3f", colors.red)
                    .addData("Green", "%.3f", colors.green)
                    .addData("Blue", "%.3f", colors.blue)
                    .addData("Alpha", "%.3f", colors.alpha);

            telemetry.update();
        }
    }
}