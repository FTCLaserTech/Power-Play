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
        double speedMultiplier;

        boolean gp2_dpad_left_pressed = false;
        boolean gp2_dpad_right_pressed = false;
        boolean gp2_dpad_up_pressed = false;
        boolean gp2_dpad_down_pressed = false;
        boolean gp2_a_pressed = false;
        boolean gp2_b_pressed = false;

        double elevMultMin = 0.5;
        double elevMult = 0;
        double elevHeightMax = 3275;
        double slope;
        double elevatorEncoderCounts;

        NormalizedRGBA colors = extras.colorSensor.getNormalizedColors();
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();

        while (!isStopRequested())
        {

            slope = -elevMultMin / elevHeightMax;
            elevatorEncoderCounts = (extras.elevator1.getCurrentPosition() + extras.elevator2.getCurrentPosition()) / 2;
            elevMult = slope * elevatorEncoderCounts + 1;

            if (gamepad1.right_bumper)
            {
                speedMultiplier = 0.6;
            }
            else if (gamepad1.left_bumper)
            {
                speedMultiplier = 0.4;
            }
            else
            {
                speedMultiplier = 0.75;
            }

            adjustedAngle = extras.adjustAngleForDriverPosition(drive.getRawExternalHeading(), ExtraOpModeFunctions.RobotStartPosition.STRAIGHT);
            stickForward = -gamepad1.left_stick_y * speedMultiplier;
            stickSideways = -gamepad1.left_stick_x * speedMultiplier;
            stickSidewaysRotated = (stickSideways * Math.cos(adjustedAngle)) - (stickForward * Math.sin(adjustedAngle));
            stickForwardRotated = (stickSideways * Math.sin(adjustedAngle)) + (stickForward * Math.cos(adjustedAngle));
            drive.setWeightedDrivePower(new Pose2d(stickForwardRotated, stickSidewaysRotated, -gamepad1.right_stick_x));
            drive.update();

            // MANUAL ELEVATOR CONTROL- gamepad 2
            if(!extras.elevatorLimit.isPressed())
            {
                extras.elevator1.setPower(elevMult * gamepad2.left_stick_y);
                extras.elevator2.setPower(elevMult * gamepad2.left_stick_y);
            }
            else
            {
                extras.elevator1.setPower(0);
                extras.elevator2.setPower(0);
            }

            // RESET IMU
            if ((gamepad1.back) && (gamepad1.b))
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
                gp2_a_pressed = true;
            }
            else if (!gamepad2.a && gp2_a_pressed)
            {
                extras.clawOpen();
            }

            if (gamepad2.b)
            {
                gp2_b_pressed = true;
            }
            else if (!gamepad2.b && gp2_b_pressed)
            {
                extras.clawClose();
            }

            // wrist left movements
            if(gamepad2.dpad_left)
            {
                gp2_dpad_left_pressed = true;
            }
            else if (!gamepad2.dpad_left && gp2_dpad_left_pressed)
            {
                gp2_dpad_left_pressed = false;
                switch(extras.wristPosition)
                {
                    case LEFT:
                        break;

                    case MIDDLE:
                        extras.wristLeft();
                        break;

                    case RIGHT:
                        extras.wristMiddle();
                        break;
                }
            }

            // wrist right movements
            if(gamepad2.dpad_right)
            {
                gp2_dpad_right_pressed = true;
            }
            else if (!gamepad2.dpad_right && gp2_dpad_right_pressed)
            {
                gp2_dpad_right_pressed = false;
                switch(extras.wristPosition)
                {
                    case LEFT:
                        extras.wristMiddle();
                        break;

                    case MIDDLE:
                        extras.wristRight();
                        break;

                    case RIGHT:
                        break;
                }
            }

            // elevator up movements
            if(gamepad2.dpad_up)
            {
                gp2_dpad_up_pressed = true;
            }
            else if (!gamepad2.dpad_up && gp2_dpad_up_pressed)
            {
                gp2_dpad_up_pressed = false;
                switch(extras.elevatorPosition)
                {
                    case COLLECT:
                        extras.elevatorJunction();
                        break;

                    case GROUND:
                        extras.elevatorLow();
                        break;

                    case LOW:
                        extras.elevatorMiddle();
                        break;

                    case MIDDLE:
                        extras.elevatorHigh();
                        break;

                    case HIGH:
                        break;
                }
            }


            // elevator down movements
            if(gamepad2.dpad_down)
            {
                gp2_dpad_down_pressed = true;
            }
            else if (!gamepad2.dpad_down && gp2_dpad_down_pressed)
            {
                gp2_dpad_down_pressed = false;
                switch(extras.elevatorPosition)
                {
                    case COLLECT:
                        break;

                    case GROUND:
                        extras.elevatorGround();
                        break;

                    case LOW:
                        extras.elevatorJunction();
                        break;

                    case MIDDLE:
                        extras.elevatorLow();
                        break;

                    case HIGH:
                        extras.elevatorMiddle();
                        break;
                }
            }

            colors = extras.colorSensor.getNormalizedColors();

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