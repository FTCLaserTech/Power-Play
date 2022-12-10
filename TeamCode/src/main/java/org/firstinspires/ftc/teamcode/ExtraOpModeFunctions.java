package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import com.vuforia.Vuforia;
import com.vuforia.PIXEL_FORMAT;

@Config
public class ExtraOpModeFunctions
{
    public enum RobotStartPosition {STRAIGHT, LEFT, RIGHT};
    public enum MarkerPosition {LEFT, MIDDLE, RIGHT}
    public enum FieldSide {RED, BLUE}

    public enum WristPosition {LEFT, MIDDLE, RIGHT}
    public WristPosition wristPosition = WristPosition.MIDDLE;
    public enum ElevatorPosition {COLLECT, GROUND, LOW, MIDDLE, HIGH}
    public ElevatorPosition elevatorPosition = ElevatorPosition.COLLECT;

    public static final double PI = 3.14159265;
    public int target = 0;

    private VuforiaLocalizer vuforia = null;
    //WebcamName webcamName = null;

    public Servo claw;
    public Servo wrist;
    public LinearOpMode localLop = null;

    public DcMotor elevator1;
    public DcMotor elevator2;

    public TouchSensor elevatorLimit;
    public RevColorSensorV3 colorSensor;
    public ColorRangeSensor testColorSensor;

    public RevBlinkinLedDriver blinkinLedDriver;
    public RevBlinkinLedDriver.BlinkinPattern pattern;

    public ExtraOpModeFunctions(HardwareMap hardwareMap, LinearOpMode lop)
    {
        claw = hardwareMap.get(Servo.class, "claw");
        wrist = hardwareMap.get(Servo.class, "wrist");
        elevator1 = hardwareMap.get(DcMotor.class, "elevator1");
        elevator2 = hardwareMap.get(DcMotor.class, "elevator2");

        //webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        localLop = lop;

        elevator1.setDirection(DcMotor.Direction.FORWARD);
        elevator1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevator1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator1.setTargetPosition(0);
        elevator1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        elevator2.setDirection(DcMotor.Direction.REVERSE);
        elevator2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevator2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator2.setTargetPosition(0);
        elevator2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        elevatorLimit = hardwareMap.get(TouchSensor.class, "elevatorLimit");

        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        pattern = RevBlinkinLedDriver.BlinkinPattern.CP1_2_BEATS_PER_MINUTE;
        blinkinLedDriver.setPattern(pattern);
        displayPattern();


        VuforiaLocalizer.Parameters parameters;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id",hardwareMap.appContext.getPackageName());
        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        //parameters.cameraName = webcamName;
        parameters.useExtendedTracking = false;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        vuforia.setFrameQueueCapacity(3);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565,true);
    }


    public void initElevator()
    {
        elevator1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevator2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        elevator1.setPower(-0.2);
        elevator2.setPower(-0.2);
        localLop.sleep(600);
        elevator1.setPower(0);
        elevator2.setPower(0);

        localLop.telemetry.addData("Limit ", elevatorLimit.getValue());
        localLop.telemetry.update();

        localLop.sleep(100);

        elevator1.setPower(0.1);
        elevator2.setPower(0.1);

        while(!elevatorLimit.isPressed())
        {
            ;
        }

        elevator1.setPower(0);
        elevator2.setPower(0);

        elevator1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        localLop.sleep(250);

        elevator1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        elevator1.setTargetPosition(-20);
        elevator2.setTargetPosition(-20);

        elevator1.setPower(1.0);
        elevator2.setPower(1.0);

        localLop.telemetry.addLine("Elevator Initialized!");
        localLop.telemetry.update();
    }


    public void clawOpen()
    {
        claw.setPosition(0.42);
    }

    public void clawClose()
    {
        claw.setPosition(0.485);
    }

    public void clawMove (int distance)
    {
        claw.setPosition(claw.getPosition() + distance);
    }

    public void wristLeft()
    {
        wrist.setPosition(1.0);
        wristPosition = wristPosition.LEFT;
    }

    public void wristMiddle()
    {
        wrist.setPosition(0.5);
        wristPosition = wristPosition.MIDDLE;
    }

    public void wristRight()
    {
        wrist.setPosition(0.0);
        wristPosition = wristPosition.RIGHT;
    }

    public void elevatorGround()
    {
        target = -10;
        elevatorPosition = elevatorPosition.COLLECT;

        elevator1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator1.setTargetPosition(target);
        elevator2.setTargetPosition(target);

        elevator1.setPower(1.0);
        elevator2.setPower(1.0);
    }

    public void elevatorJunction()
    {
        target = -20;
        elevatorPosition = elevatorPosition.GROUND;

        elevator1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator1.setTargetPosition(target);
        elevator2.setTargetPosition(target);

        elevator1.setPower(1.0);
        elevator2.setPower(1.0);
    }

    public void elevatorLow()
    {
        target = -1220;
        elevatorPosition = elevatorPosition.LOW;

        elevator1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator1.setTargetPosition(target);
        elevator2.setTargetPosition(target);

        elevator1.setPower(1.0);
        elevator2.setPower(1.0);
    }

    public void elevatorMiddle()
    {
        target = -2060;
        elevatorPosition = elevatorPosition.MIDDLE;

        elevator1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator1.setTargetPosition(target);
        elevator2.setTargetPosition(target);

        elevator1.setPower(1.0);
        elevator2.setPower(1.0);
    }

    public void elevatorHigh()
    {
        target = -2870;
        elevatorPosition = elevatorPosition.HIGH;

        elevator1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator1.setTargetPosition(target);
        elevator2.setTargetPosition(target);

        elevator1.setPower(1.0);
        elevator2.setPower(1.0);
    }

    protected void displayPattern()
    {
        blinkinLedDriver.setPattern(pattern);
    }

    private static final String VUFORIA_KEY =
            "AYkCgy7/////AAABmcVEWPZVAkr+qqRZ5GKKMtplRC79gsSR0agZEVe/znTU27Ffh0FtXPIGLOSGcu+OdpREriws8ksSpiZCvHpGc8cMP5JhNkjYOk71bfFphPQeGzxAqQr+0w4bsMkf4XHP1cXHVbaVP89ifVwqpnOLSm6Z7poTfguO8PMlHnoJIL6KEdnddmgKmQclRMFlerlVjcT55VFL4YAOetN7tbBZHcC4o/zGFgXdTfQWGNug7wHPvStMAArpFZUbSMEmHMdckbXgCCGCGVZw3qYQV9D3ALkAlwvPGQo+RXckMJ3kgk6trHnzxojWVfxsuflrcyDzorAmx+qn4Ei6R+HqxkrM7mSAgV45vyVlwN5GlyF7yv8g";


    public double adjustAngleForDriverPosition(double angle, RobotStartPosition robotStartPosition)
    {
        switch (robotStartPosition)
        {
            case STRAIGHT:
                break;
            case LEFT:
                angle = angle - PI/2;
                if(angle < (-PI))
                    angle = angle + (PI*2);
                break;
            case RIGHT:
                angle = angle + PI/2;
                if(angle > (PI))
                    angle = angle - (PI*2);
                break;
        }
        return angle;
    }

    /*
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;

    public MarkerPosition grabAndProcessImage(FieldSide fieldSide)
    {
        MarkerPosition markerPosition = MarkerPosition.RIGHT;
        Image imageRGB565 = null;

        int numGreen = 0;
        int numRed = 0;
        int numBlue = 0;

        CameraDevice.getInstance().start();

        try
        {
            Frame frame = vuforia.getFrameQueue().take();

            for (int i = 0; i < frame.getNumImages(); ++i)
            {
                Image image = frame.getImage(i);
                if (image.getFormat() == PIXEL_FORMAT.RGB565)
                {
                    imageRGB565 = image;

                    break;
                }
            }

            if (imageRGB565 != null)
            {
                // grab the image
                Bitmap bm = Bitmap.createBitmap(imageRGB565.getWidth(), imageRGB565.getHeight(), Bitmap.Config.RGB_565);
                bm.copyPixelsFromBuffer(imageRGB565.getPixels());
                if (fieldSide == FieldSide.RED)
                {
                    // create some variables to index the pixels
                    int xMidMin = 0;
                    int xMidMax = 0;
                    int yMidMin = 0;
                    int yMidMax = 0;

                    xMidMin = (int) (((2.85) * 480) / 4);
                    xMidMax = (int) (((3.4) * 480) / 4);
                    yMidMin = (int) (((2) * 640) / 5.5);
                    yMidMax = (int) (((2.7) * 640) / 5.5);

                    int pixel = 0;

                    for (int y = yMidMin; y <= yMidMax; y++)
                    {
                        for (int x = xMidMin; x <= xMidMax; x++)
                        {
                            // yellow in RGB is 0xFFFF00
                            pixel = bm.getPixel(y, x);

                            //Green
                            if ((pixel & 0x000000ff) < 0x00000000)
                            {
                                if ((pixel & 0x00ff0000) < 0x00000000)
                                {
                                    if ((pixel & 0x0000ff00) > 0x0000ff00)
                                    {
                                        numGreen++;
                                    }
                                }
                            }
                            //RED
                            if ((pixel & 0x000000ff) > 0x00000000)
                            {
                                if ((pixel & 0x00ff0000) > 0x00ff0000)
                                {
                                    if ((pixel & 0x0000ff00) > 0x00000000)
                                    {
                                        numRed++;
                                    }
                                }
                            }
                            //BLUE
                            if ((pixel & 0x000000ff) < 0x000000ff)
                            {
                                if ((pixel & 0x00ff0000) < 0x00000000)
                                {
                                    if ((pixel & 0x0000ff00) > 0x00000000)
                                    {
                                        numBlue++;
                                    }
                                }
                            }
                        }
                    }

                    if (numGreen >= 50)
                    {
                        localLop.telemetry.addData("Green", numGreen);
                        markerPosition = MarkerPosition.MIDDLE;

                    if (numRed >= 50)
                    {
                        localLop.telemetry.addData("Red", numRed);
                        markerPosition = MarkerPosition.MIDDLE;
                    }

                    if (numBlue >= 50)
                    {
                        localLop.telemetry.addData("Blue", numBlue);
                        markerPosition = MarkerPosition.MIDDLE;
                    }

                    }
                    localLop.telemetry.update();
                }
            }

            localLop.telemetry.addData("Blue_", numBlue);
            localLop.telemetry.addData("Green_", numGreen);
            localLop.telemetry.addData("Red_", numRed);
            localLop.telemetry.update();


        }
        catch(InterruptedException exc)
        {
            exc.printStackTrace();
        }

        return markerPosition;
    }
    */
}

