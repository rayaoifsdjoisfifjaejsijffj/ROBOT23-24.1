package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous
public class RED  extends LinearOpMode {

    int tag=0;
    int sleepTimer =3000;
    int scanTimer=3000;
    private OpenCvCamera webcam1;
    private OpenCvCamera webcam2;

    private static final int CAMERA_WIDTH  = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution

    private double CrLowerUpdate = 160;
    private double CbLowerUpdate = 100;
    private double CrUpperUpdate = 255;
    private double CbUpperUpdate = 255;

    public static double borderLeftX    = 0.0;   //fraction of pixels from the left side of the cam to skip
    public static double borderRightX   = 0.0;   //fraction of pixels from the right of the cam to skip
    public static double borderTopY     = 0.0;   //fraction of pixels from the top of the cam to skip
    public static double borderBottomY  = 0.0;   //fraction of pixels from the bottom of the cam to skip

    private double lowerruntime = 0;
    private double upperruntime = 0;

    // Pink Range                                      Y      Cr     Cb
    public static Scalar scalarLowerYCrCb = new Scalar(  0.0,160    , 100.0);
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 255.0, 255.0);

    // Yellow Range
//    public static Scalar scalarLowerYCrCb = new Scalar(0.0, 100.0, 0.0);
//    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 170.0, 120.0);
    ContourPipeline myPipeline1;
    ContourPipeline myPipeline2;


    public DcMotor LMotor;
    public DcMotor RMotor;
    public DcMotor Arm;
    double armSpeed = 0.5;
    public Servo trapdoor;

    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    float globalAngle;

    private ElapsedTime runtime = new ElapsedTime();

    //will use 20:1 HD Hex motor (revrobotics) + 90 mm grip wheels
    static final float     COUNTS_PER_MOTOR_REV    = 300f;
    static final float     DRIVE_GEAR_REDUCTION    = 1.0f;
    static final float     WHEEL_DIAMETER_INCHES   = 3.54f;
    static final float     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415f);
    static final float     DRIVE_SPEED             = 0.6f; //can adjust
    static final float     TURN_SPEED              = 0.2f; //can adjust


    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */

    /**
     * The variable to store our instance of the vision portal.
     */

    OpticSysAprilTag aprilTag1;
    @Override
    public void runOpMode()
    {
        // Create the AprilTag processor the easy way.
        // Create the vision portal the easy way.
        //Initialize the IMU and its parameters.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        LMotor = hardwareMap.dcMotor.get ("L_Motor"); //check with driver hub
        RMotor = hardwareMap.dcMotor.get("R_Motor"); //check with driver hub
        Arm = hardwareMap.dcMotor.get("Arm_Motor");
        trapdoor = hardwareMap.servo.get("Trapdoor");


        LMotor.setDirection(DcMotor.Direction.REVERSE); //to be tested with chassis

        LMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //The IMU does not initialize instantly. This makes it so the driver can see when they can push Play without errors.
        telemetry.addData("Mode", "calibrating...");


        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        LMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("Path0",  "Starting at %7d :%7d",
                LMotor.getCurrentPosition(),
                RMotor.getCurrentPosition());
        telemetry.update();

        //Tells the driver it is ok to start.
        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        //variable for how fast the robot will move
        float DRIVE_SPEED = 0.5f;





        ////////////////////////////////////////////
       // color=0;
        myPipeline1 = new ContourPipeline(borderLeftX,borderRightX,borderTopY,borderBottomY);
        myPipeline2 = new ContourPipeline(borderLeftX,borderRightX,borderTopY,borderBottomY);
        // OpenCV webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
                .splitLayoutForMultipleViewports(cameraMonitorViewId, //The container we're splitting
                        2, //The number of sub-containers to create
                        OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY);
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), viewportContainerIds[0]);
        webcam2 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), viewportContainerIds[1]);
        //OpenCV Pipeline

        // Configuration of Pipeline
        myPipeline1.configureScalarLower(scalarLowerYCrCb.val[0],scalarLowerYCrCb.val[1],scalarLowerYCrCb.val[2]);
        myPipeline2.configureScalarLower(scalarLowerYCrCb.val[0],scalarLowerYCrCb.val[1],scalarLowerYCrCb.val[2]);
        myPipeline1.configureScalarUpper(scalarUpperYCrCb.val[0],scalarUpperYCrCb.val[1],scalarUpperYCrCb.val[2]);        myPipeline1.configureScalarLower(scalarLowerYCrCb.val[0],scalarLowerYCrCb.val[1],scalarLowerYCrCb.val[2]);
        myPipeline2.configureScalarUpper(scalarUpperYCrCb.val[0],scalarUpperYCrCb.val[1],scalarUpperYCrCb.val[2]);
        // Webcam Streaming

        OpenCvCamera.AsyncCameraOpenListener listen1= new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam1.setPipeline( myPipeline1);
                webcam1.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        };

        OpenCvCamera.AsyncCameraOpenListener listen2=  new OpenCvCamera.AsyncCameraOpenListener()
    {
        @Override
        public void onOpened()
        {
            webcam2.setPipeline( myPipeline2);
            webcam2.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        }

        @Override
        public void onError(int errorCode)
        {
            /*
             * This will be called if the camera could not be opened
             */
        }
    };


        OpenCvCamera.AsyncCameraCloseListener listenClose1 = new OpenCvCamera.AsyncCameraCloseListener() {
            @Override
            public void onClose() {
               // webcam1.stopStreaming();
                webcam1.stopRecordingPipeline();
                webcam1.closeCameraDevice();
            }
        };
        OpenCvCamera.AsyncCameraCloseListener listenClose2 = new OpenCvCamera.AsyncCameraCloseListener() {
            @Override
            public void onClose() {
               // webcam2.stopStreaming();
                webcam2.stopRecordingPipeline();
                webcam2.closeCameraDevice();
        }
    };
        webcam1.openCameraDeviceAsync(listen1);
        webcam2.openCameraDeviceAsync(listen2);
        waitForStart();
        if(opModeIsActive())
        {
            encoderDrive(DRIVE_SPEED,  -24,  -24, 5);// Drive up to view front spike mark

            sleep(scanTimer);
            double go = run();
            telemetry.addData("location pending...", run());
            telemetry.update();
           /// sleep(3000);
            go= run();
            telemetry.addData("location", go);
            telemetry.update();

/*Pixel is straight ahead:
    0. Set tag to 2, tag will later be set to 5 if it is a red OpMode
    1. Push Pixel up to Spike Mark
    2. Move back
    3. Face Backboard
 */
            if(go==2)
            {
                tag = 2;
                telemetry.addData("straight ahead",go);
                telemetry.update();
                encoderDrive(1,-24,-24,5);
                encoderDrive(1,24,24,5);
                rotate(90,DRIVE_SPEED);

            }
/* Pixel is not straight ahead:
    1. Drive up to spike marks
    2. Check if Pixel is Left
*/
            else
            {
                telemetry.addData("NOT ", "Straight");
                telemetry.update();
                encoderDrive(1,-24,-24,5);
                sleep(sleepTimer);
                telemetry.addData("location pending", run());
                telemetry.update();
                go= run();
                telemetry.addData("location",go);
                telemetry.update();
                sleep(sleepTimer);
/* Pixel is Left
    0. Set Tag to 1
    1. Rotate to face spike mark
    2. Push pixel to spike mark
    3. Move Back
    4. Face Backboard
 */
                if(go==1)
                {
                    tag = 1;
                    telemetry.addData("drive", " left");
                    telemetry.update();
                    rotate(45,DRIVE_SPEED);
                    encoderDrive(1,-20,-20,5);
                    encoderDrive(1,48,48,5);
                    rotate(-135,DRIVE_SPEED);


                }
/* By process of elimination, the team prop is right
    0. Set tag to 3
    1. Rotate to face spike mark
    2. Push pixel
    3. Move back
    4. Face Backboard

 */
                else
                {
                    tag = 3;
                    telemetry.addData("drive", " right");
                    telemetry.update();
                    rotate(-45,DRIVE_SPEED);
                    encoderDrive(1,-20,-20,5);
                    encoderDrive(1,48,48,5);
                    rotate(-45,DRIVE_SPEED);
                }

            }
            telemetry.addData("driving", " to face backboard");
            telemetry.update();
            encoderDrive(1,-36,-36,5); // Drive up to backboard

            webcam1.closeCameraDeviceAsync(listenClose1); // Empty camera
            webcam2.closeCameraDeviceAsync(listenClose2);
//
            aprilTag1 = new OpticSysAprilTag(hardwareMap); // Initialize Apriltag
            aprilTag1.initAprilTag(); // Set up april tag with camera
            telemetry.update();
            telemetry.addData("april tag i hope ", aprilTag1.getTag());
            telemetry.update();
           // goToTag(aprilTag1.getTag());
            //placePixel();
        }
    }


    public double run()
    {
        myPipeline1.configureBorders(borderLeftX, borderRightX, borderTopY, borderBottomY);
        myPipeline2.configureBorders(borderLeftX, borderRightX, borderTopY, borderBottomY);
        if(myPipeline1.error){
            return -1;
        }
        // Only use this line of the code when you want to find the lower and upper values
        //testing(myPipeline1);
        if(myPipeline1.getRectHeight() > 50 || myPipeline2.getRectHeight() > 50 ){

            if(myPipeline1.getRectArea()>myPipeline2.getRectArea()){
                return 1;
            }
            if(myPipeline2.getRectArea()>myPipeline1.getRectArea())
            {
                return 2;

            }
        }
        return 3;

        //return myPipeline1.getRectHeight();
    }

    public double getA1()
    {
        return myPipeline1.getRectArea();
    }
    public double getA2()
    {
        return myPipeline2.getRectArea();
    }
    public void testing(ContourPipeline myPipeline){
        CrLowerUpdate = inValues(CrLowerUpdate, 0, 255);
        CrUpperUpdate = inValues(CrUpperUpdate, 0, 255);
        CbLowerUpdate = inValues(CbLowerUpdate, 0, 255);
        CbUpperUpdate = inValues(CbUpperUpdate, 0, 255);

        myPipeline.configureScalarLower(0.0, CrLowerUpdate, CbLowerUpdate);
        myPipeline.configureScalarUpper(255.0, CrUpperUpdate, CbUpperUpdate);

    }
    public Double inValues(double value, double min, double max){
        if(value < min){ value = min; }
        if(value > max){ value = max; }
        return value;
    }


    public void encoderDrive(float speed, float leftInches, float rightInches, float timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = LMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = RMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            LMotor.setTargetPosition(newLeftTarget);
            RMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            LMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            LMotor.setPower(Math.abs(speed));
            RMotor.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (LMotor.isBusy() && RMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        LMotor.getCurrentPosition(),
                        RMotor.getCurrentPosition());
                telemetry.update();
            }



            LMotor.setPower(0);
            RMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            LMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }


    }

    //This method reads the IMU getting the angle. It automatically adjusts the angle so that it is between -180 and +180.
    public float getAngle()
    {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        float deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    //The method turns the robot by a specific angle, -180 to +180.
    public void rotate(int degrees, float power)
    {
        float  leftPower, rightPower;

        resetAngle();

        //if the degrees are less than 0, the robot will turn right
        if (degrees < 0)
        {
            leftPower = power;
            rightPower = -power;
        }
        else if (degrees > 0)//if greater than 0, turn left
        {
            leftPower = -power;
            rightPower = power;
        }
        else return;

        //sets power to motors with negative signs properly assigned to make the robot go in the correct direction
        LMotor.setPower(leftPower);
        RMotor.setPower(rightPower);

        //Repeatedly check the IMU until the getAngle() function returns the value specified.
        if (degrees < 0)
        {
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {}
        }
        else
            while (opModeIsActive() && getAngle() < degrees) {}


        //stop the motors after the angle has been found.

        LMotor.setPower(0);
        RMotor.setPower(0);

        //sleep for a bit to make sure the robot doesn't over shoot
        sleep(1000); //can adjust

        resetAngle();
    }


    //this method resets the angle so that the robot's heading is now 0
    public void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }
/* Go to correct tag

 */
    public void goToTag(int tag)
    {
       if( aprilTag1.getTag()== tag+3)
       {
           driveToTag();
       }
       else if(aprilTag1.getTag()< tag)
       {
           rotate(90, DRIVE_SPEED);
           encoderDrive(DRIVE_SPEED, 10, 10,5 );
           rotate(90, DRIVE_SPEED);

       }
       else if(aprilTag1.getTag()> tag)
       {
           rotate(-90, DRIVE_SPEED);
           encoderDrive(DRIVE_SPEED, 10, 10,5 );
           rotate(-90, DRIVE_SPEED);
           goToTag(tag);

       }
    }

/*
Drive up to tag
 */
    public void driveToTag()
    {
        double x = aprilTag1.getX();
        double y = aprilTag1.getY();
        double angle =  Math.tan(x/y);
        double length = Math.sqrt(Math.pow(x,2)+Math.pow(y,2)); // set distance to travel as the hypotenuse of a triangle with x and y as sides
        rotate((int) angle, DRIVE_SPEED);
        encoderDrive(DRIVE_SPEED, (float) length, (float) length,5 );
        rotate(-90+(int)angle, DRIVE_SPEED );

    }

    public void placePIXEL()
    {
        int target = 1500;
        Arm.setTargetPosition(1500);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LMotor.setPower(Math.abs(armSpeed));
        trapdoor.setPosition(1.0);
        sleep(2000);
        trapdoor.setPosition(0.0);

    }



}
