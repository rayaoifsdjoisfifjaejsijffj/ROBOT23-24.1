package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class MoveSys {

    DcMotor LMotor;
    DcMotor RMotor;
    DcMotor Arm;

    Servo trapdoor;
    BNO055IMU imu;

    OpticSysOpenCV openCv;
    OpticSysAprilTag aprilTag;

    //numbers
    float armSpeed = 0.4f;
    static final float     COUNTS_PER_MOTOR_REV    = 300f;
    static final float     DRIVE_GEAR_REDUCTION    = 1.0f;
    static final float     WHEEL_DIAMETER_INCHES   = 3.54f;
    static final float     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415f);
    static final float     DRIVE_SPEED             = 0.7f;
    static final float     TURN_SPEED              = 0.9f;
    static final float     ARM_SPEED                = 0.4f;

    //placeholder
    Orientation lastAngles = new Orientation();
    float globalAngle;

    public MoveSys(HardwareMap hardwareMap) {
        //openCv = new OpticSysOpenCV(hardwareMap);
        aprilTag = new OpticSysAprilTag(hardwareMap);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        LMotor = hardwareMap.dcMotor.get("L_Motor");
        RMotor = hardwareMap.dcMotor.get("R_Motor");
        LMotor.setDirection(DcMotor.Direction.REVERSE);

    }

    public void driveStraight(double inches)
    {
        int newTargetLeft;
        int newTargetRight;

        // Ensure that the opmode is still active

            // Determine new target position, and pass to motor controller
            newTargetLeft = LMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newTargetRight = RMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            LMotor.setTargetPosition(newTargetLeft);
            RMotor.setTargetPosition(newTargetRight);

            // Turn On RUN_TO_POSITION
            LMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            LMotor.setPower(Math.abs(DRIVE_SPEED));
            RMotor.setPower(Math.abs(DRIVE_SPEED));


            LMotor.setPower(0);
            RMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            LMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
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
    public void rotate(int degrees)
    {       float  leftPower, rightPower;

        resetAngle();

        //if the degrees are less than 0, the robot will turn right
        if (degrees < 0)
        {
            leftPower = TURN_SPEED;
            rightPower = -TURN_SPEED;
        }
        else if (degrees > 0)//if greater than 0, turn left
        {
            leftPower = -TURN_SPEED;
            rightPower = TURN_SPEED;
        }
        else return;

        //sets power to motors with negative signs properly assigned to make the robot go in the correct direction
        LMotor.setPower(leftPower);
        RMotor.setPower(rightPower);

        //Repeatedly check the IMU until the getAngle() function returns the value specified.
        if (degrees < 0)
        {
            while (getAngle() == 0) {}

            while (getAngle() > degrees) {}
        }
        else
            while (getAngle() < degrees) {}


        //stop the motors after the angle has been found.

        LMotor.setPower(0);
        RMotor.setPower(0);

        //sleep for a bit to make sure the robot doesn't over shoot

        resetAngle();
    }
    public void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }
    //Go to correct tag

    public void placePixel()
    {
        int target = 1500;
        Arm.setTargetPosition(1500);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LMotor.setPower(Math.abs(armSpeed));
        trapdoor.setPosition(1.0);

    }
    public void returnTrapdoor()
    {
        trapdoor.setPosition(0.0);
    }
    public String getCalibrationStatus()
    {
        return imu.getCalibrationStatus().toString();
    }
}
