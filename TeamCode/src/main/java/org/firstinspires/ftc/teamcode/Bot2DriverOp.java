package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "Bot2DriverOp", group = "12417")

//hello

public class Bot2DriverOp extends LinearOpMode {

    private DcMotor FLMotor;
    private DcMotor FRMotor;
    private DcMotor BLMotor;
    private DcMotor BRMotor;
    private DcMotor armLeft;
    private DcMotor armRight;

//    private Servo jellyFish;

    private final float MAX_WHEEL_SPEED = 0.6f;

    private final float jellyFishPosition = 0.0f;

    private final float SLIDE_FREEZE = 0.07f; //freezes jellyArm, counteracting gravitational force --> net force = 0
    private final int jellyDown = 0;
    private final int pos_rest = 1500;

//variables
    private float armPwr = 0.0f;
    private float yleft = 0;
    private float yright = 0;
    private float strafe = 0;
    private float rotate = 0;
    private float cfl = 1.0f;
    private float cfr = 1.0f;
    private float cbl = 1.0f;
    private float cbr = 1.0f;
    private boolean jellyFishHold = false;


    @Override
    public void runOpMode()
    {
        waitForStart();

        FLMotor = hardwareMap.dcMotor.get("FL_Motor");
        FRMotor = hardwareMap.dcMotor.get("FR_Motor");
        BLMotor = hardwareMap.dcMotor.get("BL_Motor");
        BRMotor = hardwareMap.dcMotor.get("BR_Motor");
        armLeft = hardwareMap.dcMotor.get("armLeft");
        armRight = hardwareMap.dcMotor.get("armRight");

        FLMotor.setDirection(DcMotor.Direction.REVERSE);
        BLMotor.setDirection(DcMotor.Direction.REVERSE);
        FRMotor.setDirection(DcMotor.Direction.FORWARD);
        BRMotor.setDirection(DcMotor.Direction.FORWARD);

        DcMotor[] motors = {FLMotor, FRMotor, BLMotor, BRMotor};
        for (int i =0; i<4; i++)
        {
            motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        float FLPwr, FRPwr, BLPwr, BRPwr;
        FLPwr = 0f;
        FRPwr = 0f;
        BLPwr = 0f;
        BRPwr = 0f;

        while(opModeIsActive())
        {

//            telemetry.addData("Arm: ", armLeft.getCurrentPosition());
            yleft = gamepad1.left_stick_y;
            yright = gamepad1.right_stick_y;
            strafe = gamepad1.left_stick_x;
            rotate = gamepad1.right_stick_x;

            FLPwr = (cfl)*(yleft + yright - strafe - rotate)/4f; //cfl for possible coeffs to deal with weight distr
            FRPwr = (cfr)*(yleft + yright + strafe + rotate)/4f; //cfr for possible coeffs to deal with weight distr
            BLPwr = (cbl)*(yleft + yright + strafe - rotate)/4f; //cbl for possible coeffs to deal with weight distr
            BRPwr = (cbr)*(yleft + yright - strafe + rotate)/4f; //cbr for possible coeffs to deal with weight distr

            FLMotor.setPower(FLPwr);
            FRMotor.setPower(FRPwr);
            BLMotor.setPower(BLPwr);
            BRMotor.setPower(BRPwr);

//            telemetry.addData("FL", FLMotor.getCurrentPosition());
            telemetry.update();

// gamepad 1:
// currently set to control all (in-progress) functionality

            if (gamepad1.left_bumper) {
                //lower linear slide
                armDown();

            }
            if (gamepad1.right_bumper) {
                //raise linear slide
                armUp();

            }
            if (!(gamepad1.left_bumper || gamepad1.right_bumper)) {
                armLeft.setPower(SLIDE_FREEZE); //freezes arm
                armRight.setPower(SLIDE_FREEZE); //""
            }
//            if (gamepad1.left_trigger) {
//                //...
//
//            }
//            if (gamepad1.right_trigger) {
//                //...
            }
            if (gamepad1.a) {
                //...
            }
            if (gamepad1.b) {
                //...
            }
            if (gamepad1.x) {
                 //...

            }
            if (gamepad1.dpad_left) {
                //...

            }
            if (gamepad1.dpad_right) {
                //...

            }
            if (gamepad1.dpad_up) {
                //set driving direction forwards
                FLMotor.setDirection(DcMotor.Direction.FORWARD);
                BLMotor.setDirection(DcMotor.Direction.FORWARD);
                FRMotor.setDirection(DcMotor.Direction.REVERSE);
                BRMotor.setDirection(DcMotor.Direction.REVERSE);
            }
            if (gamepad1.dpad_down) {
                //set driving direction backwards
                FLMotor.setDirection(DcMotor.Direction.REVERSE);
                BLMotor.setDirection(DcMotor.Direction.REVERSE);
                FRMotor.setDirection(DcMotor.Direction.FORWARD);
                BRMotor.setDirection(DcMotor.Direction.FORWARD);
            }

// gamepad 2:
// currently not in use

            if (gamepad2.left_bumper) {
                //...
            }
            if (gamepad2.right_bumper) {
                //...
            }
//            if (gamepad2.left_trigger) {
//                //...
//            }
//            if (gamepad2.right_trigger) {
//                //...
//            }
            if (gamepad2.a) {
                //...
            }
            if (gamepad2.b) {
                //...
            }
            if (gamepad2.x) {
                //...
            }
            if (gamepad2.y) {
                //...
            }

            if (gamepad2.dpad_left) {
                //...
            }
            if (gamepad2.dpad_right) {
                //...
            }
            if (gamepad2.dpad_up) {
                //...
            }
            if (gamepad2.dpad_down) {
                //...
            }

//.........................................................


    }

//    public void toggleJellyFish()
//    {
//        //jellyFishHold = !jellyFishHold;
//        sleep(200);
//
//        if(jellyFish.getPosition() != 0f)
//        {
//            jellyFish.setPosition(0f);
//        }
//        else
//        {
//            jellyFish.setPosition(2f);
//        }
//    }
    public void armUp()
    {
        armLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armPwr = 0.7f;
        armLeft.setPower(armPwr);
        armRight.setPower(armPwr);

//        telemetry.addData("Arm:", armLeft.getCurrentPosition());


    }

    public void armDown()
    {
        armLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armPwr = -0.5f;
        armLeft.setPower(armPwr);
        armRight.setPower(armPwr);

//        telemetry.addData("Arm:", armLeft.getCurrentPosition());
    }
//    public void goTopos_0() {
//        //jellyArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        armLeft.setPower(-0.5f);
//        armLeft.setTargetPosition(jellyDown);
//        armLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        //sleep(500);
//
//        telemetry.addData("Step 3 jelly Arm:", armLeft.getCurrentPosition());
//        telemetry.addData("jelly Target:", armLeft.getTargetPosition());
//
//
//    }
//    public void expandJellyFish()
//    {
//        jellyFish.setPosition(2f);
//        //sleep(500);
//
//        telemetry.addData("jellyFish Position:", jellyFish.getPosition());
//
//    }
//    public void releaseJellyFish()
//    {
//        jellyFish.setPosition(0f);
//        //sleep(500);
//
//        telemetry.addData("jellyFish Position:", jellyFish.getPosition());
//
//    }

}

