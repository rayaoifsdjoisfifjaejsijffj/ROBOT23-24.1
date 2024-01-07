package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


public class OpticSysOpenCV {
    public HardwareMap hardwareMap;

    private OpenCvCamera webcam1;
    private OpenCvCamera webcam2;

    ContourPipeline myPipeline1;
    ContourPipeline myPipeline2;

    OpenCvCamera.AsyncCameraOpenListener listen1;
    OpenCvCamera.AsyncCameraOpenListener listen2;
    OpenCvCamera.AsyncCameraCloseListener listenClose1;
    OpenCvCamera.AsyncCameraCloseListener listenClose2;
    // values
    private static final int CAMERA_WIDTH = 1280; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 720; // height of wanted camera resolution


    public static Scalar scalarLowerYCrCb = new Scalar(0.0, 160, 100.0);
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 255.0, 255.0);

    public static double borderLeftX = 0.0;   //fraction of pixels from the left side of the cam to skip
    public static double borderRightX = 0.0;   //fraction of pixels from the right of the cam to skip
    public static double borderTopY = 0.0;   //fraction of pixels from the top of the cam to skip
    public static double borderBottomY = 0.0;   //fraction of pixels from the bottom of the cam to skip

    public OpticSysOpenCV(HardwareMap hardwareMap, int color) {
        this.hardwareMap = hardwareMap;
        if (color == 1)
        {
            scalarLowerYCrCb = new Scalar(0, 50, 50);
            scalarUpperYCrCb = new Scalar(255.0, 120, 255);
        }
    }

    public void initOpenCV() {
        myPipeline1 = new ContourPipeline(borderLeftX, borderRightX, borderTopY, borderBottomY);
        myPipeline2 = new ContourPipeline(borderLeftX, borderRightX, borderTopY, borderBottomY);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
                .splitLayoutForMultipleViewports(cameraMonitorViewId, //The container we're splitting
                        2, //The number of sub-containers to create
                        OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY);
        //OpenCV Pipeline

        // Configuration of Pipeline
        myPipeline1.configureScalarLower(scalarLowerYCrCb.val[0], scalarLowerYCrCb.val[1], scalarLowerYCrCb.val[2]);
        myPipeline2.configureScalarLower(scalarLowerYCrCb.val[0], scalarLowerYCrCb.val[1], scalarLowerYCrCb.val[2]);
        myPipeline1.configureScalarUpper(scalarUpperYCrCb.val[0], scalarUpperYCrCb.val[1], scalarUpperYCrCb.val[2]);
        myPipeline2.configureScalarUpper(scalarUpperYCrCb.val[0], scalarUpperYCrCb.val[1], scalarUpperYCrCb.val[2]);

        listen1 = new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam1.setPipeline(myPipeline1);
                webcam1.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        };
        listen2 = new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam2.setPipeline(myPipeline2);
                webcam2.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        };
        listenClose1 = new OpenCvCamera.AsyncCameraCloseListener() {
            @Override
            public void onClose() {
                // webcam1.stopStreaming();
                webcam1.stopRecordingPipeline();
                webcam1.closeCameraDevice();
            }
        };
        listenClose2 = new OpenCvCamera.AsyncCameraCloseListener() {
            @Override
            public void onClose() {
                // webcam2.stopStreaming();
                webcam2.stopRecordingPipeline();
                webcam2.closeCameraDevice();
            }
        };
    }

    public void startOpenCV() {
        webcam1.openCameraDeviceAsync(listen1);
        webcam2.openCameraDeviceAsync(listen2);
    }

    public void closeOpenCV()
    {
        webcam1.closeCameraDeviceAsync(listenClose1); // Empty camera
        webcam2.closeCameraDeviceAsync(listenClose2);
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

}