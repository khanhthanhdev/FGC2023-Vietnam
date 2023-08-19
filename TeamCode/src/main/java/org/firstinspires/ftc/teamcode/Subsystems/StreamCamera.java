package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;


public class StreamCamera {

    private HardwareMap hardwareMap;
    OpenCvWebcam webcam;
    public StreamCamera(OpMode opMode) {
        hardwareMap = opMode.hardwareMap;
    }


    public void init(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

    }

    public class SimplePipeline extends OpenCvPipeline {
        public Mat processFrame(Mat input) {
            // you don't need this line now, but you might want it later on for some stuff
            // Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2YCrCb);
            return input;
        }
    }
    public void cameraStream(){
        FtcDashboard.getInstance().startCameraStream(webcam, 60);
        webcam.setPipeline(new SimplePipeline());

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }
            
            @Override
            public void onError(int errorCode) {}
        });


    }





}
