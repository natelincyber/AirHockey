package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp
public class Main extends LinearOpMode {
    public OpenCvCamera camera;
    public MainPipeline cv;
    Servo robotArm;
    Timer timer;

    @Override
    public void runOpMode() throws InterruptedException {
        timer = new Timer(this);

        robotArm = hardwareMap.get(Servo.class, "s");
        camera = cameraInit();
        startStream();

        robotArm.setPosition(0);
        timer.safeDelay(500);
        robotArm.setPosition(1);
        timer.safeDelay(500);
        robotArm.setPosition(0.5);

        telemetry.addLine("Systems Initialized. Ready for start");
        telemetry.update();


        waitForStart();

        while (opModeIsActive()) {
            if (cv.moveServo(cv.getCenterError(), 0.5) == 0) {
                robotArm.setPosition(0.5);
            }
            robotArm.setPosition(Math.abs(cv.moveServo(cv.getCenterError(), 0.5)));
            telemetry.addData("bruh", cv.moveServo(cv.getCenterError(), 0.5));
            telemetry.addData("Servo Position", robotArm.getPosition());
            telemetry.addData("centerError", cv.getCenterError());
            telemetry.update();
        }
    }

    public OpenCvCamera cameraInit() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        return OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
    }

    public void startStream() {
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // GPU-accelerated render!
//                camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                requestOpModeStop();
            }
        });
        cv = new MainPipeline(telemetry);
        camera.setPipeline(cv);
    }
}
