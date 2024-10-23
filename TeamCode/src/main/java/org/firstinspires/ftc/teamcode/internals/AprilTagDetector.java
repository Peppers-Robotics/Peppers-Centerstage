package org.firstinspires.ftc.teamcode.internals;

    import com.acmerobotics.dashboard.FtcDashboard;
    import com.acmerobotics.dashboard.config.Config;
    import com.qualcomm.robotcore.hardware.HardwareMap;

        import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
    import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
    import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
    import org.openftc.apriltag.AprilTagDetection;
        import org.openftc.easyopencv.OpenCvCamera;
        import org.openftc.easyopencv.OpenCvCameraFactory;
        import org.openftc.easyopencv.OpenCvCameraRotation;
    import org.openftc.easyopencv.OpenCvWebcam;


    import java.util.ArrayList;
    import java.util.concurrent.TimeUnit;

    import kotlin.time.DurationUnit;

@Config
public class AprilTagDetector {
    public static OpenCvWebcam camera = null;
    private static AprilTagDetectionPipeline pipeline;
    public static double fx = 450.474;
    public static double fy = 450.474;
    public static double cx = 317.178;
    public static double cy = 186.578;

    public static double tagSize = 0.05;
    public static int ID = 0;
    public static boolean OPENED = false;


    public static void init(HardwareMap hardwareMap)
    {
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"));
        pipeline = new AprilTagDetectionPipeline(tagSize, fx, fy, cx, cy);

        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                try {
                    OPENED = true;
                    camera.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);
                    FtcDashboard.getInstance().startCameraStream(AprilTagDetector.camera, 30);
                } catch (Exception e) {
                    OPENED = false;
                }

            }

            @Override
            public void onError(int errorCode)
            {
                OPENED = false;
            }
        });

        camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.NATIVE_VIEW);
    }

    public static void setCamera(OpenCvWebcam camera)
    {
        AprilTagDetector.camera = camera;
        pipeline = new AprilTagDetectionPipeline(tagSize, 822.317, 822.317, 319.495, 242.502);

        AprilTagDetector.camera.setPipeline(pipeline);
    }

    public static AprilTagDetection[] getDetections()
    {
        ArrayList<AprilTagDetection> currentDetections = pipeline.getLatestDetections();
        AprilTagDetection[] detected = new AprilTagDetection[currentDetections.size()];
        for(int i = 0; i < currentDetections.size(); i++) {
            detected[i] = currentDetections.get(i);
        }
        return detected;
    }
}
