package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.jni.AprilTagJNI;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.Publisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class AprilTagVision extends SubsystemBase {

    public CvSink cvSink;
    public AprilTagDetector detector = new AprilTagDetector();
    public AprilTagDetector.Config config = new AprilTagDetector.Config();

    
    public DoubleArrayPublisher aprilTagInfo;
    private boolean debug = false;

    public AprilTagVision() {
        super();

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("datatable");
        aprilTagInfo = table.getDoubleArrayTopic("apriltags").publish();

        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
        
        // apriltag stuff, see https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/apriltag/AprilTagDetector.html
        detector.addFamily("tag16h5");
        detector.setConfig(config);
        // detector.addFamily("tag36h11"); not sure if it's use for this competition
        
        // run on new thread
        Thread vThread = new Thread(() -> tagDetection());
        vThread.setDaemon(true);
        vThread.start();
    }

    void tagDetection() {
        // setup camera & video
        UsbCamera camera = CameraServer.startAutomaticCapture();
        camera.setResolution(RobotMap.CAM_WID, RobotMap.CAM_HEI);

        cvSink = CameraServer.getVideo();
        CvSource video = CameraServer.putVideo("April Tag Detection", RobotMap.CAM_WID, RobotMap.CAM_HEI);

        // mat = matrix, very memory expensive, please reuse
        Mat mat = new Mat();
        Mat grayMat = new Mat();
        ArrayList<Integer> tags = new ArrayList<>();

        // colors for video
        Scalar outlineColor = new Scalar(0, 255, 0); // green
        Scalar xColor = new Scalar(0, 0, 255); // blue

        // can never be true, allows us to stop or restart bot
        while (!Thread.interrupted()) {

            // tells CvSink to grab frame and put it into source mat
            // == 0 if there is error
            if(cvSink.grabFrame(mat) == 0) {
                video.notifyError(cvSink.getError());
                continue;
            }    

            // convert mat to grayscalle
            Imgproc.cvtColor(mat, grayMat, Imgproc.COLOR_BGR2GRAY);

            AprilTagDetection[] detections = detector.detect(grayMat);
            tags.clear();

            // check if detected, otherwise do nothin
            if (detections.length != 0) {
                
                for (AprilTagDetection detection : detections) {
                    tags.add(detection.getId());
                    if(debug) {System.out.println("found id " + detection.getId());}
                        
                    
                    for (var i = 0; i <= 3; i++) {
                        var j = (i + 1) % 4;
                        var pt1 = new Point(detection.getCornerX(i), detection.getCornerY(i));
                        var pt2 = new Point(detection.getCornerX(j), detection.getCornerY(j));
                        Imgproc.line(mat, pt1, pt2, outlineColor, 2);
                    }
    
                    var cx = detection.getCenterX();
                    var cy = detection.getCenterY();
                    var ll = 10;
                    Imgproc.line(mat, new Point(cx - ll, cy), new Point(cx + ll, cy), xColor, 2);
                    Imgproc.line(mat, new Point(cx, cy - ll), new Point(cx, cy + ll), xColor, 2);
                    Imgproc.putText(mat, Integer.toString(detection.getId()), new Point (cx + ll, cy), Imgproc.FONT_HERSHEY_SIMPLEX, 1, xColor, 3);
                    
                    aprilTagInfo.set(new double[] {detection.getCenterX(), detection.getCenterY(), detection.getId()});
                    // add pose estimation code here if you wanna
                    // we don't really care about that stuff for this comp tho, 
                    // we're just using ATs for helping align the bot
                    // gonna preprogram movement for 15s autonomous
    
                }
            }

            

            // automatically managed, unnecessary
            // mat.release();
            // grayMat.release();

            SmartDashboard.putString("tag", tags.toString());
            video.putFrame(mat);
            
        }
    }

}