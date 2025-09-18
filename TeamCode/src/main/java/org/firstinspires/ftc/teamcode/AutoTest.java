package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Config
@Autonomous(name="Auto Test.java")
public final class      AutoTest extends LinearOpMode {
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    public WebcamName webcam;

    public enum GreenPosition {LEFT, MIDDLE, RIGHT, UNKNOWN}
    GreenPosition greenPosition = GreenPosition.UNKNOWN;

    @Override
    public void runOpMode() throws InterruptedException {
        //this code runs once after you press init
        webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(webcam, aprilTagProcessor);
        Pose2d beginPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        //visionPortal.resumeLiveView();
        //loops continously after init, before start
        while(opModeInInit()){
            List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
            telemetry.addData("# AprilTags Detected", currentDetections.size());

            // Step through the list of detections and display info for each one.
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                } else {
                    telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                    telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                }
                if (detection.id == 21) {
                    greenPosition = GreenPosition.LEFT;
                } else if (detection.id == 22) {
                    greenPosition = GreenPosition.MIDDLE;
                } else if (detection.id == 23) {
                    greenPosition = GreenPosition.RIGHT;
                }
                else{
                    greenPosition= GreenPosition.UNKNOWN;
                }

            }   // end for() loop
            telemetry.addData("green position",greenPosition);
            telemetry.update();
        }
        //this code runs as soon as you press start
        if (greenPosition == GreenPosition.LEFT){
            //RUN AUTO CODE FOR THIS RANDOMIZATION
        }

    }

}


