package frc.robot;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.collectorConstants;
import frc.robot.Constants.shelves;

public class vision {

    private PhotonCamera frontCamera;
    private PhotonCamera backCamera;

    private PhotonPoseEstimator frontPoseEstimator;
    private PhotonPoseEstimator backPoseEstimator;

    public vision(String frontName, Transform3d robotToFrontCam,
                String backName, Transform3d robotToBackCam) {
        // set up a photon vision camera
        frontCamera = new PhotonCamera(frontName);
        frontCamera.setDriverMode(false);

        backCamera = new PhotonCamera(backName);
        backCamera = new PhotonCamera(backName);

        // if it is a simulation, disable some error throwing
        if (RobotBase.isSimulation()) {
            PhotonCamera.setVersionCheckEnabled(false);
        }

        try {
            // Attempt to load the AprilTagFieldLayout that will tell us where
            // the tags are on the field.
            AprilTagFieldLayout fieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
            // Create pose estimator
            frontPoseEstimator =
                    new PhotonPoseEstimator(
                            fieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP,
                            frontCamera, robotToFrontCam);
            frontPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

            backPoseEstimator =
                    new PhotonPoseEstimator(
                            fieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP,
                            backCamera, robotToBackCam);
            backPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        } catch (IOException e) {
            // The AprilTagFieldLayout failed to load. We won't be able to
            // estimate poses if we don't know where the tags are.
            DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
            frontPoseEstimator = null;
            backPoseEstimator = null;
        }
    }

    public double[] getShooterData(shelves shelf) {

        PhotonPipelineResult frontResults = frontCamera.getLatestResult();
        PhotonPipelineResult backResults = backCamera.getLatestResult();

        PhotonTrackedTarget frontBestTarget;
        PhotonTrackedTarget backBestTarget;

        double distance = 0;

        // check if either of the cameras have targets.
        // if they do get their best targets. defaults to the front camera
        if (frontResults.hasTargets()) {
            frontBestTarget = frontResults.getBestTarget();
            distance = frontBestTarget.getBestCameraToTarget().getX();
        }
        else if (backResults.hasTargets()) {
            backBestTarget = backResults.getBestTarget();
            distance = backBestTarget.getBestCameraToTarget().getX();
        }
        
        double angle = Math.atan(
                (2/distance) *
                        (shelf.heightdiff + Constants.visionConstants.maxHeight +
                                Math.sqrt(Math.pow(Constants.visionConstants.maxHeight, 2) +
                                    shelf.heightdiff *
                                                Constants.visionConstants.maxHeight)))+(Math.PI/2);

        double speed = (Math.sqrt(2* Constants.visionConstants.g*
        (shelf.heightdiff +
                Constants.visionConstants.maxHeight)))/Math.sin(angle)
                /(collectorConstants.shooterWheelDiameter *Math.PI);

        return new double[] {angle, speed};
    }

    public Optional<EstimatedRobotPose> getFrontPose(Pose2d prevEstimatedRobotPose) {
        if (frontPoseEstimator == null) {
            // The field layout failed to load, so we cannot estimate poses.
            return Optional.empty();
        }

        frontPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return frontPoseEstimator.update();
    }

    public Optional<EstimatedRobotPose> getBackPose(Pose2d prevEstimatedRobotPose) {
        if (backPoseEstimator == null) {
            // The field layout failed to load, so we cannot estimate poses.
            return Optional.empty();
        }

        backPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return backPoseEstimator.update();
    }

    public ChassisSpeeds followTag() {
        PhotonPipelineResult frontResults = frontCamera.getLatestResult();

        PhotonTrackedTarget frontBestTarget;

        double x = 1.5;
        double y = 0;
        double z = 0;

        // check if either of the cameras have targets.
        // if they do get their best targets. defaults to the front camera
        if (frontResults.hasTargets()) {
            frontBestTarget = frontResults.getBestTarget();
            // if (frontBestTarget.getFiducialId() == 1) {
                x = frontBestTarget.getBestCameraToTarget().getX();
                y = frontBestTarget.getBestCameraToTarget().getY();
                z = frontBestTarget.getBestCameraToTarget().getRotation().getX();
            // }

        }

        SmartDashboard.putNumber("tag x dist", x);
        SmartDashboard.putNumber("tag y dist", y);
        SmartDashboard.putNumber("tag z dist", z);
        // System.out.println(x);
        // System.out.println(y);
        // System.out.println(z);
        double vx = 0;
        double vy = 0;
        double vr = 0;

        vx = limitVel((x-1.5)*1.5);
        vy = limitVel((y)*1.8);
        vr = limitVel(z*-2);

        // if (x <= 1.49) {
        //     vx = -0.3;
        // }
        // else if (x >= 1.51) {
        //     vx = 0.3;
        // }

        return new ChassisSpeeds(vx, vy, vr);
    }

    private double limitVel(double vel) {
        if (vel > 2) {
            return 2;
        }
        else if (vel < -2) {
            return -2;
        }
        return vel;
    }
}
