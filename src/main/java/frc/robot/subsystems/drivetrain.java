package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.photonvision.EstimatedRobotPose;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.vision;
import frc.robot.Constants.driveConstants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

public class drivetrain extends SubsystemBase{
    File swerveJsonDir = new File(Filesystem.getDeployDirectory(),"swerve");
	SwerveDrive drive;

    private SwerveAutoBuilder autoBuilder = null;

    // PID controller for balancing
    private final PIDController balanceController;

    private final vision Vision;

    public drivetrain(vision Vision) {
        // set the balance controller's P, I, D, tolerance, and setpoint
		balanceController  = new PIDController(
            driveConstants.balanceP,
            driveConstants.balanceI,
            driveConstants.balanceD);
        balanceController.setTolerance(0.3, 1);
        balanceController.setSetpoint(0);

        this.Vision = Vision;

        // set the swerve telemetry's verbosity
        SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;

        // create the drivetrain from the config files
        try {
            drive = new SwerveParser(swerveJsonDir).createSwerveDrive();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    public void periodic() {
        updatePose();
        drive.updateOdometry();
        SmartDashboard.putNumber("swerve/pose x", getPose().getX());
        SmartDashboard.putNumber("swerve/pose y", getPose().getY());
        SmartDashboard.putNumber("swerve/angle", getYaw().getDegrees());
    }

    private void updatePose() {
        Optional<EstimatedRobotPose> frontResult = Vision.getFrontPose(drive.getPose());
        if (frontResult.isPresent()) {
			EstimatedRobotPose camPose = frontResult.get();
			drive.addVisionMeasurement(
                camPose.estimatedPose.toPose2d(),
                camPose.timestampSeconds, false, 1);
		}
        
        Optional<EstimatedRobotPose> backResult = Vision.getFrontPose(drive.getPose());
        if (backResult.isPresent()) {
			EstimatedRobotPose camPose = backResult.get();
			drive.addVisionMeasurement(
                camPose.estimatedPose.toPose2d(),
                camPose.timestampSeconds, true, 1);
		}
    }

    public Rotation2d getRoll() {
        return drive.getRoll();
    }

    public Rotation2d getPitch() {
        return drive.getPitch();
    }

    public Rotation2d getYaw() {
        return drive.getYaw();
    }

    public double getBalanceSpeed(double gyroReading) {
        return balanceController.calculate(gyroReading);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
		drive.drive(translation, rotation, fieldRelative, isOpenLoop);
	}

    public Command driveWithJoysticks(DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier rotation, BooleanSupplier FieldRelative) {
        return this.run(() -> drive.drive(new Translation2d(
            -MathUtil.applyDeadband(vX.getAsDouble(), 0.04) * drive.getSwerveController().config.maxSpeed,
            -MathUtil.applyDeadband(vY.getAsDouble(), 0.04)* drive.getSwerveController().config.maxSpeed),
            -MathUtil.applyDeadband(rotation.getAsDouble(), 0.09) * drive.getSwerveController().config.maxAngularVelocity,
            FieldRelative.getAsBoolean(), false));
    }

    public void lock() {
        drive.lockPose();
    }

    public Pose2d getPose() {
        return drive.getPose();
    }

    private void setChassisSpeeds(ChassisSpeeds speeds) {
        drive.setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getPitch()));
        // drive(new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond), speeds.omegaRadiansPerSecond, true, false);
    }

    public void defineAutoBuilder(Map<String, Command> eventMap,
								  PIDConstants translationPID,
								  PIDConstants rotationPID, boolean useAllianceColor) {
		if (autoBuilder == null) {
			autoBuilder = new SwerveAutoBuilder(
					drive::getPose,
					drive::resetOdometry,
					translationPID,
					rotationPID,
					this::setChassisSpeeds,
					eventMap,
					useAllianceColor,
					this);
		}
	}

    public Command createTrajectory(String path, PathConstraints constraints) {
		List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(path, constraints);

		return autoBuilder.fullAuto(pathGroup);
	}
}
