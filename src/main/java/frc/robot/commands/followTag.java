package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.vision;
import frc.robot.subsystems.drivetrain;

public class followTag extends CommandBase{
    private drivetrain drive;
    private vision Vision;

    public followTag(drivetrain Drive, vision vis) {
        drive = Drive;
        Vision = vis;
    }

    @Override
    public void execute() {
        ChassisSpeeds speeds = Vision.followTag();
        drive.drive(new Translation2d(
            speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond), 
            speeds.omegaRadiansPerSecond, false, false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(new Translation2d(
            0,
            0), 
            0, false, false);
    }
}
