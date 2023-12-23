// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.balance;
import frc.robot.commands.followTag;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.collector;
import frc.robot.subsystems.drivetrain;

public class RobotContainer {

  public vision Vision = new vision("frontCamera", Constants.visionConstants.robotToFrontCam, 
                          "backCamera", Constants.visionConstants.robotToFrontCam);

  public drivetrain drive = new drivetrain(Vision);

  public collector Collector = new collector(Vision);

  public LEDs leds = new LEDs();

  public CommandXboxController driveController = new CommandXboxController(0);
  public CommandXboxController coDriveController = new CommandXboxController(1);

  public SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    drive.setDefaultCommand(drive.driveWithJoysticks(
        driveController::getLeftY,
        driveController::getLeftX, driveController::getRightX,
        () -> SmartDashboard.getBoolean("is field oriented", true)));

    Collector.setDefaultCommand(Collector.setSpeed(-0.01));

    leds.setDefaultCommand(leds.update(Collector::getState));

    configureBindings();
    configureAutonomous();
  }

  public void configureAutonomous() {
    HashMap<String, Command> eventMap = new HashMap<>();

    // create a map of all events that can happen in auto
    eventMap.put("shootMid", Collector.shoot(Constants.shelves.mid));
    eventMap.put("shootHigh", Collector.shoot(Constants.shelves.high));
    eventMap.put("shootLow", Collector.shoot(Constants.shelves.low));
    eventMap.put("collect", Collector.collect());
    eventMap.put("shoot mid", Collector.setSpeed(0.6).
                                andThen(Commands.waitSeconds(0.5)).
                                andThen(Collector.setSpeed(0)));
//      eventMap.put("eject", angleController.turnToAngle(0).
//                andThen(shooter.runShooterSpeedForTime(0.5, 0.5)).
//                andThen(angleController.turnToAngle(0.35)));
    eventMap.put("balance", new balance(drive));

      // create the auto builder
    drive.defineAutoBuilder(
            eventMap,
            new PIDConstants(5.0, 0.0, 0.0),
            new PIDConstants(0.4, 0.0, 0.0), true);

    // create constraints for velocity and acceleration
    PathConstraints constraints = new PathConstraints(2.5, 1.5);

    // create the auto commands
    // autos for the left side of the community
//     Command threePieceBalanceLeft = drive.createTrajectory(
//             "3 piece balance clean",
//             constraints);

//     Command threePieceLeft = drive.createTrajectory(
//             "3 piece clean",
//             constraints);

//     Command fourPieceLeft = drive.createTrajectory(
//             "4 piece clean", constraints);

//     Command fivePieceLeft = drive.createTrajectory(
//             "5 piece clean", constraints);

    // create autos for the right side of the community

//     Command threePieceBalanceRight = drive.createTrajectory(
//             "3 piece balance bump", constraints);

//     Command threePieceRight = drive.createTrajectory(
//             "3 piece bump", constraints);

//     Command fourPieceRight = drive.createTrajectory(
//             "4 piece bump", constraints);

//     Command fivePieceRight = drive.createTrajectory(
//             "5 piece bump", constraints);

    // create autos for the middle of the community

//     Command leftTwoPieceCharge = drive.createTrajectory(
//             "left 2 piece charge", constraints);

//     Command rightTwoPieceCharge = drive.createTrajectory(
//             "right 2 piece charge", constraints);

//     Command onePieceCharge = drive.createTrajectory(
//             "1 piece charge", constraints);

//     Command threePieceCharge = drive.createTrajectory(
//             "3 piece balance mid", constraints);

    // add them to the auto chooser
    // left side autos
//     autoChooser.setDefaultOption("3 piece balance clean", threePieceBalanceLeft);
//     autoChooser.addOption("3 piece clean", threePieceLeft);
//     autoChooser.addOption("4 piece clean", fourPieceLeft);
//     autoChooser.addOption("5 piece clean", fivePieceLeft);
//     // right side autos
//     autoChooser.addOption("3 piece balance bump", threePieceBalanceRight);
//     autoChooser.addOption("3 piece bump", threePieceRight);
//     autoChooser.addOption("4 piece bump", fourPieceRight);
//     autoChooser.addOption("5 piece bump", fivePieceRight);
//     // middle autos
//     autoChooser.addOption("3 piece balance middle", threePieceCharge);
//     autoChooser.addOption("left 2 piece balance middle", leftTwoPieceCharge);
//     autoChooser.addOption("right 2 piece balance middle", rightTwoPieceCharge);
//     autoChooser.addOption("1 piece balance middle", onePieceCharge);

    Command onePiece = drive.createTrajectory("one piece", constraints);

    autoChooser.addOption("one piece", onePiece);

    Command threePiece = drive.createTrajectory("three piece", constraints);

    autoChooser.addOption("three piece", threePiece);

    // add a do nothing auto
    autoChooser.addOption("do nothing", Commands.none());

    // put the chooser to the dashboard
    SmartDashboard.putData("autoChooser", autoChooser);
  }

  private void configureBindings() {
    driveController.rightBumper().onTrue(Commands.runOnce(() -> SmartDashboard.putBoolean("is field oriented",true)));
    driveController.leftBumper().onTrue(Commands.runOnce(() -> SmartDashboard.putBoolean("is field oriented", false)));
	driveController.a().whileTrue(drive.run(() -> drive.drive(new Translation2d(1, 0), 0, false, false)));
//     driveController.b().onTrue(Commands.runOnce(() -> drive.lock()));

    coDriveController.leftBumper().onTrue(Collector.collect());

    coDriveController.a().onTrue(Collector.setSpeed(0.3).andThen(Commands.waitSeconds(1)));
    coDriveController.b().onTrue(Collector.setSpeed(0.6).andThen(Commands.waitSeconds(1)));
//     coDriveController.a().onTrue(Collector.shoot(Constants.shelves.low));
//     coDriveController.b().onTrue(Collector.shoot(Constants.shelves.mid));
//     coDriveController.y().onTrue(Collector.shoot(Constants.shelves.high));

    //outreach
    coDriveController.x().onTrue(Collector.collectInplace());

    coDriveController.rightBumper().whileTrue(new followTag(drive, Vision));
	// coDriveController.leftBumper().whileTrue(Commands.print("running the thing"));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
