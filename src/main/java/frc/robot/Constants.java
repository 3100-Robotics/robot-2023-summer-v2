package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

public class Constants {
  public static class driveConstants {
      public static final double balanceP = 0.05;
      public static final double balanceI = 0;
      public static final double balanceD = 0;
  
      public static final double ROBOT_MASS = 45.35924; // 32lbs * kg per pound
  
      // a matter var for limiting velocity
      public static final Matter CHASSIS = new Matter(new Translation3d(0, 0,
              Units.inchesToMeters(4)), ROBOT_MASS);
  
      // loop time to use
      public static final double LOOP_TIME = 0.13;
  }

  public static class collectorConstants {
    public static final int angleMotorPort = 9;
    public static final int leftShooterPort = 10;
    public static final int rightShooterPort = 11;

    public static final double angleP = 0.05; //10.5
    public static final double angleI = 0; // 0.01
    public static final double angleD = 0; // 0
    public static final double angleFF = 0;

    public static final double shooterP = 0.1;
    public static final double shooterI = 0;
    public static final double shooterD = 0;

    public static final double shooterGearRatio = 5;
    public static final double shooterWheelDiameter = 0.3;

    public static final double collectorHomeAngle = 127;
    public static final double collectorCollectAngle = 3;
    public static final double collectorOffset = 0.8688;
  }

  public enum shelves {
    high(1) {},
    mid(20.5) {},
    low(31) {};

    public double heightdiff;

    shelves(double diff) {
        heightdiff = diff;
    } 
  }

  public enum collectorStates {
    collecting() {},
    moving() {},
    shooting() {},
    nothing() {};
  }

  public static class LEDConstants {
    public static final int PWMPort = 0;

    // number of LEDs
    public static final int length = 33;
  }

  public static class visionConstants {
    public static final double maxHeight = 6;
    public static final double g = 31;

    public static final Transform3d robotToFrontCam =
            new Transform3d(
                    new Translation3d(0.222, 0.238, 0),
                    new Rotation3d(
                            60, 0,
                            0));

    public static final Transform3d robotToBackCam =
            new Transform3d(
                    new Translation3d(-0.222, 0.238, 0),
                    new Rotation3d(
                            -60, 0,
                            0));
  }
}
