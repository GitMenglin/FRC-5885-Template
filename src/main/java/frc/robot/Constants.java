package frc.robot;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode kCurrentMode = Robot.isReal() ? Mode.REAL : Mode.SIM;
  // public static final Mode kCurrentMode = Mode.REPLAY;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final double kLoopPeriodSecs = 0.02;

  public static final class ControllerConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriverControllerDeadband = 0.1;

    public static final int kOperatorControllerPort = 1;
    public static final double kOperatorControllerDeadband = 0.1;
  }

  public static enum ModuleLocation {
    FRONT_LEFT,
    FRONT_RIGHT,
    BACK_LEFT,
    BACK_RIGHT
  }

  public static final class DriveConstants {
    public static final double kTrackWidthMeters = Units.inchesToMeters(22.35);
    public static final double kTrackLengthMeters = Units.inchesToMeters(22.35);
    public static final double kWheelDiameterMeters = Units.inchesToMeters(3.45);

    public static final double kSdrive = 0.0;
    public static final double kVdrive = 0.0;
    public static final double kPdrive = 0.1;
    public static final double kIdrive = 0.0;
    public static final double kDdrive = 0.0;

    public static final double kPturn = 1.0;
    public static final double kIturn = 0.0;
    public static final double kDturn = 0.0;

    public static final double kMaxSpeedMetersPerSecond = 4.2;

    public static final class PathPlannerConstants {
      public static final PIDConstants kTranslationPID = new PIDConstants(0.1, 0.0, 0.0);
      public static final PIDConstants kRotationPID = new PIDConstants(0.1, 0.0, 0.0);
    }
  }

  public static class StateModelConstants {
    public static final double kVdrive = 0.1;
    public static final double kAdrive = 0.1;
    public static final Vector<N2> kQdrive =
        VecBuilder.fill(Units.degreesToRadians(1.0), Units.degreesToRadians(10.0));

    public static final double kVturn = 0.001;
    public static final double kAturn = 0.001;
    public static final Vector<N2> kQturn =
        VecBuilder.fill(Units.degreesToRadians(1.0), Units.degreesToRadians(10.0));

    public static final Matrix<N2, N1> kStateStdDevs = VecBuilder.fill(3.0, 3.0);
    public static final Matrix<N2, N1> kMeasurementStdDevs = VecBuilder.fill(0.01, 0.01);
  }

  public static final class SwerveModuleConstants {

    // I hope module number corresponds with encoder port

    public static final int kFrontLeftDriveMotorPort = 13;
    public static final int kFrontLeftTurningMotorPort = 23;
    public static final int kFrontLeftEncoderPort = 3;
    public static final double kFrontLeftEncoderOffset = 2.574;

    public static final int kFrontRightDriveMotorPort = 12;
    public static final int kFrontRightTurningMotorPort = 22;
    public static final int kFrontRightEncoderPort = 2;
    public static final double kFrontRightEncoderOffset = -0.345;

    public static final int kBackLeftDriveMotorPort = 10;
    public static final int kBackLeftTurningMotorPort = 20;
    public static final int kBackLeftEncoderPort = 0;
    public static final double kBackLeftEncoderOffset = -2.483;

    public static final int kBackRightDriveMotorPort = 11;
    public static final int kBackRightTurningMotorPort = 21;
    public static final int kBackRightEncoderPort = 1;
    public static final double kBackRightEncoderOffset = -1.296;

    // Gear ratios for SDS MK4i L2, adjust as necessary
    public static final double kDriveGearRatio = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
    public static final double kTurnGearRatio = 150.0 / 7.0;

    public static final int kDriveCurrentLimit = 40;
    public static final int kTurnCurrentLimit = 30;
  }

  public static enum BaseDriveMode {
    FIELD_ORIENTED,
    ROBOT_ORIENTED,
  }

  public static enum AimDriveMode {
    NONE,
    AIM_SPEAKER,
    AIM_NOTE,
    FACE_FORWARD,
    FACE_BACKWARD,
    FACE_AMP,
    FACE_SOURCE,
    LOB_PASS,
    TRACK_NOTE,
  }

  public static class AimLockConstants {
    public static final double kP = 10;
    public static final double kI = 0.0;
    public static final double kD = 0.5;
    public static final double kToleranceRad = Units.degreesToRadians(2);
  }

  public static class VisionConstants {
    public static final String kApriltagCameraName = "copperShooterCamera";
    public static final Transform3d kRobotToApriltagCam =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(11.56 + 0.39 + 16.0), 0.0, Units.inchesToMeters(13.876 + 1.9)),
            new Rotation3d(0, Units.degreesToRadians(-15), 0));
    public static final String kIntakeCameraName = "silverIntakeCamera";
    public static final Transform3d kRobotToIntakeCam =
        new Transform3d(
            new Translation3d(-0.24, -0.2, 0.5),
            new Rotation3d(0, Units.degreesToRadians(15), Units.degreesToRadians(180)));

    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout kTagLayout =
        AprilTagFields.kDefaultField.loadAprilTagLayoutField();

    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  }
}
