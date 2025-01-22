// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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
  public static class OperatorConstants {
    public static final double kJoystickDeadBand = 0.1;
    public static final int kDriverControllerPort = 0;
  }

  /*
   * goal[0] = goalAngle
   * goal[1] = speedMetersPerSecond
   */
  public static Double[] optimate(double currentAngle, double goalAngle, double speedMetersPerSecond){
    Double[] goal = new Double[2];
    double delta = Math.abs(goalAngle - currentAngle);
    if (delta > (Math.PI / 2)) {
      goalAngle = goalAngle - Math.PI;
      speedMetersPerSecond = speedMetersPerSecond * -1;
    }
    goal[0] = goalAngle;
    goal[1] = speedMetersPerSecond;
    return goal;
  }

  public static class ModuleConstants {

    public static final double pidRangeMin = -180;
    public static final double pidRangeMax = 180;

    public static final double wheelDiameterMeters = Units.inchesToMeters(4);
    public static final double driveGearRatio = 1/5.36;
    public static final double turningGearRatio = 1.0/(150/7);

    public static final double driveEncoderRot2Meter = driveGearRatio*Math.PI*wheelDiameterMeters;

    public static final double kModuleDistance = 22.24*0.0254;

    public static SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
      new Translation2d(kModuleDistance/2, kModuleDistance/2),
      new Translation2d(kModuleDistance/2, -kModuleDistance/2),
      new Translation2d(-kModuleDistance/2, kModuleDistance/2),
      new Translation2d(-kModuleDistance/2, -kModuleDistance/2)
    );

    public static final double turningPidController_Kp = 0.012;
    public static final double turningPidController_Ki = 0;
    public static final double turningPidController_Kd = 0;

    public static final double drivePidController_Kp = 0.2;
    public static final double drivePidController_Ki = 0;
    public static final double drivePidController_Kd = 0;

    public static final double driveFeedforward_Ks = 0.05;
    public static final double driveFeedforward_Kv = 2.50;

  }

  public class SwerveConstants {
    public static final int rightFrontDrive_ID = 1;
    public static final int rightBackDrive_ID = 2;
    public static final int leftFrontDrive_ID = 4;
    public static final int leftBackDrive_ID = 3;

    public static final int rightFrontTurning_ID = 5;
    public static final int rightBackTurning_ID = 6;
    public static final int leftFrontTurning_ID = 8;
    public static final int leftBackTurning_ID = 7;

    public static final int rightFrontAbsolutedEncoder_ID = 41;
    public static final int rightBackAbsolutedEncoder_ID = 42;
    public static final int leftFrontAbsolutedEncoder_ID = 44;
    public static final int leftBackAbsolutedEncoder_ID = 43;

    public static final double leftFrontOffset = -0.343994;
    public static final double leftBackOffset = 0.220215;
    public static final double rightFrontOffset = -0.350586;
    public static final double rightBackOffset = 0.149170;

    public static final int gyro_ID = 56;

    public static final double kModuleDistance = 22.24*0.0254;
    public static SwerveDriveKinematics swerveKineatics = new SwerveDriveKinematics(
      new Translation2d(kModuleDistance/2, kModuleDistance/2),
      new Translation2d(kModuleDistance/2, -kModuleDistance/2),
      new Translation2d(-kModuleDistance/2, kModuleDistance/2),
      new Translation2d(-kModuleDistance/2, -kModuleDistance/2));

    
    public static final double pathingMoving_Kp = 0;
    public static final double pathingMoving_Ki = 0;
    public static final double pathingMoving_Kd = 0;

    public static final double pathingtheta_Kp = 0;
    public static final double pathingtheta_Ki = 0;
    public static final double pathingtheta_Kd = 0;

    public static final double maxOutput = 0;

    public static final double maxDriveSpeed_MeterPerSecond = 5.94;
    public static final double kDriveBaseRadius = 15.73 * 0.0254;
    public static final double maxAngularVelocity_Angle = 50;

  }

  public class PhotonConstants {
    public static final double xPidController_Kp = 0;
    public static final double xPidController_Ki = 0;
    public static final double xPidController_Kd = 0;

    public static final double yPidController_Kp = 0;
    public static final double yPidController_Ki = 0;
    public static final double yPidController_Kd = 0;

    public static final double zPidController_Kp = 0;
    public static final double zPidController_Ki = 0;
    public static final double zPidController_Kd = 0;

    public static final double rotationPidController_Kp = 0;
    public static final double rotationPidController_Ki = 0;
    public static final double rotationPidController_Kd = 0;

    
    
  }
}
