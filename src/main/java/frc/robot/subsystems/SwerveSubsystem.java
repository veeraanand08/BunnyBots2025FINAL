
package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import com.studica.frc.AHRS;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed);

    private final AHRS gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);

    SwerveModulePosition [] modulePositions = new SwerveModulePosition[] {
        frontLeft.getPosition(),
        frontRight.getPosition(),
        backLeft.getPosition(),
        backRight.getPosition()
      };

    SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, gyro.getRotation2d(), 
    modulePositions);

    //Waits a second for the gyro to bootstrap, then zeros the heading
    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(2000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    @Override
    public void periodic() {
        modulePositions = new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        };

        odometer.update(getRotation2d(), modulePositions);
        SmartDashboard.putNumber("Robot Heading", gyro.getYaw());
        SmartDashboard.putBoolean("Gyro Connected?", gyro.isConnected());
        SmartDashboard.putNumber("Gyro Port", gyro.getPort());
        SmartDashboard.putNumber("Gyro Update Count", gyro.getUpdateCount());
        SmartDashboard.putNumber("Gyro Byte Count", gyro.getByteCount());
        SmartDashboard.putNumber("Gyro Last Update Time", gyro.getLastSensorTimestamp());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    }

    public void zeroHeading() {
       gyro.reset();
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(-gyro.getAngle());
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), modulePositions, pose);
    }

    public SwerveModule[] getSwerveModules() {
        return new SwerveModule[] {
            frontLeft, frontRight, backLeft, backRight
        };
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }
}

