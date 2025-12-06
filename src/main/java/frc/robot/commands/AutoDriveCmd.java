package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoDriveCmd extends Command {
    private final SwerveSubsystem swerve;
    private final Timer timer = new Timer();
    private final double durationSec;
    private final double xSpeed, ySpeed, rotSpeed;

    public AutoDriveCmd(SwerveSubsystem swerve, double durationSec, 
                       double xSpeed, double ySpeed, double rotSpeed) {
        this.swerve = swerve;
        this.durationSec = durationSec;
        this.xSpeed = xSpeed * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        this.ySpeed = ySpeed * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        this.rotSpeed = rotSpeed * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        // Robot-relative speeds (like joystick command)
        ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);
        
        // Convert to module states using your kinematics
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
        
        // Desaturate to prevent impossible speeds
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, DriveConstants.kTeleDriveMaxSpeedMetersPerSecond);
        
        // Use your subsystem's method
        swerve.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stopModules();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(durationSec);
    }
}
