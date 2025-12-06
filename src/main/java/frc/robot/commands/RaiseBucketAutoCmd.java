package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ModuleConstants;
import frc.robot.subsystems.BucketSubsystem;
import frc.robot.subsystems.BucketSubsystem.BucketState;


public class RaiseBucketAutoCmd extends Command {
    private final BucketSubsystem bucketSubsystem;
    private final Timer raiseTimer;

    public RaiseBucketAutoCmd(BucketSubsystem bucketSubsystem) {
        this.bucketSubsystem = bucketSubsystem;
        raiseTimer = new Timer();
        addRequirements(bucketSubsystem);
    }

    @Override
    public void initialize() {
        bucketSubsystem.bucketState = BucketSubsystem.BucketState.RAISING;
        raiseTimer.reset();
        raiseTimer.start();
    }

    @Override
    public void execute() {
        double currentAngle = bucketSubsystem.getPositionDeg();
        SmartDashboard.putNumber("CurrentAngle", currentAngle);
        switch (bucketSubsystem.bucketState) {
            case RAISING:
                if (currentAngle >= 1000)
                    bucketSubsystem.bucketState = BucketSubsystem.BucketState.RAISED;
            case RAISED:
                bucketSubsystem.setMotorAngle(ModuleConstants.kBucketEngagedAngle);
                break;
            case LOWERING:
            case LOWERED:
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return bucketSubsystem.bucketState == BucketState.RAISED || raiseTimer.get() > 4.0;
    }

    @Override
    public void end(boolean interrupted) {
        bucketSubsystem.setMotorAngle(ModuleConstants.kBucketEngagedAngle);
        raiseTimer.stop();
    }
}
