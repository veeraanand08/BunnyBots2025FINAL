package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ModuleConstants;
import frc.robot.subsystems.BucketSubsystem;
import frc.robot.subsystems.BucketSubsystem.BucketState;

public class LowerBucketAutoCmd extends Command {
    private final BucketSubsystem bucketSubsystem;

    public LowerBucketAutoCmd(BucketSubsystem bucketSubsystem) {
        this.bucketSubsystem = bucketSubsystem;
        addRequirements(bucketSubsystem);
    }

    @Override
    public void initialize() {
        bucketSubsystem.bucketState = BucketSubsystem.BucketState.LOWERING;
    }

    @Override
    public void execute() {
        double currentAngle = bucketSubsystem.getPositionDeg();
        switch (bucketSubsystem.bucketState) {
            case RAISING:
            case RAISED:
                break;
            case LOWERING:
                if (currentAngle <= 5) {
                    bucketSubsystem.bucketState = BucketSubsystem.BucketState.LOWERED;
                    break;
                }
                bucketSubsystem.setMotorSpeed(-0.075);
                break;
            case LOWERED:
                bucketSubsystem.stop();
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return bucketSubsystem.bucketState == BucketState.LOWERED;
    }

    @Override
    public void end(boolean interrupted) {
        bucketSubsystem.stop();
    }
}
