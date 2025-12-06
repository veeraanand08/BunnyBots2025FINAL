package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ModuleConstants;
import frc.robot.subsystems.BucketSubsystem;

public class BucketCmd extends Command {

    private final BucketSubsystem bucketSubsystem;
    private final Supplier<Boolean> leftBumper, rightBumper;

    public BucketCmd(BucketSubsystem bucketSubsystem,
            Supplier<Boolean> leftBumper, Supplier<Boolean> rightBumper) {
        this.bucketSubsystem = bucketSubsystem;
        this.leftBumper = leftBumper;
        this.rightBumper = rightBumper;
        addRequirements(bucketSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (leftBumper.get()) {
            if (bucketSubsystem.bucketState != BucketSubsystem.BucketState.LOWERED) {
                bucketSubsystem.bucketState = BucketSubsystem.BucketState.LOWERING;
            }
        }
        else if (rightBumper.get()) {
            if (bucketSubsystem.bucketState != BucketSubsystem.BucketState.RAISED) {
                bucketSubsystem.bucketState = BucketSubsystem.BucketState.RAISING;
            }
        }

        double currentAngle = bucketSubsystem.getPositionDeg();
        switch (bucketSubsystem.bucketState) {
            case RAISING:
                if (currentAngle >= ModuleConstants.kBucketEngagedAngle-5)
                    bucketSubsystem.bucketState = BucketSubsystem.BucketState.RAISED;
            case RAISED:
                bucketSubsystem.setMotorAngle(ModuleConstants.kBucketEngagedAngle);
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
    public void end(boolean interrupted) {
        bucketSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
