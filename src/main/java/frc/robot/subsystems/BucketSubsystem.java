package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;


public class BucketSubsystem extends SubsystemBase {
    private final SparkMax leftMotor;
    private final SparkMax rightMotor;

    private final SparkMaxConfig leftConfig;
    private final SparkMaxConfig rightConfig;

    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;

    private final PIDController leftPIDController;
    private final PIDController rightPIDController;

    public BucketState bucketState;

    public BucketSubsystem() {
        leftMotor = new SparkMax(DriveConstants.kLeftBucketMotor, MotorType.kBrushless);
        rightMotor = new SparkMax(DriveConstants.kRightBucketMotor, MotorType.kBrushless);
        leftConfig = new SparkMaxConfig();
        rightConfig = new SparkMaxConfig();

        leftConfig.inverted(DriveConstants.kLeftBucketMotorReversed);
        rightConfig.inverted(DriveConstants.kRightBucketMotorReversed);
        leftConfig.encoder.positionConversionFactor(ModuleConstants.kBucketEncoderRot2Rad);
        rightConfig.encoder.positionConversionFactor(ModuleConstants.kBucketEncoderRot2Rad);
        leftConfig.idleMode(IdleMode.kBrake);
        rightConfig.idleMode(IdleMode.kBrake);

        leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        leftEncoder = leftMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();

        leftPIDController = new PIDController(ModuleConstants.kPBucket, ModuleConstants.kIBucket, ModuleConstants.kDBucket);
        rightPIDController = new PIDController(ModuleConstants.kPBucket, ModuleConstants.kIBucket, ModuleConstants.kDBucket);

        resetEncoders();
        bucketState = BucketState.LOWERED;
    }

    public void setMotorAngle(double angle) {
        leftMotor.set(leftPIDController.calculate(getLeftPosition(), Math.toRadians(angle)));
        rightMotor.set(rightPIDController.calculate(getRightPosition(), Math.toRadians(angle)));
    }

    public void setMotorSpeed(double speed) {
        leftMotor.set(speed);
        rightMotor.set(speed);
    }

    public double getPositionDeg() { return Math.toDegrees((getLeftPosition() + getRightPosition()) / 2); }
    public double getLeftPosition() { return leftEncoder.getPosition(); }
    public double getRightPosition() { return rightEncoder.getPosition(); }

    public void resetEncoders() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

    public void stop() {
        leftMotor.set(0);
        rightMotor.set(0);
    }

    public enum BucketState {
        RAISING, RAISED, LOWERING, LOWERED
    }
}
