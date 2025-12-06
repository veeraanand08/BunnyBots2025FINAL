
package frc.robot;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.BucketCmd;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.AutoDriveCmd;
import frc.robot.commands.LowerBucketAutoCmd;
import frc.robot.commands.RaiseBucketAutoCmd;
import frc.robot.subsystems.BucketSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.SwerveModule;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;


public class RobotContainer {

    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final BucketSubsystem bucketSubsystem = new BucketSubsystem();

    private final XboxController controller = new XboxController(OIConstants.kDriverControllerPort);

    public RobotContainer() {
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> controller.getLeftY(),
                () -> -controller.getLeftX(), 
                controller::getRightX,             
                true
        ));
        
        bucketSubsystem.setDefaultCommand(new BucketCmd(
                bucketSubsystem,
                controller::getLeftBumperButton,
                controller::getRightBumperButton
        ));

        autoChooser.setDefaultOption("Auto Drive Only", new AutoDriveCmd(
            swerveSubsystem, 1, 1, 0, 0
        ));

        autoChooser.addOption("Outer",
            new SequentialCommandGroup(
                new RaiseBucketAutoCmd(bucketSubsystem),
                new WaitCommand(3),
                new AutoDriveCmd(swerveSubsystem, 2.5, -1, 0, 0),
                new LowerBucketAutoCmd(bucketSubsystem)
            )
        );

        autoChooser.addOption("Outer2",
            new SequentialCommandGroup(
                new RaiseBucketAutoCmd(bucketSubsystem),
                new WaitCommand(3),
                //new AutoDriveCmd(swerveSubsystem, 3, -1, 0, 0),
                new LowerBucketAutoCmd(bucketSubsystem)
            )
        );
        
        autoChooser.addOption("Inner Blue",
            new SequentialCommandGroup(
                new RaiseBucketAutoCmd(bucketSubsystem),
                new WaitCommand(3),
                new AutoDriveCmd(swerveSubsystem, 5, 0, -1, 0),
                new AutoDriveCmd(swerveSubsystem, 4, -2, 0, 0),
                new LowerBucketAutoCmd(bucketSubsystem)
            )
        );

        autoChooser.addOption("Inner Red",
            new SequentialCommandGroup(
                new RaiseBucketAutoCmd(bucketSubsystem),
                new WaitCommand(3),
                new AutoDriveCmd(swerveSubsystem, 5, 0, 1, 0),
                new AutoDriveCmd(swerveSubsystem, 4, -2, 0, 0),
                new LowerBucketAutoCmd(bucketSubsystem)
            )
        );

        SmartDashboard.putData("Autonomous Routine", autoChooser);

        }

        public Command getAutonomousCommand(){
            return autoChooser.getSelected();
        }
        
    }


