package frc.team10505.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.ParentConfiguration;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.team10505.robot.generated.TunerConstants;
import frc.team10505.robot.subsystems.AlgaeSubsystem;
import frc.team10505.robot.subsystems.CoralSubsystem;
import frc.team10505.robot.subsystems.DrivetrainSubsystem;
import frc.team10505.robot.subsystems.ElevatorSubsystem;

public class RobotContainer {

    private final CommandJoystick joystick = new CommandJoystick(0);
    private final CommandJoystick joystick2 = new CommandJoystick(1);
    private final CommandJoystick joystick3 = new CommandJoystick(2);
    private final CommandJoystick joystick4 = new CommandJoystick(3);
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    /* Subsystems */
    private final AlgaeSubsystem algaeSubsys = new AlgaeSubsystem();
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final CoralSubsystem coralSubsystem = new CoralSubsystem();
    private final DrivetrainSubsystem driveTrainSubsys = TunerConstants.createDrivetrain();
    /* Algae Controls */

    private void simAlgaePivotButtonBindings() {
        if (Utils.isSimulation()) {
            joystick.button(1).onTrue(algaeSubsys.setAngle(90));
            joystick.button(2).onTrue(algaeSubsys.setAngle(45));
            joystick.button(3).onTrue(algaeSubsys.setAngle(-45));
            joystick.button(4).onTrue(algaeSubsys.setAngle(-90));
        }
    }

    private void algaePivotButtonBindings() {// cooper is a bafoon

        driverController.x().onTrue(algaeSubsys.setAngle(90));
        driverController.y().onTrue(algaeSubsys.setAngle(45));
        driverController.b().onTrue(algaeSubsys.setAngle(-25.67));// Roasted Toasted
        driverController.a().onTrue(algaeSubsys.setAngle(0));

    }

    private void algaeIntakeButtonBindings() {
        if (Utils.isSimulation()) {
            joystick2.button(1).whileTrue(algaeSubsys.runIntake(1 * 2));
            joystick2.button(2).whileTrue(algaeSubsys.runIntake(-1 * 2));
            joystick2.button(3).onTrue(algaeSubsys.runIntake(0));

        } else {
            driverController.leftTrigger().whileTrue(algaeSubsys.runIntake(0.5));
            driverController.rightTrigger().whileTrue(algaeSubsys.runIntake(-0.5));
            driverController.rightBumper().onTrue(algaeSubsys.runIntake(0));
        }
    }

    /* Coral Button Bindings */

    private void coralButtonBindings() {
        if (Utils.isSimulation()) {
            joystick3.button(1).onTrue(coralSubsystem.runIntake(-15));
            joystick3.button(2).onTrue(coralSubsystem.runIntake(15));
            joystick3.button(3).onTrue(coralSubsystem.runIntake(0));

        } else {
            driverController.povDown().onTrue(coralSubsystem.runIntake(0));
            driverController.povUp().onTrue(coralSubsystem.runIntake(0.2));
        }
    }

    /* Elevator Button Bindings */

    private void elevatorButtonBindings() {
        if (Utils.isSimulation()) {
            joystick4.button(1).onTrue(elevatorSubsystem.setHeight(0));
            joystick4.button(2).onTrue(elevatorSubsystem.setHeight(20));
            joystick4.button(3).onTrue(elevatorSubsystem.setHeight(40));
            joystick4.button(4).onTrue(elevatorSubsystem.setHeight(60));
        } else {
            operatorController.a().onTrue(elevatorSubsystem.setHeight(0));
            operatorController.b().onTrue(elevatorSubsystem.setHeight(0));
            operatorController.y().onTrue(elevatorSubsystem.setHeight(0));
            operatorController.x().onTrue(elevatorSubsystem.setHeight(0));
        }
    }

    private final SendableChooser<Command> autonChooser;

    /* Contructor */

    public RobotContainer() {
        if (Utils.isSimulation()) {
            simAlgaePivotButtonBindings();
            algaeIntakeButtonBindings();
            coralButtonBindings();
            elevatorButtonBindings();

        } else {
            algaePivotButtonBindings();
            algaeIntakeButtonBindings();
            coralButtonBindings();

        }
        NamedCommands.registerCommand("test", Commands.print("donny will never get makayla"));
        driveTrainSubsys.configPathPlanner();
        autonChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autonChooser);

    }

}
