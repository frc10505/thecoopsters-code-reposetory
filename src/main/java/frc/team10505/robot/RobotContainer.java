package frc.team10505.robot;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.team10505.robot.subsystems.AlgaeSubsystem;
import frc.team10505.robot.subsystems.CoralSubsystem;
import frc.team10505.robot.subsystems.ElevatorSubsystem;

public class RobotContainer {

    private final CommandJoystick joystick = new CommandJoystick(0);
    private final CommandJoystick joystick2 = new CommandJoystick(1);
    private final CommandJoystick joystick3 = new CommandJoystick(2);
    private final CommandJoystick joystick4 = new CommandJoystick(3);
    private final CommandXboxController xboxController = new CommandXboxController(0);

    /* Subsystems */
    private final AlgaeSubsystem algaeSubsys = new AlgaeSubsystem();
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final CoralSubsystem coralSubsystem = new CoralSubsystem();

    private void simConfigAlgaePivotButtonBindings() {
        if (Utils.isSimulation()) {
            joystick.button(1).onTrue(algaeSubsys.setAngle(90));
            joystick.button(2).onTrue(algaeSubsys.setAngle(45));
            joystick.button(3).onTrue(algaeSubsys.setAngle(-45));
            joystick.button(4).onTrue(algaeSubsys.setAngle(-90));
        }
    }

    /* Algae Pivot Controls */
    private void configAlgaePivotButtonBindings() {
        xboxController.x().onTrue(algaeSubsys.setAngle(90));
        xboxController.y().onTrue(algaeSubsys.setAngle(45));
        xboxController.b().onTrue(algaeSubsys.setAngle(-25.67));//Roasted Toasted
        xboxController.a().onTrue(algaeSubsys.setAngle(0));

    }

    private void configAlgaeIntakeButtonBindings() {
        {
            xboxController.leftTrigger().whileTrue(algaeSubsys.runIntake(0.8));
            xboxController.rightTrigger().whileTrue(algaeSubsys.runIntake(-0.8));
            xboxController.rightBumper().onTrue(algaeSubsys.runIntake(0));

        }
    }

    /* Algae Intake Controls Sim */
    private void simConfigAlgaeIntakeButtonBindings() {
        if (Utils.isSimulation()) {
            joystick2.button(1).whileTrue(algaeSubsys.runIntake(1 * 2));
            joystick2.button(2).whileTrue(algaeSubsys.runIntake(-1 * 2));
            joystick2.button(3).onTrue(algaeSubsys.runIntake(0));

        }
    }

    private void simCoralButtonBindings() {
        if (Utils.isSimulation()) {
            joystick3.button(1).onTrue(coralSubsystem.runIntake(-15));
            joystick3.button(2).onTrue(coralSubsystem.runIntake(15));
            joystick3.button(3).onTrue(coralSubsystem.runIntake(0));

        }
    }

    private void simElevatorButtonBindings() {
        if (Utils.isSimulation()) {
            joystick4.button(1).onTrue(elevatorSubsystem.setHeight(0));
            joystick4.button(2).onTrue(elevatorSubsystem.setHeight(2));
            joystick4.button(3).onTrue(elevatorSubsystem.setHeight(4));
            joystick4.button(4).onTrue(elevatorSubsystem.setHeight(6));
        }
    }

    public RobotContainer() {
        if (Utils.isSimulation()) {
            simConfigAlgaePivotButtonBindings();
            simConfigAlgaeIntakeButtonBindings();
            simCoralButtonBindings();
            simElevatorButtonBindings();

        } else {
            configAlgaePivotButtonBindings();
            configAlgaeIntakeButtonBindings();

        }
    }

}
