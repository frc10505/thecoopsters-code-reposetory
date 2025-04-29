package frc.team10505.robot;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.team10505.robot.subsystems.AlgaeSubsystem;

public class RobotContainer {

    private final CommandJoystick joystick = new CommandJoystick(0);
    private final CommandJoystick joystick2 = new CommandJoystick(1);

    /* Subsystems */
    private final AlgaeSubsystem algaeSubsys = new AlgaeSubsystem();

    /* Algae Pivot Controls Sim */
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
        if (Utils.isSimulation()) {
            joystick.button(1).onTrue(algaeSubsys.setAngle(5));
            joystick.button(2).onTrue(algaeSubsys.setAngle(15));
            joystick.button(3).onTrue(algaeSubsys.setAngle(25));
            joystick.button(4).onTrue(algaeSubsys.setAngle(25));
        }
    }

    /* Algae Intake Controls Sim */
    private void simConfigAlgaeIntakeButtonBindings() {
        if (Utils.isSimulation()) {
            joystick2.button(1).whileTrue(algaeSubsys.runIntake(30));
            joystick2.button(2).whileTrue(algaeSubsys.runIntake(-30));
            joystick2.button(3).onTrue(algaeSubsys.runIntake(0));

        }
    }

    public RobotContainer() {
        if (Utils.isSimulation()) {
            simConfigAlgaePivotButtonBindings();
            simConfigAlgaeIntakeButtonBindings();

        } else {
            configAlgaePivotButtonBindings();

        }
    }

}
