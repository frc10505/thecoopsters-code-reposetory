package frc.team10505.robot;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.team10505.robot.subsystems.AlgaeSubsystem;

public class RobotContainer {

    private final CommandJoystick joystick = new CommandJoystick(0);
    private final CommandJoystick joystick2 = new CommandJoystick(1);

    /* Subsystems */
    private final AlgaeSubsystem algaeSubsys = new AlgaeSubsystem();

    private void simConfigAlgaeButtonBindings() {
        if (Utils.isSimulation()) {
            joystick.button(1).onTrue(algaeSubsys.setAngle(0));
            joystick.button(2).onTrue(algaeSubsys.setAngle(45));
            joystick.button(3).onTrue(algaeSubsys.setAngle(-90));
            joystick.button(4).onTrue(algaeSubsys.setAngle(90));
        }
    }

    private void configAlgaeButtonBindings() {
        if (Utils.isSimulation()) {
            joystick.button(1).onTrue(algaeSubsys.setAngle(5));
            joystick.button(2).onTrue(algaeSubsys.setAngle(15));
            joystick.button(3).onTrue(algaeSubsys.setAngle(25));
            joystick.button(4).onTrue(algaeSubsys.setAngle(25));
        }
    }

    public RobotContainer() {
        if (Utils.isSimulation()) {
            simConfigAlgaeButtonBindings();

        } else {
            configAlgaeButtonBindings();

        }
    }

}
