package frc.team10505.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeSubsystem extends SubsystemBase {

    private final int pivotMotorId = 0;
    private final int intakeMotorId = 0;

    private final double startingAngle = 0.0;

    /* Motor Ids */
    private TalonFX pivotMotor = new TalonFX(pivotMotorId);
    private TalonFX intakeMotor = new TalonFX(intakeMotorId);

    /* PID FeedFoward */
    private PIDController pivotController = new PIDController(0, 0, 0);
    private ArmFeedforward pivotFeedForward = new ArmFeedforward(0, 0, 0, 0);

    /* Fancy Sim stuff */
    private final Mechanism2d pivotMech = new Mechanism2d(2, 2.5);
    private MechanismRoot2d pivotRoot = pivotMech.getRoot("pivotRoot", 1, 1.25);
    private MechanismLigament2d pivotViz = pivotRoot.append(new MechanismLigament2d("pivotViz", 0.75, 0));
    private SingleJointedArmSim pivotSim = new SingleJointedArmSim(DCMotor.getKrakenX60(2), 80,
            SingleJointedArmSim.estimateMOI(0.305, 2), 0.305, Units.degreesToRadians(-110), Units.degreesToRadians(110),
            true, startingAngle, Units.degreesToRadians(startingAngle));

    public AlgaeSubsystem() {
        if (Utils.isSimulation()) {
            pivotMotor = new TalonFX(pivotMotorId);
            intakeMotor = new TalonFX(intakeMotorId);
            pivotController = new PIDController(0, 0, 0);
            pivotFeedForward = new ArmFeedforward(0, 0, 0, 0);

        } else {
            pivotMotor = new TalonFX(pivotMotorId);
            intakeMotor = new TalonFX(intakeMotorId);
            pivotController = new PIDController(0, 0, 0);
            pivotFeedForward = new ArmFeedforward(0, 0, 0, 0);

        }

    }

}
