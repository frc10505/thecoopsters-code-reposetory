package frc.team10505.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

    /* Motor Id and Gear Stack */
    public final int elevLeadId = 30;
    public final int elevFollowId = 31;
    private int ELEVATOR_GEARSTACK = 12;

    /* Strange Configs */
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    /* Random Things */
    public Boolean usePID = true;
    private double elevatorEncoder = 0;
    private int ELEVATOR_MOTOR_CURRENT_LIMIT = 30;

    /* Motor Stuff */
    private TalonFX elevLead = new TalonFX(elevLeadId);
    private TalonFX elevFollow = new TalonFX(elevFollowId);

    /* Sim Variables */
    private final Mechanism2d elevatorMech = new Mechanism2d(3, 6);
    private MechanismRoot2d elevatorRoot = elevatorMech.getRoot("elevRoot", 1.5, 0);
    private MechanismLigament2d elevatorViz = elevatorRoot.append(new MechanismLigament2d("elevLigament", 1.5, 90));
    private ElevatorSim elevatorSim = new ElevatorSim(DCMotor.getKrakenX60(2),
            12, 10, 0.05, 0.0, 3.0, true, 0.6);

    /* Constructor */
    public ElevatorSubsystem() {
        if (Utils.isSimulation()) {
            SmartDashboard.putData("elevSimMech", elevatorMech);
            elevLead = new TalonFX(elevLeadId);
            elevFollow = new TalonFX(elevFollowId);
        } else {
            elevLead = new TalonFX(elevLeadId);
            elevFollow = new TalonFX(elevFollowId);

        }
    }

    // MotorOutputConfigs motorOutput = cfg.MotorOutput;
    // motorOutput.NeutralMode = NeutralModeValue.Brake;

    // StatusCode leaderStatus = StatusCode.StatusCodeNotInitialized;
    // for (int i = 0; i < 5; ++i) {
    // leaderStatus = elevLead.getConfigurator().apply(cfg);
    // if (leaderStatus.isOK())
    // break;
    // }
    // if (!leaderStatus.isOK()) {
    // System.out.println("Could not configure Elevator Motor. Error: " +
    // leaderStatus.toString());
    // }

    // StatusCode followerStatus = StatusCode.StatusCodeNotInitialized;
    // for (int i = 0; i < 5; ++i) {
    // followerStatus = elevFollow.getConfigurator().apply(cfg);
    // if (followerStatus.isOK())
    // break;
    // }
    // if (!followerStatus.isOK()) {
    // System.out.println("Could not configure Elevator Leader Motor. Error: " +
    // followerStatus.toString());
    // }

    // elevatorFollowerMotor.setControl(new Follower(elevatorMotor.getDeviceID(),
    // false));

}
