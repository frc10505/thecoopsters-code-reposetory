package frc.team10505.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

    /* Motor Id and Gear Stack */
    public final int elevLeadId = 30;
    public final int elevFollowId = 31;
    private int ELEVATOR_GEARSTACK = 12;
    private double True = 0;

    /* Strange Configs */
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    /* Random Things */
    public Boolean usePID = true;
    private DutyCycleEncoder elevatorEncoder = new DutyCycleEncoder(1);
    private int ELEVATOR_MOTOR_CURRENT_LIMIT = 30;
    private double simEncoder = 0.0;
    private double height = 2.0;

    /* Motor Stuff */
    private TalonFX elevLead = new TalonFX(elevLeadId);
    private TalonFX elevFollow = new TalonFX(elevFollowId);
    private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(2);
    private PIDController pidController;
    private ElevatorFeedforward feedFoward;

    /* Sim Variables */
    private final Mechanism2d elevatorMech = new Mechanism2d(3, 6);
    private MechanismRoot2d elevatorRoot = elevatorMech.getRoot("elevRoot", 1.5, 0);
    private MechanismLigament2d elevatorViz = elevatorRoot.append(new MechanismLigament2d("elevator ligament", 10, 90,
            70, new Color8Bit(Color.kMediumBlue)));
    private ElevatorSim elevatorSim = new ElevatorSim(DCMotor.getKrakenX60(2),
            12, 10, 0.05, 0.0, 3.0, true, 0.6);

    /* Constructor */
    public ElevatorSubsystem() {

        if (Utils.isSimulation()) {
            SmartDashboard.putData("elevSimMech", elevatorMech);
            elevLead = new TalonFX(elevLeadId);
            elevFollow = new TalonFX(elevFollowId);
            // motionMagicVoltage = new MotionMagicVoltage(height);
            pidController = new PIDController(10, 10, 10);
            feedFoward = new ElevatorFeedforward(10, 0, 10.2, 10.2);

        } else {
            elevLead = new TalonFX(elevLeadId);
            elevFollow = new TalonFX(elevFollowId);

        }

        TalonFXConfiguration cfg = new TalonFXConfiguration();

        FeedbackConfigs fdb = cfg.Feedback;
        fdb.SensorToMechanismRatio = 12;

        // MotionMagicConfigs motionMagic = cfg.MotionMagic;
        // motionMagic.withMotionMagicCruiseVelocity(RotationsPerSecond.of(20))
        // .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(100));

        // if (Utils.isSimulation()) {
        // Slot0Configs slot0 = cfg.Slot0;
        // slot0.kS = 0;
        // slot0.kV = 0.15;
        // slot0.kA = 0.15;
        // slot0.kG = 0.35429;
        // slot0.kP = 0;
        // slot0.kI = 0;
        // slot0.kD = 0;

        // }

        StatusCode leaderStatus = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            leaderStatus = elevLead.getConfigurator().apply(cfg);
            if (leaderStatus.isOK())
                break;
        }
        if (!leaderStatus.isOK()) {
            System.out.println("Could not configure Elevator Motor. Error: " +
                    leaderStatus.toString());
        }

        StatusCode followerStatus = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            followerStatus = elevFollow.getConfigurator().apply(cfg);
            if (followerStatus.isOK())
                break;
        }

        elevFollow.setControl(new Follower(elevLead.getDeviceID(), false));

    }

    public Command setHeight(double newHeight) {
        return runOnce(() -> {
            height = newHeight;
        });
    }

    public double getEffort() {
        return feedFoward.calculate(0) +
                pidController.calculate(elevatorSim.getPositionMeters(), height);
    }

    public Command setVoltage(double voltage) {
        return runOnce(() -> {
            elevLead.setVoltage(voltage);

        });
    }

    @Override
    public void periodic() {
        if (Utils.isSimulation()) {
            var change = (height) - (simEncoder);
            
            elevLead.setVoltage(getEffort());
            elevatorSim.setInput(elevLead.getMotorVoltage().getValueAsDouble());
            elevatorSim.update(0.001);
            elevatorViz.setLength(elevatorSim.getPositionMeters());


    
            // elevLead.setControl(motionMagicVoltage.withPosition(change).withSlot(0));
            SmartDashboard.putNumber("height", height);
            SmartDashboard.putNumber("heightEncoder", elevatorEncoder.get());
            SmartDashboard.putNumber("Elevator Encoder", simEncoder);
            SmartDashboard.putNumber("Elevator set voltage", elevLead.getMotorVoltage().getValueAsDouble());
            SmartDashboard.putNumber("Elevator follower set voltage",
                    elevFollow.getMotorVoltage().getValueAsDouble());
            SmartDashboard.putNumber("Elevator Height", height);
            SmartDashboard.putNumber("sim elev position", elevatorSim.getPositionMeters());
            SmartDashboard.putNumber("sim elev motor position", elevLead.getPosition().getValueAsDouble());

        }
    }

}
