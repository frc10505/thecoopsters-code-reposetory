package frc.team10505.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

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
    public final int elevLeadId = 10;
    public final int elevFollowId = 11;
    private int ELEVATOR_GEARSTACK = 12;
    private double True = 0;

    /* Strange Configs */
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    /* Random Things */
    public Boolean usePID = true;
    private DutyCycleEncoder elevatorEncoder = new DutyCycleEncoder(1);
    private int ELEVATOR_MOTOR_CURRENT_LIMIT = 40;
    private double simEncoder = 0.0;
    private double height = 10;

    /* PID Fancy */

    public static double KP;
    public static double KI;
    public static double KD;

    /* Feed forward Fancy */

    public static double KS;
    public static double KG;
    public static double KV;
    public static double KA;

    /* Motor Stuff */
    private TalonFX elevLead = new TalonFX(elevLeadId);
    private TalonFX elevFollow = new TalonFX(elevFollowId);
    // private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(2);
    private PIDController pidController; // = new PIDController(KP, KI, KD);
    private ElevatorFeedforward feedFoRward;// = new ElevatorFeedforward(KS, KG, KV, KA);

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
            SmartDashboard.putData("Elevator Sim", elevatorMech);
            elevLead = new TalonFX(elevLeadId);
            elevFollow = new TalonFX(elevFollowId);
            // motionMagicVoltage = new MotionMagicVoltage(height);
            pidController = new PIDController(1, 0, 0);
            feedFoRward = new ElevatorFeedforward(0, 0.1795, 0.2, 0.2);

        } else {
            elevLead = new TalonFX(elevLeadId, "kingKan");
            elevFollow = new TalonFX(elevFollowId, "kingKan");
            pidController = new PIDController(0, 0, 0);
            feedFoRward = new ElevatorFeedforward(0, 0, 0, 0);

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
        // slot0.kV = 0;
        // slot0.kA = 0;
        // slot0.kG = 50;
        // slot0.kP = 0;
        // slot0.kI = 0;
        // slot0.kD = 0;

        // }
        var motorConfig = new MotorOutputConfigs();

        var limitConfigs = new CurrentLimitsConfigs();
        limitConfigs.StatorCurrentLimit = ELEVATOR_MOTOR_CURRENT_LIMIT;
        limitConfigs.StatorCurrentLimitEnable = true;

        motorConfig.NeutralMode = NeutralModeValue.Brake;
        elevLead.getConfigurator().apply(motorConfig);
        elevLead.getConfigurator().apply(limitConfigs);

        elevFollow.getConfigurator().apply(motorConfig);
        elevFollow.getConfigurator().apply(limitConfigs);
        StatusCode leaderStatus = StatusCode.StatusCodeNotInitialized;

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

        elevFollow.setControl(new Follower(elevLead.getDeviceID(), false));

    }

    private double getEncoder() {
        return (elevLead.getRotorPosition().getValueAsDouble() * (Math.PI * 1.751 * 2) / 12.0) * -1.0;

    }

    public Command setHeight(double newHeight) {
        return runOnce(() -> {
            height = newHeight;
        });
    }

    public double getEffort() {
        if (Utils.isSimulation()) {
            return feedFoRward.calculate(0) +
                    pidController.calculate(ELEVATOR_MOTOR_CURRENT_LIMIT, ELEVATOR_GEARSTACK);
        } else {
            return feedFoRward.calculate(0) +
                    pidController.calculate(getEncoder(), height);
        }

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
            simEncoder = elevatorViz.getLength();
            elevLead.setVoltage(getEffort());
            // elevatorSim.setInput(elevLead.getMotorVoltage().getValueAsDouble() * 2);
            elevatorSim.update(0.01);
            elevatorViz.setLength(elevatorSim.getPositionMeters());
            // elevatorSim.setInput(getEffort());

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

        } else {
            // elevLead.setVoltage(-getEffort());

        }
    }

}
