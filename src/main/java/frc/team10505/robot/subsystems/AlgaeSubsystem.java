package frc.team10505.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeSubsystem extends SubsystemBase {

    private final int kAlgaeIntakeMotorID = 0;
    private final int kAlgaePivotMotorID = 1;

    private final double startingAngle = 0.0;

    /* Intake Speeds */
    private double intakeSpeed = 15;
    private double intakeSpeedSlow = 10;
    private double intakeStop = 0;

    private double pivotSetPoint = 90.0;
    private double absoluteOffset = 180.0;
    private double encoderValue;
    private double simEncoder = startingAngle;

    private final static int kPivotMotorCurrentLimit = 8;
    private final static int kIntakeMotorCurrentLimit = 15;
    private final static int pivotEncoderScale = 360;
    private final static int pivotEncoderOffset = 0;

    /* Motor Ids */
    private SparkMax pivotMotor = new SparkMax(kAlgaePivotMotorID, MotorType.kBrushless);
    private SparkAbsoluteEncoder pivotEncoder = pivotMotor.getAbsoluteEncoder();
    private SparkMax intakeMotor = new SparkMax(kAlgaeIntakeMotorID, MotorType.kBrushless);
    private SparkMaxConfig pivotMotorConfig = new SparkMaxConfig();
    private SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();
    private SparkMaxConfig simPivotMotorConfig = new SparkMaxConfig();
    private SparkMaxConfig simIntakeMotorConfig = new SparkMaxConfig();

    /* PID FeedFoward */
    private PIDController pivotController = new PIDController(0, 0, 0);
    private ArmFeedforward pivotFeedForward = new ArmFeedforward(0, 0, 0, 0);
    private PIDController simPivotController = new PIDController(0, 0, 0);
    private ArmFeedforward simPivotFeedForward = new ArmFeedforward(0, 0, 0, 0);

    /* Fancy Sim stuff */
    private final Mechanism2d pivotMech = new Mechanism2d(2, 2.5);
    private MechanismRoot2d pivotRoot = pivotMech.getRoot("pivotRoot", 1, 1.25);
    private MechanismLigament2d pivotViz = pivotRoot.append(new MechanismLigament2d("pivotViz", 0.75, startingAngle));
    private SingleJointedArmSim pivotSim = new SingleJointedArmSim(DCMotor.getNEO(1), 80,
            SingleJointedArmSim.estimateMOI(0.305, 2), 0.305, Units.degreesToRadians(-110), Units.degreesToRadians(110),
            true, Units.degreesToRadians(startingAngle));

    public AlgaeSubsystem() {
        if (Utils.isSimulation()) {
            pivotController = new PIDController(0, 0, 0);
            pivotFeedForward = new ArmFeedforward(0, 0, 0, 0);

            pivotMotor = new SparkMax(kAlgaePivotMotorID, MotorType.kBrushless);
            intakeMotor = new SparkMax(kAlgaeIntakeMotorID, MotorType.kBrushless);
            pivotMotorConfig = new SparkMaxConfig();
            intakeMotorConfig = new SparkMaxConfig();

        } else {
            pivotMotor = new SparkMax(kAlgaePivotMotorID, MotorType.kBrushless);
            intakeMotor = new SparkMax(kAlgaeIntakeMotorID, MotorType.kBrushless);
            pivotController = new PIDController(0, 0, 0);
            pivotFeedForward = new ArmFeedforward(0, 0, 0, 0);

        }
        // Pivot motor config
        pivotMotorConfig.idleMode(IdleMode.kBrake);
        pivotMotorConfig.smartCurrentLimit(kPivotMotorCurrentLimit,
                kPivotMotorCurrentLimit);
        pivotMotorConfig.absoluteEncoder.positionConversionFactor(pivotEncoderScale); // Angle encoder scale
        pivotMotorConfig.absoluteEncoder.zeroOffset(pivotEncoderOffset); // Angle encoder offset
        pivotMotor.configure(pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Intake motor config
        intakeMotorConfig.idleMode(IdleMode.kBrake);
        intakeMotorConfig.smartCurrentLimit(kIntakeMotorCurrentLimit,
                kIntakeMotorCurrentLimit);
        intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /* Fancy PID/FeedFoward Stuff */

    public double getPivotEncoder() {
        return (-pivotEncoder.getPosition() + absoluteOffset);
    }

    public double GetEffort() {
        return pivotFeedForward.calculate(Units.degreesToRadians(getPivotEncoder()), 0)
                + pivotController.calculate(getPivotEncoder(), pivotSetPoint);
    }

    public double simGetEffort() {
        return simPivotFeedForward.calculate(Units.degreesToRadians(pivotViz.getAngle()), 0)
                + simPivotController.calculate(simEncoder, pivotSetPoint);
    }

    /* Pivot Commands to Referance */

    public Command setAngle(double angle) {
        return runOnce(() -> {
            pivotSetPoint = angle;
        });
    }

    public Command setVoltage(double voltage) {
        return run(() -> {
            pivotMotor.setVoltage(voltage);
        });
    }

    public Command stopPivot() {
        return run(() -> {
            pivotMotor.stopMotor();
        });
    }

    public Command pivotSpaz() {
        return run(() -> {
            pivotMotor.setVoltage(20);
            pivotMotor.setVoltage(0);
            pivotMotor.setVoltage(5);
            pivotMotor.setVoltage(10);
        });
    }

    /* Intake Commands to Referance */

    public Command intakeFoward() {
        return runOnce(() -> {
            intakeMotor.set(intakeSpeed);
        });
    }

    public Command intakeFowardSlow() {
        return runOnce(() -> {
            intakeMotor.set(intakeSpeedSlow);
        });
    }

    public Command intakeBackward() {
        return runOnce(() -> {
            intakeMotor.set(-intakeSpeed);
        });
    }

    public Command intakeBackwardSlow() {
        return runOnce(() -> {
            intakeMotor.set(-intakeSpeedSlow);
        });
    }

    public Command intakeStop() {
        return runOnce(() -> {
            intakeMotor.set(intakeStop);
        });
    }


    public Command intakeSpaz() {
        return run(() -> {
            intakeMotor.set(intakeSpeed);
            intakeMotor.set(-intakeSpeed);
            intakeMotor.set(Math.random() * 10);
            intakeMotor.set(-intakeSpeed);
        });
    }

    @Override
    public void periodic() {
        encoderValue = getPivotEncoder();
        SmartDashboard.putNumber("pivotEncoder", encoderValue);
        SmartDashboard.putNumber("Intake Motor Output", intakeMotor.getAppliedOutput());
        SmartDashboard.putNumber("Pivot Motor Output", pivotMotor.getAppliedOutput());
        if (Utils.isSimulation()) {
            simEncoder = pivotViz.getAngle();
            pivotSim.setInput(simGetEffort());
            pivotSim.update(0.01);
            pivotViz.setAngle(Units.radiansToDegrees(pivotSim.getAngleRads()));

        }
    }

}
