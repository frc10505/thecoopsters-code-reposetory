
package frc.team10505.robot.subsystems;

import org.w3c.dom.Text;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.TalonFX;
import com.fasterxml.jackson.databind.cfg.ConstructorDetector.SingleArgConstructor;
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
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeSubsystem extends SubsystemBase {

    private final int kAlgaeIntakeMotorID = 7;
    private final int kAlgaePivotMotorID = 8;

    private final double startingAngle = 90.0;

    /* Intake Speeds */
    private double intakeSpeed = 15;
    private double intakeSpeedSlow = 10;
    private double intakeStop = 0;
    private double simMotorSpeed = 30;
    private double simSpeed = 0;

    /* Pivot Stuff */
    private double pivotSetPoint = 0.0;
    private double absoluteOffset = 180.0;
    private double encoderValue;
    private double simEncoder = startingAngle;

    private final static int kPivotMotorCurrentLimit = 15;
    private final static int kIntakeMotorCurrentLimit = 25;
    private final static int pivotEncoderScale = 360;
    private final static int pivotEncoderOffset = 0;

    /* Motor Ids */
    private SparkMax pivotMotor = new SparkMax(kAlgaePivotMotorID, MotorType.kBrushless);
    private SparkAbsoluteEncoder pivotEncoder = pivotMotor.getAbsoluteEncoder();
    private SparkMax intakeMotor = new SparkMax(kAlgaeIntakeMotorID, MotorType.kBrushless);
    private SparkMaxConfig pivotMotorConfig = new SparkMaxConfig();
    private SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();

    /* PID FeedFoward */
    private PIDController pivotController;// = new PIDController(0, 0, 0);
    private ArmFeedforward pivotFeedForward;// = new ArmFeedforward(0, 0, 0, 0);
    // private PIDController simPivotController = new PIDController(0, 0, 0);
    // private ArmFeedforward simPivotFeedForward = new ArmFeedforward(0, 0, 0, 0);

    /* Pivot Sim */
    private final Mechanism2d pivotMech = new Mechanism2d(2, 2.5);
    private MechanismRoot2d pivotRoot = pivotMech.getRoot("pivotRoot", 1, 1.25);
    private MechanismLigament2d pivotViz = pivotRoot.append(new MechanismLigament2d("pivotViz", 0.75, startingAngle));
    private SingleJointedArmSim pivotSim = new SingleJointedArmSim(DCMotor.getNEO(1), 80,
            SingleJointedArmSim.estimateMOI(0.305, 2), 0.305, Units.degreesToRadians(-90), Units.degreesToRadians(90),
            true, Units.degreesToRadians(startingAngle));

    /* Intake Sim */
    private final Mechanism2d intakeMech = new Mechanism2d(2, 2.5);
    private MechanismRoot2d intakeRoot = intakeMech.getRoot("intakeRoot", 1, 1.25);
    private MechanismLigament2d intakeViz1 = intakeRoot
            .append(new MechanismLigament2d("intakeViz1", 0.4, startingAngle));
    private MechanismLigament2d intakeViz2 = intakeRoot
            .append(new MechanismLigament2d("intakeViz2", -0.4, startingAngle));
    private FlywheelSim intakeSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.0000000001, 2),
            DCMotor.getNEO(1), 0);

    public AlgaeSubsystem() {
        SmartDashboard.putData("Pivot sim", pivotMech);
        SmartDashboard.putData("Algae intake sim ", intakeMech);

        if (Utils.isSimulation()) {
            pivotController = new PIDController(8.55, 0, 0.4); // Gooder?
            pivotFeedForward = new ArmFeedforward(0, 0.17227, 0.1, 0.1); // Gooder?

        } else {

            pivotController = new PIDController(0.105, 0, 0);
            pivotFeedForward = new ArmFeedforward(0.05, 0.08, 0.06, 0.06); // KV and KA 0.06 = gooder
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
        if (Utils.isSimulation()) {
            return pivotViz.getAngle();
        } else {
            return (-pivotEncoder.getPosition() + absoluteOffset);

        }
    }

    public double getEffort() {
        if (Utils.isSimulation()) {
            return pivotFeedForward.calculate(Units.degreesToRadians(getPivotEncoder()), 0)
             + pivotController.calculate(pivotViz.getAngle(), pivotSetPoint);

        } else {
            return pivotFeedForward.calculate(Units.degreesToRadians(getPivotEncoder()), 0)
                    + pivotController.calculate(getPivotEncoder(), pivotSetPoint);
        }

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

    public Command runIntake(double speed) {
        if (Utils.isSimulation()) {
            return runEnd(() -> {
                simSpeed = speed;
            }, () -> {
                simSpeed = 0;
            });
        } else {
            return runEnd(() -> {
                intakeMotor.set(speed);

            }, () -> {
                intakeMotor.set(0);

            });
        }
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

        if (Utils.isSimulation()) {
            simEncoder = pivotViz.getAngle();
            pivotSim.setInput(getEffort());
            pivotSim.update(0.001);
            pivotViz.setAngle(Units.radiansToDegrees(pivotSim.getAngleRads()));
            encoderValue = getPivotEncoder();
            intakeSim.update(0.001);
            intakeSim.setInput(simSpeed);
            intakeViz1.setAngle(intakeViz1.getAngle() + intakeSim.getAngularVelocityRPM() * 0.05);
            intakeViz2.setAngle(intakeViz2.getAngle() + intakeSim.getAngularVelocityRPM() * 0.05);

            SmartDashboard.putNumber("pivotEncoder", encoderValue);
            SmartDashboard.putNumber("Intake Motor Output", intakeMotor.getAppliedOutput());
            SmartDashboard.putNumber("Pivot Motor Output", pivotMotor.getAppliedOutput());

        } else {
            pivotMotor.setVoltage(getEffort());
            SmartDashboard.putNumber("PivotEncoder", getPivotEncoder());
            SmartDashboard.putNumber("Pivot Motor Output", pivotMotor.getAppliedOutput());
            SmartDashboard.putNumber(" pivot calculated effort", pivotMotor.getMotorTemperature());
        }
    }

}
