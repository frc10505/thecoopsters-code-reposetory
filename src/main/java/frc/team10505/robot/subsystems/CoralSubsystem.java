package frc.team10505.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralSubsystem extends SubsystemBase {

    /* Motor Ids */
    private final int leftIntakeMotorId = 20;
    private final int rightIntakeMotorId = 21;
    private SparkMaxConfig leftIntakeMotorConfig = new SparkMaxConfig();
    private SparkMaxConfig rightIntakeMotorConfig = new SparkMaxConfig();

    private final static int kLeftIntakeMotorCurrentLimit = 30;
    private final static int kRightIntakeMotorCurrentLimit = 30;

    /* Coral Intake speeds */
    private double intakeCoral = 20;
    private double outakeCoral = 40;
    private double simSpeed = 0;

    /* Motor Stuff */
    private final SparkMax leftIntakeMotor = new SparkMax(leftIntakeMotorId, MotorType.kBrushless);
    private final SparkMax rightIntakeMotor = new SparkMax(rightIntakeMotorId, MotorType.kBrushless);

    /* Sim Variables */
    private final Mechanism2d intakeMech = new Mechanism2d(4, 3);
    private MechanismRoot2d leftIntakeRoot = intakeMech.getRoot("leftIntakeRoot", 1, 1.5);
    private MechanismRoot2d rightIntakeRoot = intakeMech.getRoot("rightIntakeRoot", 3, 1.5);
    
    private MechanismLigament2d leftIntakeViz1 = leftIntakeRoot.append(new MechanismLigament2d("leftIntakeViz1", 0.5, 180));
    private MechanismLigament2d leftIntakeViz2 = leftIntakeRoot.append(new MechanismLigament2d("leftIntakeViz2", -0.5, 180));
    private MechanismLigament2d rightIntakeViz1 = rightIntakeRoot.append(new MechanismLigament2d("rightIntakeViz1", 0.5, 180));
    private MechanismLigament2d rightIntakeViz2 = rightIntakeRoot.append(new MechanismLigament2d("rightIntakeViz2", -0.5, 180));
    
    private FlywheelSim intakeSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.0000001, 15), DCMotor.getNEO(1), 0);
    
    
    /* Constructor */

    public CoralSubsystem() {
       

        if (Utils.isSimulation()) {
            leftIntakeMotorConfig = new SparkMaxConfig();
            rightIntakeMotorConfig = new SparkMaxConfig();
            SmartDashboard.putData("Coral Intake", intakeMech);
        } else {

        }
        leftIntakeMotorConfig.idleMode(IdleMode.kBrake);
        leftIntakeMotorConfig.smartCurrentLimit(kLeftIntakeMotorCurrentLimit,
                kLeftIntakeMotorCurrentLimit);
        leftIntakeMotor.configure(leftIntakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        rightIntakeMotorConfig.idleMode(IdleMode.kBrake);
        rightIntakeMotorConfig.smartCurrentLimit(kRightIntakeMotorCurrentLimit,
                kRightIntakeMotorCurrentLimit);
        rightIntakeMotor.configure(rightIntakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    }

public Command runIntake(double speed) {
        if (Utils.isSimulation()) {
            return runEnd(() -> {
                simSpeed = speed;
            }, () -> {
                simSpeed = 0;
            });
        } else {
            return runEnd(() -> {
                leftIntakeMotor.set(speed);
                rightIntakeMotor.set(-speed);
               

            }, () -> {
                leftIntakeMotor.set(0);
                rightIntakeMotor.set(0);
                

            });
        }
    }

    


    @Override
    public void periodic() {
        if (Utils.isSimulation()) {
            intakeSim.update(0.01);
            leftIntakeViz1.setAngle(leftIntakeViz1.getAngle() + intakeSim.getAngularVelocityRPM() * 0.05);
            leftIntakeViz2.setAngle(leftIntakeViz2.getAngle() + intakeSim.getAngularVelocityRPM() * 0.05);
            rightIntakeViz1.setAngle(rightIntakeViz1.getAngle() + intakeSim.getAngularVelocityRPM() * 0.05);
            rightIntakeViz2.setAngle(rightIntakeViz2.getAngle() + intakeSim.getAngularVelocityRPM() * 0.05);
            intakeSim.setInput(simSpeed);
            
        }
    }





}
