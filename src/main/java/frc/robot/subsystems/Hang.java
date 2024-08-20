package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

import java.util.List;
import java.util.ArrayList;

public class Hang extends SubsystemBase{
    public static final List<Integer> deviceIds = new ArrayList<>(List.of(19,20));
    private static final double GEAR_RATIO = 21;
    private static final ArmFeedforward k_feedForward = new ArmFeedforward(0.03, 0.18, 2.99);
    private static final PIDController k_feedBack = new PIDController(7.0/Math.toRadians(26), 0, 0);
    private static final TrapezoidProfile k_profile = new TrapezoidProfile(new Constraints(180, 240));
    /* arm positions, in degrees */
    public static final double TIGHT_POSITION = 5 * 360,    // eg: 5 rounds
        LOOSE_POSITION = 0;

    private final List<TalonFX> hangFalcons = new ArrayList<>();
    private final List<StatusSignal<Double>> hangPositionRevolutionsList = new ArrayList<>(), supplyCurrentList = new ArrayList<>();
    private State currentState = new State(TIGHT_POSITION, 0);
    private double setPoint = TIGHT_POSITION;

    public Hang() {
        for (Integer deviceId : deviceIds) {
            TalonFX hangFalcon = new TalonFX(deviceId);
            StatusSignal<Double> hangPositionRevolutions = hangFalcon.getPosition();
            this.hangPositionRevolutionsList.add(hangPositionRevolutions);
            StatusSignal<Double> supplyCurrent = hangFalcon.getSupplyCurrent();
            this.supplyCurrentList.add(supplyCurrent);
            BaseStatusSignal.setUpdateFrequencyForAll(100, supplyCurrent, hangPositionRevolutions);
            final CurrentLimitsConfigs hangCurrentLimit = new CurrentLimitsConfigs();
            hangCurrentLimit.SupplyCurrentLimit = 40;
            hangCurrentLimit.SupplyCurrentLimitEnable = true;
            hangFalcon.getConfigurator().apply(hangCurrentLimit);
            hangFalcon.setNeutralMode(NeutralModeValue.Coast);
            hangFalcon.setInverted(true);
            hangFalcon.optimizeBusUtilization();
            setDefaultCommand(Commands.run(() -> runSetPointProfiled(LOOSE_POSITION), this));
            hangFalcons.add(hangFalcon);
        }
    }

    @Override
    public void periodic() {
        for (int i = 0; i < deviceIds.size(); i++) {
            BaseStatusSignal.refreshAll(hangPositionRevolutionsList.get(i), supplyCurrentList.get(i));
            SmartDashboard.putNumber("Hang/hang" + i + " angle measured (deg)", Math.toDegrees(getRotationsRad(i)));
            SmartDashboard.putNumber("Hang/hang" + i + " supply current (A)", supplyCurrentList.get(i).getValue());
        }
    }

    public double getRotationsRad(int index) {
        return Units.rotationsToRadians(
                hangPositionRevolutionsList.get(index).getValue() / GEAR_RATIO
        ) - TIGHT_POSITION;
    }

    public void runSetPointProfiled(double setPoint) {
        this.setPoint = setPoint;
        final State goalStateDeg = new State(setPoint, 0);
        this.currentState = k_profile.calculate(Robot.kDefaultPeriod, currentState, goalStateDeg);

        for (int i = 0; i < deviceIds.size(); i++) {
            if (inPosition(i)){ //if position is ok,
                continue;
            }
            final double feedForwardVoltage = k_feedForward.calculate(
                    getRotationsRad(i),
                    currentState.velocity
            );
            final double feedBackVoltage = k_feedBack.calculate(getRotationsRad(i), currentState.position);

            final VoltageOut vOut = new VoltageOut(feedForwardVoltage + feedBackVoltage).withEnableFOC(true);
            SmartDashboard.putNumber("Hang/hang" + i + ": FeedForwardVoltage", feedForwardVoltage);
            SmartDashboard.putNumber("Hang/hang" + i + ": FeedBackVoltage", feedBackVoltage);
            hangFalcons.get(i).setControl(vOut);
        }
    }

    public boolean inPosition(int index) {
        double tightRad = Math.toRadians(TIGHT_POSITION);
        double looseRad = Math.toRadians(LOOSE_POSITION);
        return Math.abs(getRotationsRad(index) - setPoint) > tightRad && setPoint + 0.01 >= tightRad || Math.abs(getRotationsRad(index) - setPoint) < looseRad && setPoint - 0.01 <= looseRad;
    }
}
