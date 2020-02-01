package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

public class TalonSRXSendable implements Sendable {
    TalonSRX talon;
    public TalonSRXSendable(TalonSRX t) {
        talon = t;
    }
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("PIDController");

        builder.setActuator(true);
        builder.setSafeState(() -> talon.set(ControlMode.Disabled, 0));

        builder.addDoubleProperty("p", () -> {
            SlotConfiguration c = new SlotConfiguration();
            talon.getSlotConfigs(c);
            return c.kP;
        }, (double p) -> {
            talon.config_kP(0, p);
        });

        builder.addDoubleProperty("i", () -> {
            SlotConfiguration c = new SlotConfiguration();
            talon.getSlotConfigs(c);
            return c.kI;
        }, (double p) -> {
            talon.config_kI(0, p);
        });

        builder.addDoubleProperty("d", () -> {
            SlotConfiguration c = new SlotConfiguration();
            talon.getSlotConfigs(c);
            return c.kD;
        }, (double p) -> {
            talon.config_kD(0, p);
        });
    }

	
}
