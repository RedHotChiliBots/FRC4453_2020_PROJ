package frc.robot;

import java.util.function.Supplier;

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
        builder.addDoubleProperty("P", () -> {
            SlotConfiguration c = new SlotConfiguration();
            talon.getSlotConfigs(c);
            return c.kP;
        }, (double p) -> {
            talon.config_kP(0, p);
        });

        builder.addDoubleProperty("I", () -> {
            SlotConfiguration c = new SlotConfiguration();
            talon.getSlotConfigs(c);
            return c.kI;
        }, (double p) -> {
            talon.config_kI(0, p);
        });

        builder.addDoubleProperty("D", () -> {
            SlotConfiguration c = new SlotConfiguration();
            talon.getSlotConfigs(c);
            return c.kD;
        }, (double p) -> {
            talon.config_kD(0, p);
        });

        builder.addDoubleProperty("F", () -> {
            SlotConfiguration c = new SlotConfiguration();
            talon.getSlotConfigs(c);
            return c.kF;
        }, (double p) -> {
            talon.config_kF(0, p);
        });

        builder.addDoubleProperty("position", () -> talon.getSelectedSensorPosition(), (double p) -> {});
    }

	
}
