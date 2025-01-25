package org.firstinspires.ftc.teamcode.commands.armcommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ArmCorrected;

public class ManualArmCommand extends CommandBase {
    private final ArmCorrected arm;
    private final GamepadEx manipulator;
    public double deadzone = 0.1;

    public ManualArmCommand(ArmCorrected arm, GamepadEx manipulator) {
        addRequirements(arm);

        this.arm = arm;
        this.manipulator = manipulator;
    }

    public boolean isManualActive() {
        return manipulator.getButton(GamepadKeys.Button.START);
    }

    @Override
    public void execute() {
        arm.manual(-manipulator.getRightY());
    }
}
