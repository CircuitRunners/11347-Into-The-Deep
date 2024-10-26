package org.firstinspires.ftc.teamcode.commands.armcommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.Arm;

public class ManualArmCommand extends CommandBase {
    private final Arm arm;
    private final GamepadEx manipulator;
    public double deadzone = 0.1;

    public ManualArmCommand(Arm arm, GamepadEx manipulator) {
        addRequirements(arm);

        this.arm = arm;
        this.manipulator = manipulator;
    }

    public boolean isManualActive() {
        return manipulator.getButton(GamepadKeys.Button.X);
    }

    @Override
    public void execute() {
        arm.setPower(-manipulator.getRightX());
    }
}
