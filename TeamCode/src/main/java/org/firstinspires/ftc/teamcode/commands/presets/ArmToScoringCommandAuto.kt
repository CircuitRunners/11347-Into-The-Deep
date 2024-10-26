package org.firstinspires.ftc.teamcode.commands.presets

import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import org.firstinspires.ftc.teamcode.commands.armcommands.ProfiledArmCommand
import org.firstinspires.ftc.teamcode.subsystems.Arm
import org.firstinspires.ftc.teamcode.subsystems.Arm.ArmPositions
import org.firstinspires.ftc.teamcode.subsystems.Claw

class ArmToScoringCommandAuto(arm: Arm, claw: Claw, preset: Presets) : ParallelCommandGroup() {
    enum class Presets {
        AUTO
    }

    init {
        addCommands(
                ParallelCommandGroup(
                        SequentialCommandGroup(
                                WaitCommand(800),
                                InstantCommand({

                                }),
                        ),
                        InstantCommand({
                            claw.close()
                        }),
                        when (preset) {
                            Presets.AUTO ->
                                ProfiledArmCommand(arm, ArmPositions.AUTO.position, false)
                        }
                )
        )
        addRequirements(arm)
    }
}
