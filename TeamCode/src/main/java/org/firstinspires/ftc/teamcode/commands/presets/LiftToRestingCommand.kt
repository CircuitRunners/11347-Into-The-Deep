package org.firstinspires.ftc.teamcode.commands.presets

import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import org.firstinspires.ftc.teamcode.commands.liftcommands.ProfiledLiftCommand
import org.firstinspires.ftc.teamcode.subsystems.Slides
import org.firstinspires.ftc.teamcode.subsystems.Slides.SlidePositions
import org.firstinspires.ftc.teamcode.subsystems.ArmCorrected
class LiftToRestingCommand(lift: Slides, preset: Presets, arm: ArmCorrected) : ParallelCommandGroup() {

    enum class Presets {
        DOWN,
    }

    init {
        addCommands(
            ParallelCommandGroup(
                SequentialCommandGroup(
                    // Change this ms to change when the arm comes up
                    WaitCommand(800),
                    InstantCommand({
                        arm.toSpecimenPos();
                    }),
                ),
                InstantCommand({

                }),
                when (preset) {
                    Presets.DOWN ->
                        ProfiledLiftCommand(lift, SlidePositions.STAGE_0.position, false)

                }
            )
        )
        addRequirements(lift)
    }
}
