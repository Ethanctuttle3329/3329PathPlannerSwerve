package frc.robot.commands.drivebase;

import java.util.List;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve.Swerve;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

public class AbsoluteFieldDrive extends Command
{
    private final Swerve swerve;
    private final DoubleSupplier vX, vY, heading;

    public AbsoluteFieldDrive(Swerve swerve, DoubleSupplier vX, DoubleSupplier vY,
                              DoubleSupplier heading)
    {
        this.swerve = swerve;
        this.vX = vX;
        this.vY = vY;
        this.heading = heading;

        addRequirements(swerve);
    }

    @Override
    public void initialize()
    {
    }

    @Override
    public void execute()
    {
        ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(vX.getAsDouble(), vY.getAsDouble(),
                                                             new Rotation2d(heading.getAsDouble() * Math.PI));
        Translation2d translation =SwerveController.getTranslation2d(desiredSpeeds);
        translation = SwerveMath.limitVelocity(translation, swerve.getFieldVelocity(), swerve.getPose(),
                                                             Constants.LOOP_TIME, Constants.ROBOT_MASS, List.of(Constants.CHASSIS),
                                                             swerve.getSwerveDriveConfiguration());
        SmartDashboard.putNumber("LimitedTranslation", translation.getX());
        SmartDashboard.putString("Translation", translation.toString());
        swerve.drive(translation, desiredSpeeds.omegaRadiansPerSecond, true);
    }

    @Override
    public void end(boolean interrupted)
    {  
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
