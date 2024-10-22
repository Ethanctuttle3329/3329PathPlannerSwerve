package frc.robot.commands.drivebase;

import java.util.List;
import java.util.function.BooleanSupplier;
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

public class AbsoluteDriveAdv extends Command
{
    private final Swerve swerve;
    private final DoubleSupplier vX, vY;
    private final DoubleSupplier headingAdjust;
    private final BooleanSupplier lookAway, lookTowards, lookLeft, lookRight;
    private boolean resetHeading = false;
    
    public AbsoluteDriveAdv(Swerve swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier headingAdjust,
                            BooleanSupplier lookAway, BooleanSupplier lookTowards, BooleanSupplier lookLeft,
                            BooleanSupplier lookRight)
    {
        this.swerve = swerve;
        this.vX = vX;
        this.vY = vY;
        this.headingAdjust = headingAdjust;
        this.lookAway = lookAway;
        this.lookTowards = lookTowards;
        this.lookLeft = lookLeft;
        this.lookRight = lookRight;

        addRequirements(swerve);
    }

    @Override
    public void initialize()
    {
        resetHeading = true;
    }

    @Override
    public void execute()
    {
        double headingX = 0;
        double headingY = 0;
        if(lookAway.getAsBoolean())
        {
            headingY = -1;
        }
        if(lookRight.getAsBoolean())
        {
            headingX = 1;
        }
        if(lookLeft.getAsBoolean())
        {
            headingX = -1;
        }
        if(lookTowards.getAsBoolean())
        {
            headingY = 1;
        }
        if(resetHeading)
        {
            if(headingX == 0 && headingY == 0 && Math.abs(headingAdjust.getAsDouble()) == 0)
            {
                Rotation2d currentHeading = swerve.getHeading();
                headingX = currentHeading.getSin();
                headingY = currentHeading.getCos();
            }
            resetHeading = false;
        }
        ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(vX.getAsDouble(), vY.getAsDouble(), headingX, headingY);
        Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
        translation = SwerveMath.limitVelocity(translation, swerve.getFieldVelocity(), swerve.getPose(),
                                               Constants.LOOP_TIME, Constants.ROBOT_MASS, List.of(Constants.CHASSIS),
                                               swerve.getSwerveDriveConfiguration());
        SmartDashboard.putNumber("LimitedTranslation", translation.getX());
        SmartDashboard.putString("Translation", translation.toString());
        if(headingX == 0 && headingY == 0 && Math.abs(headingAdjust.getAsDouble()) > 0)
        {
            resetHeading = true;
            swerve.drive(translation, (Constants.OperatorConstants.TURN_CONSTANT * -headingAdjust.getAsDouble()), true);
        } else
        {
            swerve.drive(translation, desiredSpeeds.omegaRadiansPerSecond, true);
        }
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
