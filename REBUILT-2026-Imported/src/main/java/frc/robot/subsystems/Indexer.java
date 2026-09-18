package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;

import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import frc.robot.utils.Configs.IndexerConfigs;
import frc.robot.utils.Constants.IndexerConstants;
import org.wpilib.command2.StartEndCommand;

public class Indexer extends SubsystemBase{

    private TalonFX m_roller;

    public Indexer(){
        m_roller = new TalonFX(IndexerConstants.kIndexerID, new CANBus(IndexerConstants.kCanbusName));
        m_roller.getConfigurator().apply(IndexerConfigs.rollerConfigs);
    }

    public Command runIndexerCommand(double speed) {
        return this.run(() -> m_roller.setThrottle(speed));
    }

    public Command stopIndexerCommand(){
        return this.run(() -> m_roller.setThrottle(0));
    }

    public Command runIndexer(double speed){
        return new StartEndCommand(
            () -> runIndexerCommand(speed),
            () -> stopIndexerCommand(),
            this
        );
    }


}