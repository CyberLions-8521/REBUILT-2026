package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Configs.IndexerConfigs;
import frc.robot.utils.Constants.CANBusConstants;
import frc.robot.utils.Constants.IndexerConstants;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

public class Indexer extends SubsystemBase{

    private TalonFX m_indexer;

    public Indexer(){
        m_indexer = new TalonFX(IndexerConstants.kIndexerID, CANBusConstants.subsystemCANBus);
        m_indexer.getConfigurator().apply(IndexerConfigs.kIndexConfig);
    }

    public Command runIndexerCommand(double speed) {
        return this.run(() -> m_indexer.set(speed));
    }

    public Command stopIndexerCommand(){
        return this.run(() -> m_indexer.set(0));
    }

    public Command runIndexer(double speed){
        return new StartEndCommand(
            () -> runIndexerCommand(speed),
            () -> stopIndexerCommand(),
            this
        );
    }


}