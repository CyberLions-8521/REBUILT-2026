
package frc.robot.subsystems;


import edu.wpi.first.wpilibj.motorcontrol.Spark;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.units.measure.Force;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.LimelightHelpers;
import frc.robot.utils.Configs.CANdleConfigs;
import frc.robot.utils.Constants.CANdleConstants;
import frc.robot.utils.Constants.LimelightConstants;
import frc.robot.subsystems.Shooter;

public class LEDLights extends SubsystemBase {
  private final Spark m_blinkinSpark = new Spark(0); 
  private final Shooter m_shooter;

  private boolean seesApirilTag;
  private boolean centeredApirilTag;
  private boolean inRange;

  public enum LEDmode {
    Rainbow(0.99),
    PartyRainbow(0.97),
    OceanRainbow(0.95),
    LavaRainbow(0.93),
    ForestRainbow(0.91),
    RainbowwGlitter(0.89),
    Confetti(0.87),
    ShotRed(0.85),
    ShotBlue(0.83),
    ShotWhite(0.81),
    SinelonRainbow(0.79),
    SinelonParty(0.77),
    SinelonOcean(0.75),
    SinelonLava(0.73),
    SinelonForest(0.71),
    BPMRainbow(0.69),
    BPMParty(0.67),
    BPMOcean(0.65),
    BPMLava(0.63),
    BPMForest(0.61),
    FireMedium(0.59),
    FireLarge(0.57),
    TwinklesRainbow(0.55),
    TwinklesParty(0.53),
    TwinklesOcean(0.51),
    TwinklesLava(0.49),
    TwinklesForest(0.47),
    ColorWavesRainbow(0.45),
    ColorWavesParty(0.43),
    ColorWavesOcean(0.41),
    ColorWavesLava(0.39),
    ColorWavesForest(0.37),
    LarsonScannerRed(0.35),
    LarsonScannerGray(0.33),
    LightChaseRed(0.31),
    LightChaseBlue(0.29),
    LightChaseGray(0.27),
    HeartbeatRed(0.25),
    HeartbeatBlue(0.23),
    HeartbeatWhite(0.21),
    HeartbeatGray(0.19),
    BreathRed(0.17),
    BreathBlue(0.15),
    BreathGray(0.13),
    StrobeRed(0.11),
    StrobeBlue(0.09),
    StrobeGold(0.07),
    StrobeWhite(0.05),
    Color1BlendtoBlack(0.03),
    Color1Larson(0.01),
    Color1Chase(0.01),
    Color1HeartbeatSlow(0.03),
    Color1HeartbeatMed(0.05),
    Color1HeartbeatFast(0.07),
    Color1BreathSlow(0.09),
    Color1BreathFast(0.11),
    Color1Shot(0.13),
    Color1Strobe(0.15),
    Color2BlendtoBlack(0.17),
    Color2Larson(0.19),
    Color2Chase(0.21),
    Color2HeartbeatSlow(0.23),
    Color2HeartbeatMed(0.25),
    Color2HeartbeatFast(0.27),
    Color2BreathSlow(0.29),
    Color2BreathFast(0.31),
    Color2Shot(0.33),
    Color2Strobe(0.35),
    SparkleC1onC2(0.37),
    SparkleC2onC1(0.39),
    GradientC1C2(0.41),
    BPMC1C2(0.43),
    BlendC1toC2(0.45),
    BlendC1C2(0.47),
    NoBlendC1C2(0.49),
    TwinklesC1C2(0.51),
    ColorWavesC1C2(0.53),
    SinelonC1C2(0.55),
    HotPink(0.57),
    DarkRed(0.59),
    Red(0.61),
    RedOrange(0.63),
    Orange(0.65),
    Gold(0.67),
    Yellow(0.69),
    LawnGreen(0.71),
    Lime(0.73),
    DarkGreen(0.75),
    Green(0.77),
    BlueGreen(0.79),
    Aqua(0.81),
    SkyBlue(0.83),
    DarkBlue(0.85),
    Blue(0.87),
    BlueViolet(0.89),
    Violet(0.91),
    White(0.93),
    Gray(0.95),
    DarkGray(0.97),
    Black(0.99),
    Off(0.0);
    
    public final double value;

    private LEDmode (double value) {
      this.value = value;
    }
  }



  public LEDLights(Shooter i_shooter) {
    this.m_shooter = i_shooter;
  }

  public void setLEDcommand(double pattern) {
    m_blinkinSpark.set(pattern);
  }

  public void Off() {
    m_blinkinSpark.set(LEDmode.Off.value);
  }

  public Command CheckLimeLights() {
    return new RunCommand(() ->
    {
      seesApirilTag = LimelightHelpers.getTV(LimelightConstants.limelightName);
      centeredApirilTag = Math.abs(LimelightHelpers.getTX(LimelightConstants.limelightName)) < 1;
      inRange = m_shooter.getDistance() > 1.5;

      if (seesApirilTag && centeredApirilTag && inRange) {
        setLEDcommand(LEDmode.Green.value);
      }
      else if (seesApirilTag && inRange) {
        setLEDcommand(LEDmode.Orange.value);
      }
      else if (seesApirilTag) {
          setLEDcommand(LEDmode.Red.value);
      } else {
        Off();
      }
    }, this);
  }
  @Override
  public void periodic() {
    
  }
}
