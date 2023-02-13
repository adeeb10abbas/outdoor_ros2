``` mermaid
graph LR
DesiredVehicleSpeed -->|Inputs|AckermannController;
DesiredSteeringAngle-->|Inputs|AckermannController;
AckermannController-->|Outputs|_steering;

subgraph Steering
_steering-->LeftSteeringControllerCommand;
_steering-->RightSteeringControllerCommand;
end

AckermannController-->|Outputs|_throttle;
subgraph Throttle
_throttle-->LeftFrontWheelControllerCommand;
_throttle-->RightFrontWheelControllerCommand;
_throttle-->LeftRearWheelControllerCommand;
_throttle-->RightRearWheelControllerCommand;
end
```
