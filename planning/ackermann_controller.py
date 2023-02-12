from pydrake.all import (AddMultibodyPlantSceneGraph, DiagramBuilder,
                         LeafSystem, DiscreteContactSolver, MeshcatVisualizer,
                         Parser, Simulator, StartMeshcat)

class AckermannController(LeafSystem):
    def __init__(self, meshcat, ackermann):
        LeafSystem.__init__(self)

        self._meshcat = meshcat
        self._ackermann = ackermann

        self.DeclareVectorInputPort("state", 3)
        self.DeclareVectorOutputPort("command", 2, self.CalcOutput)

    def CalcOutput(self, context, output):
        state = self.EvalVectorInput(context, 0).get_value()
        gamepad = self._meshcat.GetGamepad()
        assert gamepad.index != None
        output[:] = self._ackermann.get_control(state, gamepad.axes)
    
    def DoCalcTimeDerivatives(self, context, derivatives):
        state = self.EvalVectorInput(context, 0).get_value()
        gamepad = self._meshcat.GetGamepad()
        assert gamepad.index != None
        derivatives[:] = self._ackermann.get_derivatives(state, gamepad.axes)
