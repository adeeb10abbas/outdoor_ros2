## This is the code that's originally from TRI and is used in Russ Tedrake's coursework at MIT. 
## He has graciously given us permission to use this code for the purposes of this project.

## Allows you to control the robot with a gamepad or sliders in Meshcat using Pydrake.

from functools import partial
import numpy as np
from pydrake.all import (AddMultibodyPlantSceneGraph, DiagramBuilder,
                         LeafSystem, DiscreteContactSolver, MeshcatVisualizer,
                         Parser, Simulator, StartMeshcat)

meshcat = StartMeshcat()

# From manipulation.meshcat_utils
class MeshcatSliders(LeafSystem):
    """
    A system that outputs the ``value``s from meshcat sliders.

    .. pydrake_system::

      name: MeshcatSliderSystem
      output_ports:
      - slider_group_0
      - ...
      - slider_group_{N-1}
    """

    def __init__(self, meshcat, slider_names):
        """
        An output port is created for each element in the list `slider_names`.
        Each element of `slider_names` must itself be an iterable collection
        (list, tuple, set, ...) of strings, with the names of sliders that have
        *already* been added to Meshcat via Meshcat.AddSlider().

        The same slider may be used in multiple ports.
        """
        LeafSystem.__init__(self)

        self._meshcat = meshcat
        self._sliders = slider_names
        for i, slider_iterable in enumerate(self._sliders):
            port = self.DeclareVectorOutputPort(
                f"slider_group_{i}", len(slider_iterable),
                partial(self.CalcOutput, port_index=i))
            port.disable_caching_by_default()

    def CalcOutput(self, context, output, port_index):
        for i, slider in enumerate(self._sliders[port_index]):
            output[i] = self._meshcat.GetSliderValue(slider)

class GamepadCommand(LeafSystem):
    def __init__(self, meshcat):
        LeafSystem.__init__(self)

        self._meshcat = meshcat
        port = self.DeclareVectorOutputPort("command", 3, self.CalcOutput)
        port.disable_caching_by_default()

    def CalcOutput(self, context, output):
        gamepad = self._meshcat.GetGamepad()
        assert gamepad.index != None

        # https://beej.us/blog/data/javascript-gamepad/
        def CreateStickDeadzone(x, y):
            stick = np.array([x, y])
            deadzone = 0.2
            m = np.linalg.norm(stick)
            if m < deadzone:
                return np.array([0, 0])
            over = (m - deadzone) / (1 - deadzone)
            return stick * over / m

        left = CreateStickDeadzone(gamepad.axes[0], gamepad.axes[1])
        right = CreateStickDeadzone(gamepad.axes[2], gamepad.axes[3])

        output[0] = -left[1]  # Left stick y => vx
        output[1] = -left[0]  # Left stick x => vy
        output[2] = right[0]  # Right stick x => wz

class MyController(LeafSystem):

    def __init__(self, plant, model_instance):
        LeafSystem.__init__(self)
        self._wheel_velocity_indices = np.array([
            plant.GetJointByName('front_left_wheel_joint', model_instance).velocity_start(),
            plant.GetJointByName('front_right_wheel_joint', model_instance).velocity_start(),
            plant.GetJointByName('rear_left_wheel_joint', model_instance).velocity_start(),
            plant.GetJointByName('rear_right_wheel_joint', model_instance).velocity_start(),
        ]) + plant.num_positions()

        # command is the [vx, vy, wz] components of V_WRobot_Robot.
        self.DeclareVectorInputPort("command", 3)
        self.DeclareVectorInputPort("state", plant.num_multibody_states())
        self.DeclareVectorOutputPort("motor_torque", 4, self.CalcTorques)

        # These should match the parameters used to create the URDF.
        wheel_radius = 0.045 + (0.015 / 2) # hub_radius + (roller_diameter / 2).
        wheelbase = 0.2
        track = 0.205
        self._wheel_velocity_kp = 0.1

        lx = track * 0.5
        ly = wheelbase * 0.5

        # From anzu/punito/control/mecanum_kinematics.cc
        self._vehicle_to_wheel_map = np.array([
            [1, -1,  (lx+ly)],
            [1,  1, -(lx+ly)],
            [1,  1,  (lx+ly)],
            [1, -1, -(lx+ly)],
        ]) / wheel_radius

    def CalcTorques(self, context, output):
        # From anzu/punito/sim/robot_master_controller.cc
        command = self.get_input_port(0).Eval(context)
        state = self.get_input_port(1).Eval(context)
        wheel_velocity = state[self._wheel_velocity_indices]

        desired_wheel_velocity = self._vehicle_to_wheel_map @ command
        torque = self._wheel_velocity_kp * (desired_wheel_velocity - wheel_velocity)

        output.SetFromVector(torque)

def teleop():
    builder = DiagramBuilder()

    time_step = 0.005
    plant, scenegraph = AddMultibodyPlantSceneGraph(builder,
                                                    time_step=time_step)
    #plant.set_discrete_contact_solver(DiscreteContactSolver.kSap)
    parser = Parser(plant)
    parser.package_map().AddPackageXml("planning/teleop/data/package.xml")
    parser.AddModels('planning/teleop/data/drake_obstacles.dmd.yaml')
    plant.set_discrete_contact_solver(DiscreteContactSolver.kSap)
    plant.Finalize()
    robot_instance = plant.GetModelInstanceByName("mecanum_base")

    controller = builder.AddSystem(MyController(plant, robot_instance))
    builder.Connect(plant.get_state_output_port(), controller.get_input_port(1))
    builder.Connect(controller.get_output_port(),
                    plant.get_actuation_input_port())

    meshcat.Delete()
    meshcat.DeleteAddedControls()
    meshcat.SetProperty("/Lights/PointLightNegativeX","visible",False)

    gamepad = meshcat.GetGamepad()
    if gamepad.index != None: # use gamepad teleop
        print("Gamepad found.  Using gamepad teleop.")
        gamepad_command = builder.AddSystem(GamepadCommand(meshcat))
        builder.Connect(gamepad_command.get_output_port(),
                        controller.get_input_port(0))

    else: # use keyboard teleop
        print("No gamepad found.  Using keyboard/slider teleop.")
        step = 0.02
        meshcat.AddSlider(name="xdot",
                          increment_keycode="ArrowUp",
                          decrement_keycode="ArrowDown",
                          min=-1,
                          max=1,
                          step=step,
                          value=0)
        meshcat.AddSlider(name="ydot",
                          increment_keycode="ArrowLeft",
                          decrement_keycode="ArrowRight",
                          min=-1,
                          max=1,
                          step=step,
                          value=0)
        meshcat.AddSlider(name="yawdot",
                          increment_keycode="KeyD",
                          decrement_keycode="KeyA",
                          min=-1,
                          max=1,
                          step=step,
                          value=0)
        sliders = builder.AddSystem(
            MeshcatSliders(meshcat, [["xdot", "ydot", "yawdot"]]))
        builder.Connect(sliders.get_output_port(), controller.get_input_port(0))

    # Add a meshcat visualizer.
    visualizer = MeshcatVisualizer.AddToBuilder(
        builder, scenegraph.get_query_output_port(), meshcat)

    diagram = builder.Build()
    simulator = Simulator(diagram)
    context = simulator.get_mutable_context()


    simulator.set_target_realtime_rate(1.0)

    # For debugging:
    # command is the [vx, vy, wz] components of V_WRobot_Robot.
    #command = [0, 0, 0.2]
    #controller.get_input_port(0).FixValue(controller.GetMyContextFromRoot(context), command)
    # simulator.AdvanceTo(5.0)
    # return

    meshcat.AddButton("Stop Simulation", "Escape")
    while meshcat.GetButtonClicks("Stop Simulation") < 1:
        simulator.AdvanceTo(simulator.get_context().get_time() + 2.0)
    meshcat.DeleteButton("Stop Simulation")


teleop()
