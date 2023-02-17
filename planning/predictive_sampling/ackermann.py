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
        port = self.DeclareVectorOutputPort("command", 2, self.CalcOutput)
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
            plant.GetJointByName('front_left_wheel', model_instance).velocity_start(),
            plant.GetJointByName('front_right_wheel', model_instance).velocity_start(),
            plant.GetJointByName('rear_left_wheel', model_instance).velocity_start(),
            plant.GetJointByName('rear_right_wheel', model_instance).velocity_start(),
        ]) + plant.num_positions()

        self._vehicle_steer_angle_indices = np.array([
            plant.GetJointByName('left_steering_hinge_wheel', model_instance).position_start(),
            plant.GetJointByName('right_steering_hinge_wheel', model_instance).position_start(),
        ]) + plant.num_positions()
        
        ## Vehicle parameters
        self._wheelbase = 0.2
        self._track = 0.14
        self._wheel_radius = 0.045
        self._wheel_length = 0.045
        ## 

        # command is the [desired_speed, desired_steering] components of V_WRobot_Robot.
        # state is the full state of the robot.
        self.DeclareVectorInputPort("command", 2) # 2 is the size of the input vector it
        # expects. This is the command that we will be sending to the robot.
        self.DeclareVectorInputPort("state", plant.num_multibody_states()) # This is the state of the robot.
        # This is the output of the controller. It is the torque that we will be applying to the robot.
        self.DeclareVectorOutputPort("motor_torque", 6, self.finalOutput)
    
    def AckermannTorques(self, command, wheel_velocity, _steerAngle):
        """
        This is the method that I wrote to implement Ackermann steering.
        """
        desired_speed = command[0]
        desired_angle = command[1]
        steer_angle_left, steer_angle_right = steer_angle[0], steer_angle[1]
        L = self._wheelbase
        R = 1 / np.abs(np.tan(desired_angle))
        v_l = desired_speed * (R - 0.5 * self._track * np.tan(steer_angle_left)) / R
        v_r = desired_speed * (R + 0.5 * self._track * np.tan(steer_angle_right)) / R
        tau_l = (v_l - wheel_velocity[0]) / self._wheel_radius
        tau_r = (v_r - wheel_velocity[1]) / self._wheel_radius
        return np.array([tau_l, tau_l, tau_r, tau_r])

    def AckermannSteeringAngles(self, command, _steerAngle):
        """
        This method implements Ackerman steering angles
        """
        desired_angle = command[1]
        steer_angle_left, steer_angle_right = _steerAngle[0], _steerAngle[1]
        L = self._wheelbase
        beta = np.arctan2(np.tan(desired_angle), 2.0)
        steer_angle_left_desired = np.arctan2(L * np.tan(beta) - 0.5 * self._track, L)
        steer_angle_right_desired = np.arctan2(L * np.tan(beta) + 0.5 * self._track, L)
        return np.array([steer_angle_left_desired, steer_angle_right_desired])

    def finalOutput(self, context, output):
        # From anzu/punito/sim/robot_master_controller.cc
        command = self.get_input_port(0).Eval(context)
        state = self.get_input_port(1).Eval(context)
        
        wheel_velocity = state[self._wheel_velocity_indices]
        steer_angle = state[self._vehicle_steer_angle_indices]
        
        ## Get the Ackermann steering angles
        _steerAngle = self.AckermannSteeringAngles(command, steer_angle)
        _torqeOutput = self.AckermannTorques(command, wheel_velocity, _steerAngle)
        # desired_wheel_velocity = self._vehicle_to_wheel_map @ command
        
        output.SetFromVector(np.concat(_torqeOutput, _finalOutput))

def teleop():
    builder = DiagramBuilder()

    time_step = 0.005
    plant, scenegraph = AddMultibodyPlantSceneGraph(builder,
                                                    time_step=time_step)
			    #plant.set_discrete_contact_solver(DiscreteContactSolver.kSap)
    parser = Parser(plant)
    parser.package_map().AddPackageXml("planning/predictive_sampling/data/package.xml")
    parser.AddModels('planning/predictive_sampling/data/drake_obstacles.dmd.yaml')
    plant.set_discrete_contact_solver(DiscreteContactSolver.kSap)
    plant.Finalize()
    robot_instance = plant.GetModelInstanceByName("base_link")
    print("Number of multibody States: ", plant.num_multibody_states())

    controller = builder.AddSystem(MyController(plant, robot_instance))

    builder.Connect(plant.get_state_output_port(), controller.get_input_port(1))

    builder.Connect(controller.get_output_port(0),
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
        meshcat.AddSlider(name="steer",
                          increment_keycode="KeyA",
                          decrement_keycode="KeyD",
                          min=-1,
                          max=1,
                          step=step,
                          value=0)

        meshcat.AddSlider(name="vel",
                            increment_keycode="KeyW",
                            decrement_keycode="KeyS",
                            min=-1,
                            max=1,
                            step=step,
                            value=0)
        sliders = builder.AddSystem(
            MeshcatSliders(meshcat, [["steer", "vel"]]))
        builder.Connect(sliders.get_output_port(), controller.get_input_port(0))

    # Add a meshcat visualizer.
    visualizer = MeshcatVisualizer.AddToBuilder(
        builder, scenegraph.get_query_output_port(), meshcat)

    diagram = builder.Build()
    simulator = Simulator(diagram)
    context = simulator.get_mutable_context()


    simulator.set_target_realtime_rate(5.0)

    # For debugging:
    # command is the [vx, vy, wz] components of V_WRobot_Robot.
    # simulator.AdvanceTo(500000000.0)
    # return

    meshcat.AddButton("Stop Simulation", "Escape")
    while meshcat.GetButtonClicks("Stop Simulation") < 1:
        simulator.AdvanceTo(simulator.get_context().get_time() + 2.0)
    meshcat.DeleteButton("Stop Simulation")


teleop()
