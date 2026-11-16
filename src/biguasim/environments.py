"""Module containing the environment interface for BiguaSim.
An environment contains all elements required to communicate with a world binary or BiguaSimCore
editor.

It specifies an environment, which contains a number of agents, and the interface for communicating
with the agents.
"""
import atexit
import copy
import os
import random
import subprocess
import sys
import torch

import numpy as np

from biguasim.command import (
    CommandCenter,
    SpawnAgentCommand,
    TeleportCameraCommand,
    RenderViewportCommand,
    RenderQualityCommand,
    CustomCommand,
    DebugDrawCommand,
    SpawnPropCommand,
    RemovePropCommand,
)

from biguasim.exceptions import BiguaSimException
from biguasim.biguasimclient import BiguaSimClient
from biguasim.agents import AgentDefinition, SensorDefinition, AgentFactory
from biguasim.weather import WeatherController

from biguasim.sensors import AcousticBeaconSensor
from biguasim.sensors import DynamicsSensor
from biguasim.sensors import OpticalModemSensor

from biguasim.util import gpu

from biguasim.dynamics.agents import ModelsFactory


class BiguaSimEnvironment:
    """Proxy for communicating with a BiguaSim world

    Instantiate this object using :meth:`biguasim.biguasim.make`.

    Args:
        agent_definitions (:obj:`list` of :class:`AgentDefinition`):
            Which agents are already in the environment

        binary_path (:obj:`str`, optional):
            The path to the binary to load the world from. Defaults to None.

        window_size ((:obj:`int`,:obj:`int`)):
            height, width of the window to open

        start_world (:obj:`bool`, optional):
            Whether to load a binary or not. Defaults to True.

        uuid (:obj:`str`):
            A unique identifier, used when running multiple instances of biguasim. Defaults to "".

        gl_version (:obj:`int`, optional):
            The version of OpenGL to use for Linux. Defaults to 4.

        verbose (:obj:`bool`):
            If engine log output should be printed to stdout

        pre_start_steps (:obj:`int`):
            Number of ticks to call after initializing the world, allows the level to load and settle.

        show_viewport (:obj:`bool`, optional):
            If the viewport should be shown (Linux only) Defaults to True.

        ticks_per_sec (:obj:`int`, optional):
            The number of frame ticks per unreal seconds. This will override whatever is
            in the configuration json. Defaults to 30.

        frames_per_sec (:obj:`int` or :obj:`bool`, optional):
            The max number of frames ticks per real seconds. This will override whatever is
            in the configuration json. If True, will match ticks_per_sec. If False, will not be
            turned on. If an integer, will set to that value. Defaults to true.

        copy_state (:obj:`bool`, optional):
            If the state should be copied or returned as a reference. Defaults to True.

        scenario (:obj:`dict`):
            The scenario that is to be loaded. See :ref:`scenario-files` for the schema.

    """

    def __init__(
        self,
        agent_definitions=None,
        binary_path=None,
        window_size=None,
        start_world=True,
        uuid="",
        gl_version=4,
        verbose=False,
        pre_start_steps=2,
        show_viewport=True,
        ticks_per_sec=None,
        frames_per_sec=None,
        copy_state=True,
        scenario=None,
    ):
        
        if agent_definitions is None:
            agent_definitions = []

        # Initialize variables
        if window_size is None:
            # Check if it has been configured in the scenario
            if scenario is not None and "window_height" in scenario:
                self._window_size = scenario["window_height"], scenario["window_width"]
            else:
                # Default resolution
                self._window_size = 720, 1280
        else:
            self._window_size = window_size

        # Use env size from scenario/world config
        if scenario is not None and "env_min" in scenario:
            self._env_min = scenario["env_min"]
            self._env_max = scenario["env_max"]
        # Default resolution
        else:
            self._env_min = [-10, -10, -10]
            self._env_max = [10, 10, 10]

        if scenario is not None and "octree_min" in scenario:
            self._octree_min = scenario["octree_min"]
            self._octree_max = scenario["octree_max"]
        else:
            # Default resolution
            self._octree_min = 0.02
            self._octree_max = 5

        if scenario is not None and "lcm_provider" not in scenario:
            scenario["lcm_provider"] = ""

        self._uuid = uuid
        self._pre_start_steps = pre_start_steps
        self._copy_state = copy_state
        self._scenario = scenario
        self._copy_scenario = copy.deepcopy(self._scenario)
        self._initial_agent_defs = agent_definitions
        self._spawned_agent_defs = []

        # Choose one that was passed in function
        if ticks_per_sec is not None:
            self._ticks_per_sec = ticks_per_sec
        # otherwise use one in scenario
        elif scenario is not None and "ticks_per_sec" in scenario:
            self._ticks_per_sec = scenario["ticks_per_sec"]
        # default to 30
        else:
            self._ticks_per_sec = 30

        # If one wasn't passed in, use one in scenario
        if frames_per_sec is None and scenario is not None and "frames_per_sec" in scenario:
            frames_per_sec = scenario["frames_per_sec"]
        # default to true
        else:
            frames_per_sec = True

        # parse frames_per_sec
        if frames_per_sec is True:
            self._frames_per_sec = self._ticks_per_sec
        elif frames_per_sec is False:
            self._frames_per_sec = 0
        else:
            self._frames_per_sec = frames_per_sec

        self._lcm = None
        self._num_ticks = 0
        self._dt = 1/self._ticks_per_sec

        # Start world based on OS
        if start_world:
            world_key = self._scenario["world"]
            if os.name == "posix":
                self.__linux_start_process__(
                    binary_path,
                    world_key,
                    gl_version,
                    verbose=verbose,
                    show_viewport=show_viewport,
                )
            elif os.name == "nt":
                self.__windows_start_process__(
                    binary_path, world_key, verbose=verbose, show_viewport=show_viewport
                )
            else:
                raise BiguaSimException("Unknown platform: " + os.name)


        # Initialize Client
        self._client = BiguaSimClient(self._uuid)
        self._command_center = CommandCenter(self._client)
        self._client.command_center = self._command_center
        self._reset_ptr = self._client.malloc("RESET", [1], np.bool_)
        self._reset_ptr[0] = False


        # Initialize environment controller
        self.weather = WeatherController(self.send_world_command)

        # Set up agents already in the world
        self.agents = dict()
        # Names of props spawned since the last level (re)load
        self._props = set()
        self._state_dict = dict()
        self._dynamics_dict = dict()
        self._agent = None

        # Set the default state function
        self.num_agents = len(self.agents)

        # Whether we need to wait for a sonar to load
        self.start_world = start_world
        self._has_sonar = False

        if self.num_agents == 1:
            self._default_state_fn = self._get_single_state
        else:
            self._default_state_fn = self._get_full_state

        self._client.acquire(self._timeout)


        if os.name == "posix" and show_viewport is False:
            self.should_render_viewport(False)

        # Flag indicates if the user has called .reset() before .tick() and .step()
        self._initial_reset = False
        self.reset()

    @property
    def _timeout(self):
        # Returns the timeout that should be processed
        # Turns off timeout when creating octrees might be made
        if self._has_sonar or not self.start_world:
            if os.name == "posix":
                return None
            elif os.name == "nt":
                import win32event

                return win32event.INFINITE

        else:
            return 60

    @property
    def action_space(self):
        """Gives the action space for the main agent.

        Returns:
            :class:`~biguasim.spaces.ActionSpace`: The action space for the main agent.
        """
        return self._agent.action_space
    
    @property
    def scenario(self):
        return self._copy_scenario


    def info(self):
        """Returns a string with specific information about the environment.
        This information includes which agents are in the environment and which sensors they have.

        Returns:
            :obj:`str`: Information in a string format.
        """
        result = list()
        result.append("Agents:\n")
        for agent_name in self.agents:
            agent = self.agents[agent_name]
            result.append("\tName: ")
            result.append(agent.name)
            result.append("\n\tType: ")
            result.append(type(agent).__name__)
            result.append("\n\t")
            result.append("Sensors:\n")
            for _, sensor in agent.sensors.items():
                result.append("\t\t")
                result.append(sensor.name)
                result.append("\n")
        return "".join(result)
  

    def _load_scenario(self):
        """Loads the scenario defined in self._scenario_key.

        Instantiates agents, sensors, and weather.

        If no scenario is defined, does nothing.
        """

        def has_valid_dynamics_sensor(agent: dict) -> bool:
            # The dynamics models need COM data with a quaternion; other options (e.g. ExportGeometry) are free.
            return any(
                sensor.get("sensor_type") == "DynamicsSensor"
                and sensor.get("configuration", {}).get("UseCOM") is True
                and sensor.get("configuration", {}).get("UseRPY") is False
                for sensor in agent.get("sensors", [])
            )
        

        if self._scenario is None:
            return

        agents = list()

        #split agents
        for i in range(len(self._scenario["agents"])):
            agent = self._scenario["agents"][i]
            if 'agent_name' in agent.keys():
                agent_name = agent['agent_name']
            else:
                agent_name = agent['agent_type']

            batch_size= agent['dynamics']['batch_size']
            device = torch.device('cuda:'+ str(gpu()) if torch.cuda.is_available() else 'cpu')
            model = ModelsFactory.build_model(agent['agent_type'])
            params = model._params.copy()
            for k in  params.keys():
                if k in agent['dynamics'].keys():
                    params[k] = agent['dynamics'][k]
            # Optional keys no model lists in its _params (VehicleModel reads them with a default)
            for k in ('water_level',):
                if k in agent['dynamics']:
                    params[k] = agent['dynamics'][k]

            self._dynamics_dict[agent['agent_name']] = model(batch_size= batch_size, device= device, 
                                                     control_abstraction=agent["control_abstraction"], 
                                                     params=params)
            
            self._dynamics_dict[agent['agent_name']].dt = self._dt

            agent['control_scheme'] = self._dynamics_dict[agent['agent_name']]._scheme

            if 'dynamics' in agent.keys():
                if 'batch_size' in agent['dynamics'].keys():
                    if agent['dynamics']['batch_size'] > 1:
                        replacement = []
                        for j in range(agent['dynamics']['batch_size']):
                            new_agent = agent.copy()
                            new_agent['agent_name'] = f"{agent_name}-id{j}"
                            new_agent['dynamics']['batch_size'] = 1
                            replacement.append(new_agent)
                            
                        agents.extend(replacement)
                    else:
                        agent['agent_name'] = f"{agent_name}-id0"
                        agents.append(agent)

                else:
                    raise BiguaSimException(
                        "Dynamics parameter for agent {} is missing required parameter "
                        "'batch_size'".format(agent["agent_name"])
                    )
            else:
                raise BiguaSimException(
                        "Parameter for agent {} is missing required parameter "
                        "'dynamics'".format(agent["agent_name"])
                    )


        self._scenario['agents'] = agents



        for agent in self._scenario["agents"]:
            sensors = []
            if not has_valid_dynamics_sensor(agent):
                raise BiguaSimException(
                        "Sensor for agent {} is missing required valid sensor "
                        "'DynamicsSensor'".format(agent["agent_name"])
                    )
            for sensor in agent["sensors"]:
                if "sensor_type" not in sensor:
                    raise BiguaSimException(
                        "Sensor for agent {} is missing required key "
                        "'sensor_type'".format(agent["agent_name"])
                    )

                # Default values for a sensor
                sensor_config = {
                    "location": [0, 0, 0],
                    "rotation": [0, 0, 0],
                    "socket": "",
                    "configuration": None,
                    "sensor_name": sensor["sensor_type"],
                    "existing": False,
                    "Hz": self._ticks_per_sec,
                    "lcm_channel": None,
                    "ros_publish": False,
                }
                # Overwrite the default values with what is defined in the scenario config
                sensor_config.update(sensor)

                # set up sensor rates
                if self._ticks_per_sec < sensor_config["Hz"]:
                    raise ValueError(
                        f"{sensor_config['sensor_name']} is sampled at {sensor_config['Hz']} which is greater than ticks_per_sec {self._ticks_per_sec}"
                    )

                # round sensor rate as needed
                tick_every = self._ticks_per_sec / sensor_config["Hz"]
                if int(tick_every) != tick_every:
                    print(
                        f"{sensor_config['sensor_name']} rate {sensor_config['Hz']} is not a factor of ticks_per_sec {self._ticks_per_sec}, rounding to {self._ticks_per_sec//int(tick_every)}"
                    )
                sensor_config["tick_every"] = int(tick_every)

                sensors.append(
                    SensorDefinition(
                        agent["agent_name"],
                        agent["agent_type"],
                        sensor_config["sensor_name"],
                        sensor_config["sensor_type"],
                        socket=sensor_config["socket"],
                        location=sensor_config["location"],
                        rotation=sensor_config["rotation"],
                        config=sensor_config["configuration"],
                        tick_every=sensor_config["tick_every"],
                        lcm_channel=sensor_config["lcm_channel"],
                    )
                )

        


                if sensor_config["sensor_type"] in SensorDefinition._sonar_sensors:
                    self._has_sonar = True

                # Import LCM if needed
                if sensor_config["lcm_channel"] is not None and self._lcm is None:
                    globals()["lcm"] = __import__("lcm")
                    self._lcm = lcm.LCM(self._scenario["lcm_provider"])
                
                #Update the scenario of the enviornment to match the sensor_config with defaults applied
                sensor.update(sensor_config)

            # Default values for an agent
            agent_config = {
                "location": [0, 0, 0],
                "rotation": [0, 0, 0],
                "agent_name": agent["agent_type"],
                "existing": False,
                "location_randomization": [0, 0, 0],
                "rotation_randomization": [0, 0, 0],
            }

            agent_config.update(agent)
            is_main_agent = False

            if "main_agent" in self._scenario:
                main_agent, _id = agent["agent_name"].split('-id')
                is_main_agent = (self._scenario["main_agent"] == main_agent) and int(_id) == 0

            agent_location = agent_config["location"]
            agent_rotation = agent_config["rotation"]

            # Randomize the agent start location
            dx = agent_config["location_randomization"][0]
            dy = agent_config["location_randomization"][1]
            dz = agent_config["location_randomization"][2]

            agent_location[0] += random.uniform(-dx, dx)
            agent_location[1] += random.uniform(-dy, dy)
            agent_location[2] += random.uniform(-dz, dz)

            # Randomize the agent rotation
            d_pitch = agent_config["rotation_randomization"][0]
            d_roll = agent_config["rotation_randomization"][1]
            d_yaw = agent_config["rotation_randomization"][2]

            agent_rotation[0] += random.uniform(-d_pitch, d_pitch)
            agent_rotation[1] += random.uniform(-d_roll, d_roll)
            agent_rotation[2] += random.uniform(-d_yaw, d_yaw)

            agent_def = AgentDefinition(
                agent_config["agent_name"],
                agent_config["agent_type"],
                starting_loc=agent_location,
                starting_rot=agent_rotation,
                sensors=sensors,
                existing=agent_config["existing"],
                is_main_agent=is_main_agent,
            )

            self.add_agent(agent_def, is_main_agent)
            self.agents[agent["agent_name"]].set_control_scheme(agent["control_scheme"])
            self.agents[agent["agent_name"]].set_control_abstraction(agent["control_abstraction"])
            self._spawned_agent_defs.append(agent_def)

        if "weather" in self._scenario:
            weather = self._scenario["weather"]
            if "hour" in weather:
                self.weather.set_day_time(weather["hour"])
            if "type" in weather:
                self.weather.set_weather(weather["type"])
            if "fog_density" in weather:
                self.weather.set_fog_density(weather["fog_density"])
            if "day_cycle_length" in weather:
                day_cycle_length = weather["day_cycle_length"]
                self.weather.start_day_cycle(day_cycle_length)

    def reset(self):
        """Resets the environment, and returns the state.
        If it is a single agent environment, it returns that state for that agent. Otherwise, it
        returns a dict from agent name to state.

        Returns:
         :obj:`tuple` or :obj:`dict`:
            Returns the same as `tick`.
        """
        # Reset level
        self._initial_reset = True
        self._reset_ptr[0] = True
        for agent in self.agents.values():
            agent.clear_action()

        # reset all sensors
        for agent_name, agent in self.agents.items():
            for sensor_name, sensor in agent.sensors.items():
                sensor.reset()

        self.tick(
            publish=False,tick_clock=False
        )  # Must tick once to send reset before sending spawning commands
        self.tick(
            publish=False,tick_clock=False
        )  # Bad fix to potential race condition. See issue BYU-PCCL/holodeck#224
        self.tick(publish=False,tick_clock=False)
        # Clear command queue
        if self._command_center.queue_size > 0:
            print(
                "Warning: Reset called before all commands could be sent. Discarding",
                self._command_center.queue_size,
                "commands.",
            )
        self._command_center.clear()

        # The level reload destroyed every prop
        self._props = set()

        # Load agents
        self._spawned_agent_defs = []
        self.agents = dict()
        self._state_dict = dict()
        for agent_def in self._initial_agent_defs:
            self.add_agent(agent_def, agent_def.is_main_agent)

        self._scenario = copy.deepcopy(self._copy_scenario)
        self._load_scenario()


        self.num_agents = len(self.agents)

        if self.num_agents == 1:
            self._default_state_fn = self._get_single_state
        else:
            self._default_state_fn = self._get_full_state


        for _ in range(self._pre_start_steps + 1):
            self.tick(publish=False, tick_clock=False) #Spawn vehicles problem

        self._apply_exported_geometry()

        return self._default_state_fn()

    def _apply_exported_geometry(self):
        """Hands the collision mesh of each agent's DynamicsSensor to its dynamics model.

        Only for agents whose DynamicsSensor has ExportGeometry. The engine writes the geometry on the first tick
        after the sensor is created, so this runs at the end of reset, once the agents have ticked.
        """
        for agent_name, model in self._dynamics_dict.items():
            clones = [self.agents[f"{agent_name}-id{j}"] for j in range(model.batch_size)]
            sensors = [next(s for s in clone.sensors.values() if isinstance(s, DynamicsSensor)) for clone in clones]
            if not all(sensor.config.get("ExportGeometry", False) for sensor in sensors):
                continue
            geometries = [sensor.geometry for sensor in sensors]
            missing = [clone.name for clone, geometry in zip(clones, geometries) if geometry is None]
            if missing:
                raise BiguaSimException(f"The engine has not written the DynamicsSensor geometry of {missing} yet")
            model.set_geometry(geometries)

    def act(self, agent_name, action):
        """Supplies an action to a particular agent, but doesn't tick the environment.
           Primary mode of interaction for multi-agent environments. After all agent commands are
           supplied, they can be applied with a call to `tick`.

        Args:
            agent_name (:obj:`str`): The name of the agent to supply an action for.
            action (:obj:`np.ndarray` or :obj:`list`): The action to apply to the agent. This
                action will be applied every time `tick` is called, until a new action is supplied
                with another call to act.
        """

        self.agents[agent_name].act(action)

    def step(self, cmds, action_repeat=1):
        if not self._initial_reset:
            raise BiguaSimException("You must call .reset() before .step()")
        
        for _ in range(action_repeat):
            state = self.tick()  
            
            agents_names = self._dynamics_dict.keys()
            for agent_name in agents_names:
                dynamics_model = self._dynamics_dict[agent_name]

                if len(agents_names) > 1:
                    if not isinstance(cmds, dict):
                        raise BiguaSimException(
                            "Multiple agent types detected; control command must be a dict "
                            "mapping agent names to List[List[int]]] or List[int]."
                        )
                    
                    if dynamics_model.batch_size > 1:
                        if not (isinstance(cmds[agent_name], list) and all(isinstance(el, list) for el in cmds[agent_name])):
                            raise BiguaSimException(
                            "A batch of multiple agents detected; control command must be a dict "
                            "mapping agent names to List[List[int]]]."
                        )
                        action = dynamics_model.step(state[agent_name]['DynamicsSensor'], cmds[agent_name])

                    else:
                        if not isinstance(cmds[agent_name], list) or \
                        (isinstance(cmds[agent_name], list) and all(isinstance(el, list) for el in cmds[agent_name])):
                            raise BiguaSimException(
                            "Multiple, but single agents detected; control command must be a dict "
                            "mapping agent names to List[int]."
                        )

                        action = dynamics_model.step(state[agent_name]['DynamicsSensor'], [cmds[agent_name]])

                else:
                    if dynamics_model.batch_size > 1:
                        if not (isinstance(cmds, list) and all(isinstance(el, list) for el in cmds)):
                            raise BiguaSimException(
                            "A batch of agents detected; control command must be a List[List[int]]]."
                        )

                        action = dynamics_model.step(state['DynamicsSensor'], cmds)

                    else:
                        if not isinstance(cmds, list) or (isinstance(cmds, list) and all(isinstance(el, list) for el in cmds)):
                            raise BiguaSimException(
                            "Single agent detected; control command must be a List[int]."
                        )
                        action = dynamics_model.step([state['DynamicsSensor']], [cmds])

                for i in range(dynamics_model.batch_size):
                    self.agents[f"{agent_name}-id{i}"].act(action[i])
                    


        return state

    def tick(self, num_ticks=1, publish=True, tick_clock=True):
        """Ticks the environment once. Normally used for multi-agent environments.

        Args:
            num_ticks (:obj:`int`): Number of ticks to perform. Defaults to 1.
            publish (:obj:`bool`): Whether or not to publish as defined by scenario. Defaults to True.

        Returns:
            :obj:`dict`:
                A dictionary from agent name to its full state. The full state is another
                dictionary from :obj:`biguasim.sensors.Sensors` enum to np.ndarray, containing the
                sensors information for each sensor. The sensors always include the reward and
                terminal sensors. Reward and terminals can also be gotten through :meth:`get_reward_terminal`.

                Will return the state from the last tick executed.
        """
        if not self._initial_reset:
            raise BiguaSimException("You must call .reset() before .tick()")

        for _ in range(num_ticks):
            
            self._command_center.handle_buffer()

            self._client.release()
            self._client.acquire(self._timeout)

            state = self._default_state_fn()
            
            #Clock will not advance if the tick_clock is false (pre_start_steps in reset() so clock starts at 0)
            if tick_clock:
                self._tick_sensor()
                self._num_ticks += 1
            if publish and self._lcm is not None:
                self._publish(state)

        return state

    def _publish(self, state):
        """Publishes given state to channels chosen by the scenario config."""
        # if it was a partial state
        if self._agent.name not in state:
            state = {self._agent.name: state}

        # iterate through all agents and sensors
        for agent_name, agent in self.agents.items():
            for sensor_name, sensor in agent.sensors.items():
                # check if it's a full state, or single one
                if sensor.lcm_msg is not None and sensor_name in state[agent_name]:
                    # send message if it's in the dictionary and if LCM message is turned on
                    sensor.lcm_msg.set_value(
                        int(1000 * self._num_ticks / self._ticks_per_sec),
                        state[agent_name][sensor_name],
                    )
                    self._lcm.publish(
                        sensor.lcm_msg.channel, sensor.lcm_msg.msg.encode()
                    )

    def _enqueue_command(self, command_to_send):
        self._command_center.enqueue_command(command_to_send)


    def get_action_space(self, main_agent):
        return self.agents[f'{main_agent}-id0'].action_space

    def get_action_space_constraints(self, main_agent, control_abstraction):
        control_abstraction_id = self.agents[f'{main_agent}-id0'].abstractions_space[control_abstraction]
        return self.agents[f'{main_agent}-id0'].control_abstractions[control_abstraction_id]
    
    def get_dt(self):
        return self._dt


    def add_agent(self, agent_def, is_main_agent=False):
        """Add an agent in the world.

        It will be spawn when :meth:`tick` or :meth:`step` is called next.

        The agent won't be able to be used until the next frame.

        Args:
            agent_def (:class:`~biguasim.agents.AgentDefinition`): The definition of the agent to
            spawn.
        """
        if agent_def.name in self.agents:
            raise BiguaSimException("Error. Duplicate agent name. ")

        self.agents[agent_def.name] = AgentFactory.build_agent(self._client, agent_def)
        self._state_dict[agent_def.name] = self.agents[agent_def.name].agent_state_dict

        if not agent_def.existing:
            command_to_send = SpawnAgentCommand(
                location=agent_def.starting_loc,
                rotation=agent_def.starting_rot,
                name=agent_def.name,
                agent_type=agent_def.type.agent_type,
                is_main_agent=agent_def.is_main_agent,
            )

            self._client.command_center.enqueue_command(command_to_send)
        self.agents[agent_def.name].add_sensors(agent_def.sensors)
        if is_main_agent:
            self._agent = self.agents[agent_def.name]

    # Legacy ``spawn_prop`` names, kept so existing scripts keep working.
    _PROP_SHAPES = {
        "box": "/Engine/BasicShapes/Cube.Cube",
        "sphere": "/Engine/BasicShapes/Sphere.Sphere",
        "cylinder": "/Engine/BasicShapes/Cylinder.Cylinder",
        "cone": "/Engine/BasicShapes/Cone.Cone",
    }
    _PROP_MATERIALS = {
        "white": "/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial",
        "gold": "/Game/StarterContent/Materials/M_Metal_Gold.M_Metal_Gold",
        "cobblestone": "/Game/StarterContent/Materials/M_CobbleStone_Rough.M_CobbleStone_Rough",
        "brick": "/Game/StarterContent/Materials/M_Brick_Clay_Old.M_Brick_Clay_Old",
        "wood": "/Game/StarterContent/Materials/M_Wood_Pine.M_Wood_Pine",
        "grass": "/Game/StarterContent/Materials/M_Ground_Grass.M_Ground_Grass",
        "steel": "/Game/StarterContent/Materials/M_Metal_Steel.M_Metal_Steel",
        "black": "/Game/HolodeckContent/Agents/HoveringAUV/Materials/color_000000ff.color_000000ff",
    }

    def spawn_prop(
        self,
        prop_type,
        location=None,
        rotation=None,
        scale=1,
        sim_physics=False,
        material="",
        tag="",
        name=None,
    ):
        """Spawns any static mesh from the world's Unreal project as a prop.

        The prop appears on the next :meth:`tick` or :meth:`step`, and can be removed with
        :meth:`remove_prop`. Props do not persist after :meth:`reset`.

        The engine resolves meshes and materials by asset reference. Since commands are one-way,
        a reference it cannot resolve is reported only in the engine log, and no prop appears.

        Args:
            prop_type (:obj:`string`):
                The static mesh to spawn. One of:

                - a basic shape: ``box``, ``sphere``, ``cylinder`` or ``cone`` (1 meter across at scale 1);
                - an asset path, e.g. ``/Game/Spawnable/Buoy/SM_Buoy`` or ``/Game/Spawnable/Buoy/SM_Buoy.SM_Buoy``;
                - an editor "Copy Reference" string, e.g. ``/Script/Engine.StaticMesh'/Game/Spawnable/Buoy/SM_Buoy.SM_Buoy'``;
                - a bare asset name, e.g. ``SM_Buoy``, if exactly one static mesh has that name.

                In a packaged world only cooked meshes exist: those placed in the map, plus the
                folders listed under ``DirectoriesToAlwaysCook`` in the engine's ``DefaultGame.ini``
                (``/Game/Spawnable`` is reserved for meshes meant to be spawned).

            location (:obj:`list` of :obj:`float`):
                The ``[x, y, z]`` location of the prop in meters.

            rotation (:obj:`list` of :obj:`float`):
                The ``[roll, pitch, yaw]`` rotation of the prop in degrees.

            scale (:obj:`list` of :obj:`float`) or (:obj:`float`):
                The ``[x, y, z]`` scale factors of the mesh. A single value scales every dimension.

            sim_physics (:obj:`boolean`):
                Whether the object is mobile and is affected by gravity. Requires the mesh to have
                simple collision.

            material (:obj:`string`):
                Material applied to every slot of the mesh; empty keeps the mesh's own materials.
                Either one of ``white``, ``gold``, ``cobblestone``, ``brick``, ``wood``, ``grass``,
                ``steel`` or ``black``, or a material asset reference in any form accepted by
                ``prop_type``.

            tag (:obj:`string`):
                The tag to apply to the prop. Useful for task references. It is also read as the
                semantic segmentation label (e.g. ``Boats``); without a tag, the label comes from
                the mesh's folder.

            name (:obj:`string`):
                Unique name to refer to the prop in :meth:`remove_prop`. Generated if omitted.

        Returns:
            :obj:`str`: The name of the prop.
        """
        location = [0, 0, 0] if location is None else location
        rotation = [0, 0, 0] if rotation is None else rotation
        # if the given scale is an single value, then scale every dimension to that value
        if not isinstance(scale, (list, tuple)):
            scale = [scale, scale, scale]

        if name is None:
            index = len(self._props)
            while f"prop{index}" in self._props:
                index += 1
            name = f"prop{index}"
        elif name in self._props:
            raise BiguaSimException(f"A prop named '{name}' already exists.")

        mesh = self._PROP_SHAPES.get(prop_type.lower(), prop_type)
        material = self._PROP_MATERIALS.get(material.lower(), material)

        for value in (name, mesh, material, tag):
            # Commands are serialized to JSON by concatenation, without escaping.
            if '"' in value or "\\" in value:
                raise BiguaSimException(
                    f"Prop parameters cannot contain double quotes or backslashes: {value!r}"
                )
        if not mesh:
            raise BiguaSimException("prop_type cannot be empty.")

        self._enqueue_command(
            SpawnPropCommand(name, mesh, location, rotation, scale, sim_physics, material, tag)
        )
        self._props.add(name)
        return name

    def remove_prop(self, name):
        """Removes a prop spawned with :meth:`spawn_prop`.

        The prop disappears on the next :meth:`tick` or :meth:`step`.

        Args:
            name (:obj:`str`): The name returned by :meth:`spawn_prop`.
        """
        if name not in self._props:
            raise BiguaSimException(f"No prop named '{name}' exists.")
        self._enqueue_command(RemovePropCommand(name))
        self._props.remove(name)

    def draw_line(self, start, end, color=None, thickness=10.0, lifetime=1.0):
        """Draws a debug line in the world

        Args:
            start (:obj:`list` of :obj:`float`): The start ``[x, y, z]`` location in meters of the line.
                (see :ref:`coordinate-system`)
            end (:obj:`list` of :obj:`float`): The end ``[x, y, z]`` location in meters of the line
            color (:obj:`list``): ``[r, g, b]`` color value (from 0 to 255). Defaults to [255, 0, 0] (red).
            thickness (:obj:`float`): Thickness of the line. Defaults to 10.
            lifetime (:obj:`float`): Number of simulation seconds the object should persist. If 0, makes persistent. Defaults to 1.
        """
        color = [255, 0, 0] if color is None else color
        command_to_send = DebugDrawCommand(0, start, end, color, thickness, lifetime)
        self._enqueue_command(command_to_send)

    def draw_arrow(self, start, end, color=None, thickness=10.0, lifetime=1.0):
        """Draws a debug arrow in the world

        Args:
            start (:obj:`list` of :obj:`float`): The start ``[x, y, z]`` location in meters of the line.
                (see :ref:`coordinate-system`)
            end (:obj:`list` of :obj:`float`): The end ``[x, y, z]`` location in meters of the arrow
            color (:obj:`list`): ``[r, g, b]`` color value (from 0 to 255). Defaults to [255, 0, 0] (red).
            thickness (:obj:`float`): Thickness of the arrow. Defaults to 10.
            lifetime (:obj:`float`): Number of simulation seconds the object should persist. If 0, makes persistent. Defaults to 1.
        """
        color = [255, 0, 0] if color is None else color
        command_to_send = DebugDrawCommand(1, start, end, color, thickness, lifetime)
        self._enqueue_command(command_to_send)

    def draw_box(self, center, extent, color=None, thickness=10.0, lifetime=1.0):
        """Draws a debug box in the world

        Args:
            center (:obj:`list` of :obj:`float`): The start ``[x, y, z]`` location in meters of the box.
                (see :ref:`coordinate-system`)
            extent (:obj:`list` of :obj:`float`): The ``[x, y, z]`` extent of the box
            color (:obj:`list`): ``[r, g, b]`` color value (from 0 to 255). Defaults to [255, 0, 0] (red).
            thickness (:obj:`float`): Thickness of the lines. Defaults to 10.
            lifetime (:obj:`float`): Number of simulation seconds the object should persist. If 0, makes persistent. Defaults to 1.
        """
        color = [255, 0, 0] if color is None else color
        command_to_send = DebugDrawCommand(
            2, center, extent, color, thickness, lifetime
        )
        self._enqueue_command(command_to_send)

    def draw_point(self, loc, color=None, thickness=10.0, lifetime=1.0):
        """Draws a debug point in the world

        Args:
            loc (:obj:`list` of :obj:`float`): The ``[x, y, z]`` start of the box.
                (see :ref:`coordinate-system`)
            color (:obj:`list` of :obj:`float`): ``[r, g, b]`` color value (from 0 to 255). Defaults to [255, 0, 0] (red).
            thickness (:obj:`float`): Thickness of the point. Defaults to 10.
            lifetime (:obj:`float`): Number of simulation seconds the object should persist. If 0, makes persistent. Defaults to 1.
        """
        color = [255, 0, 0] if color is None else color
        command_to_send = DebugDrawCommand(
            3, loc, [0, 0, 0], color, thickness, lifetime
        )
        self._enqueue_command(command_to_send)

    def move_viewport(self, location, rotation):
        """Teleport the camera to the given location

        By the next tick, the camera's location and rotation will be updated

        Args:
            location (:obj:`list` of :obj:`float`): The ``[x, y, z]`` location to give the camera
                (see :ref:`coordinate-system`)
            rotation (:obj:`list` of :obj:`float`): The x-axis that the camera should look down. Other 2 axes are formed
                by a horizontal y-aixs, and then the corresponding z-axis.
                (see :ref:`rotations`)

        """
        # test_viewport_capture_after_teleport
        self._enqueue_command(TeleportCameraCommand(location, rotation))

    def move_agent(self, main_name, location, rotation, idxs=0):
        if isinstance(idxs, list):
            for idx in idxs:
                self.agents[f"{main_name}-id{idx}"].teleport(location[idx], rotation[idx])

        else:
            self.agents[f"{main_name}-id{idxs}"].teleport(location, rotation)


    def should_render_viewport(self, render_viewport):
        """Controls whether the viewport is rendered or not

        Args:
            render_viewport (:obj:`boolean`): If the viewport should be rendered
        """
        self._enqueue_command(RenderViewportCommand(render_viewport))

    def set_render_quality(self, render_quality):
        """Adjusts the rendering quality of BiguaSim.

        Args:
            render_quality (:obj:`int`): An integer between 0 = Low Quality and 3 = Epic quality.
        """
        self._enqueue_command(RenderQualityCommand(render_quality))

    def set_control_scheme(self, agent_name, control_scheme):
        """Set the control scheme for a specific agent.

        Args:
            agent_name (:obj:`str`): The name of the agent to set the control scheme for.
            control_scheme (:obj:`int`): A control scheme value
                (see :class:`~biguasim.agents.ControlSchemes`)
        """
        if agent_name not in self.agents:
            print("No such agent %s" % agent_name)
        else:
            self.agents[agent_name].set_control_scheme(control_scheme)

    def set_control_abstraction(self, main_name, control_abstraction):
        """Set the control abstraction for a specific agents.

        Args:
            agent_name (:obj:`str`): The name of the agent to set the control abstraction for.
            control_abstraction (:obj:`str`): A control abstraction value
        """
        if control_abstraction not in self._dynamics_dict:
            print("No such agent %s" % main_name)
        else:
            self._dynamics_dict[main_name].control_abstraction = control_abstraction
        

    def send_world_command(self, name, num_params=None, string_params=None):
        """Send a world command.

        A world command sends an abitrary command that may only exist in a specific world or
        package. It is given a name and any amount of string and number parameters that allow it to
        alter the state of the world.

        If a command is sent that does not exist in the world, the environment will exit.

        Args:
            name (:obj:`str`): The name of the command, ex "OpenDoor"
            num_params (obj:`list` of :obj:`int`): List of arbitrary number parameters
            string_params (obj:`list` of :obj:`string`): List of arbitrary string parameters
        """
        num_params = [] if num_params is None else num_params
        string_params = [] if string_params is None else string_params

        command_to_send = CustomCommand(name, num_params, string_params)
        self._enqueue_command(command_to_send)

    def __linux_start_process__(
        self, binary_path, task_key, gl_version, verbose, show_viewport=True
    ):
        import posix_ipc

        out_stream = sys.stdout if verbose else open(os.devnull, "w")
        loading_semaphore = posix_ipc.Semaphore(
            "/HOLODECK_LOADING_SEM" + self._uuid,
            os.O_CREAT | os.O_EXCL,
            initial_value=0,
        )
        arguments = [
            binary_path,
            task_key,
            "-HolodeckOn",
            "-LOG=HolodeckLog.txt",
            "-ForceRes",
            "-ResX=" + str(self._window_size[1]),
            "-ResY=" + str(self._window_size[0]),
            # An empty "--HolodeckUUID=" makes the engine (FParse::Value) take the next argument as the UUID, so
            # its shared memory names never match the client's; without the flag it uses "" as well.
            *(["--HolodeckUUID=" + self._uuid] if self._uuid else []),
            "-TicksPerSec=" + str(self._ticks_per_sec),
            "-FramesPerSec=" + str(self._frames_per_sec),
            "-EnvMinX=" + str(self._env_min[0]),
            "-EnvMinY=" + str(self._env_min[1]),
            "-EnvMinZ=" + str(self._env_min[2]),
            "-EnvMaxX=" + str(self._env_max[0]),
            "-EnvMaxY=" + str(self._env_max[1]),
            "-EnvMaxZ=" + str(self._env_max[2]),
            "-OctreeMin=" + str(self._octree_min),
            "-OctreeMax=" + str(self._octree_max),
            "-Trace=cpu,gpu,frame,log,bookmark",
            "-TraceHost=127.0.0.1"
        ]   

        if not show_viewport:
            arguments.append("-RenderOffScreen")

        self._world_process = subprocess.Popen(
            arguments, stdout=out_stream, stderr=out_stream
        )

        atexit.register(self.__on_exit__)

        try:
            loading_semaphore.acquire(30)
        except posix_ipc.BusyError:
            raise BiguaSimException(
                "Timed out waiting for binary to load. Ensure that biguasim "
                "is not being run with root priveleges."
            )
        loading_semaphore.unlink()

    def __windows_start_process__(
        self, binary_path, task_key, verbose, show_viewport=True
    ):
        import win32event

        out_stream = sys.stdout if verbose else open(os.devnull, "w")
        loading_semaphore = win32event.CreateSemaphore(
            None, 0, 1, "Global\\HOLODECK_LOADING_SEM" + self._uuid
        )

        arguments = [
            binary_path,
            task_key,
            "-HolodeckOn",
            "-LOG=HolodeckLog.txt",
            "-ForceRes",
            "-ResX=" + str(self._window_size[1]),
            "-ResY=" + str(self._window_size[0]),
            # An empty "--HolodeckUUID=" makes the engine (FParse::Value) take the next argument as the UUID, so
            # its shared memory names never match the client's; without the flag it uses "" as well.
            *(["--HolodeckUUID=" + self._uuid] if self._uuid else []),
            "-TicksPerSec=" + str(self._ticks_per_sec),
            "-FramesPerSec=" + str(self._frames_per_sec),
            "-EnvMinX=" + str(self._env_min[0]),
            "-EnvMinY=" + str(self._env_min[1]),
            "-EnvMinZ=" + str(self._env_min[2]),
            "-EnvMaxX=" + str(self._env_max[0]),
            "-EnvMaxY=" + str(self._env_max[1]),
            "-EnvMaxZ=" + str(self._env_max[2]),
            "-OctreeMin=" + str(self._octree_min),
            "-OctreeMax=" + str(self._octree_max),
        ]

        if not show_viewport:
            arguments.append("-RenderOffScreen")

        self._world_process = subprocess.Popen(
            arguments, stdout=out_stream, stderr=out_stream
        )

        atexit.register(self.__on_exit__)
        response = win32event.WaitForSingleObject(
            loading_semaphore, 100000
        )  # 100 second timeout
        if response == win32event.WAIT_TIMEOUT:
            raise BiguaSimException("Timed out waiting for binary to load")

    def __on_exit__(self):
        if hasattr(self, "_exited"):
            return

        if hasattr(self,"_client"):
            self._client.unlink()
        if hasattr(self, "_world_process"):
            self._world_process.kill()
            self._world_process.wait(10)

        self._exited = True

    # Context manager APIs, allows `with` statement to be used
    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        # TODO: Surpress exceptions?
        self.__on_exit__()

    def _tick_sensor(self):
        for agent_name, agent in self.agents.items():
            for sensor_name, sensor in agent.sensors.items():
                # if it's not time, tick the sensor
                if sensor.tick_count != sensor.tick_every:
                    sensor.tick_count += 1
                # otherwise remove, it and reset count
                else:
                    sensor.tick_count = 1

    def _get_single_state(self):
        if self._agent is not None:
            # rebuild state dictionary to drop/change data as needed
            if self._copy_state:
                st = dict()
                for sensor_name, sensor in self.agents[
                    self._agent.name
                ].sensors.items():
                    data = sensor.sensor_data
                    if isinstance(data, np.ndarray):
                        st[sensor_name] = np.copy(data)
                    elif data is not None:
                        st[sensor_name] = data

                st["t"] = self._num_ticks / self._ticks_per_sec

            else:
                st = self._state_dict[self._agent.name]

            return st
        

        return self._get_full_state()

    def _get_full_state(self):
        if self._copy_state:
            state = dict()
            for agent_name, agent in self.agents.items():
                main_agent = agent_name.split('-id')[0]
                if not main_agent in state:
                    state[main_agent] = dict()


                for sensor_name, sensor in agent.sensors.items():
                    if not sensor_name in state[main_agent]:
                        state[main_agent][sensor_name] = list()
                    data = sensor.sensor_data
                    if isinstance(data, np.ndarray):
                        state[main_agent][sensor_name].append(np.copy(data))
                    elif data is not None:
                        state[main_agent][sensor_name].append(data)


            state["t"] = self._num_ticks / self._ticks_per_sec

        else:
            state = self._state_dict

        agents = set(state) - {'t'}
        if len(agents) == 1:
            single_state = state[next(iter(agents))]
            single_state['t'] = state['t']
            return single_state

        return state


    def _create_copy(self, obj):
        if isinstance(obj, dict):  # Deep copy dictionary
            copy = dict()
            for k, v in obj.items():
                if isinstance(v, dict):
                    copy[k] = self._create_copy(v)
                else:
                    copy[k] = np.copy(v)
            return copy
        return None  # Not implemented for other types

    ######################### BIGUASIM CUSTOM #############################

    ######################## ACOUSTIC BEACON HELPERS ###########################

    def send_acoustic_message(self, id_from, id_to, msg_type, msg_data):
        """Send a message from one beacon to another.

        Args:
            id_from (:obj:`int`): The integer ID of the transmitting modem.
            id_to (:obj:`int`): The integer ID of the receiving modem.
            msg_type (:obj:`str`): The message type. See :class:`biguasim.sensors.AcousticBeaconSensor` for a list.
            msg_data : The message to be transmitted. Currently can be any python object.
        """
        AcousticBeaconSensor.instances[id_from].send_message(id_to, msg_type, msg_data)

    @property
    def beacons(self):
        """Gets all instances of AcousticBeaconSensor in the environment.

        Returns:
            (:obj:`list` of :obj:`AcousticBeaconSensor`): List of all AcousticBeaconSensor in environment
        """
        return AcousticBeaconSensor.instances

    @property
    def beacons_id(self):
        """Gets all ids of AcousticBeaconSensor in the environment.

        Returns:
            (:obj:`list` of :obj:`int`): List of all AcousticBeaconSensor ids in environment
        """
        return list(AcousticBeaconSensor.instances.keys())

    @property
    def beacons_status(self):
        """Gets all status of AcousticBeaconSensor in the environment.

        Returns:
            (:obj:`list` of :obj:`str`): List of all AcousticBeaconSensor status in environment
        """
        return [i.status for i in AcousticBeaconSensor.instances.values()]

    ####################### OPTICAL MODEM HELPERS ###############################

    def send_optical_message(self, id_from, id_to, msg_data):
        """Sends data between various instances of OpticalModemSensor

        Args:
            id_from (:obj:`int`): The integer ID of the transmitting modem.
            id_to (:obj:`int`): The integer ID of the receiving modem.
            msg_data : The message to be transmitted. Currently can be any python object.
        """

        return OpticalModemSensor.instances[id_from].send_message(id_to, msg_data)

    @property
    def modems(self):
        """Gets all instances of OpticalModemSensor in the environment.

        Returns:
            (:obj:`list` of :obj:`OpticalModemSensor`): List of all OpticalModemSensor in environment
        """
        return OpticalModemSensor.instances

    @property
    def modems_id(self):
        """Gets all ids of OpticalModemSensor in the environment.

        Returns:
            (:obj:`list` of :obj:`int`): List of all OpticalModemSensor ids in environment
        """
        return list(OpticalModemSensor.instances.keys())
