import sys
import os
project_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..'))
if project_path not in sys.path:
    sys.path.append(project_path)
from isaacsim.simulation_app import SimulationApp
import carb
import omni.kit.commands
from isaacsim.core.utils.stage import is_stage_loading
from isaacsim.core.prims import SingleRigidPrim #能控制姿态和刚体的实时物理状态（朝向、线速度、角速度）
from isaacsim.core.utils.extensions import enable_extension
from common.logger_loader import logger
from common.config_loader import config_loader
from common.x_registry import Registry
from tasks.TienKung3_Brainco2_tasks.TienKung3_Brainco2_task_base import TienKung3_Brainco2_Task_Base
# others
import numpy as np
from pxr import Sdf
# settings
np.set_printoptions(precision=4)
carb.settings.get_settings().set("persistent/app/omniverse/gamepadCameraControl", False)
logger.info("TienKung3_Brainco2_task_01 imports all deps")


@Registry.register("TienKung3_Brainco2_task_01")
class TienKung3_Brainco2_task_01(TienKung3_Brainco2_Task_Base):

    def __init__(
        self,
        simulation_app: SimulationApp,
        physics_dt=1.0 / 120,
        render_dt=1.0 / 120,
        stage_units_in_meters=1.00,
        environment_path=None,
        robot_init_position=None,
        robot_init_orientation=None,
        #修复构造参数不同
        episode_id=0,
        task_name="",
        condition="standard",
        baseline="",
        seed=0,
        ):
        logger.debug("TienKung3_Brainco2 runner in")
        enable_extension("isaacsim.asset.gen.conveyor.ui")
        super().__init__(simulation_app=simulation_app,
                         physics_dt=physics_dt,
                         render_dt=render_dt,
                         stage_units_in_meters=stage_units_in_meters,
                         environment_path=environment_path,
                         episode_id=episode_id,
                         task_name=task_name,
                         condition=condition,
                         baseline=baseline,
                         seed=seed,)



        # Set rendering
        # Smaller number reduces material resolution to avoid out-of-memory, max is 15
        carb.settings.get_settings().set("/rtx-transient/resourcemanager/maxMipCount", 15)
        # Enable DLSS
        carb.settings.get_settings().set("/rtx-transient/dlssg/enabled", True)
        # omni.kit.commands.execute("ModifyLayerMetadata",
        #                           layer_identifier=self.environment_path,
        #                           parent_layer_identifier=None,
        #                           meta_index=2,
        #                           value=120)
        # "Auto": 3, "Performance": 0, "Balanced": 1, "Quality": 2
        carb.settings.get_settings().set("/rtx/post/dlss/execMode", 0)
        logger.info("rendering mode all set")
        # Tiangong
        self.robot = Registry.create(
            "TienKung3_Brainco2",
            init_position=robot_init_position,
            init_orientation=robot_init_orientation,
            l_arm_home_pose=self.l_arm_home_pose,
            r_arm_home_pose=self.r_arm_home_pose,
        )
        logger.info('TienKung3_Brainco2 initialized.')
        while is_stage_loading():
            simulation_app.update()
        self.container_prim = config_loader.task_config["task"]["container_prim"]
        self.Object_prim = config_loader.task_config["task"]["Object_prim"] 

        #新增的开关的主体刚体路径
        self.Object_body_prim = config_loader.task_config["task"]["Object_body_prim"] 
        self.switch_rigid = SingleRigidPrim(
        prim_path=self.Object_body_prim,
        name="switch_body",
        reset_xform_properties=False,
        )

        #获取开关的初始位置和朝向
        self.init_switch_pose, self.init_switch_orien = (
        self.switch_rigid.get_world_pose()
        )

        self.ConveyorNode_prim = config_loader.task_config["task"]["ConveyorNode_prim"]
        self.Conveyor_vel = config_loader.task_config["task"]["Conveyor_vel"]
        self.init_states()
        logger.success('TienKung3_Brainco2 runner initialized')

    def init_play(self, step_num: int):
        self.switch_rigid.initialize() #初始化开关主体刚体

        self.switch_rigid.set_linear_velocity( #设置开关主体刚体的线速度为0
            np.zeros(3, dtype=np.float32)
        )
        self.switch_rigid.set_angular_velocity( #设置开关主体刚体的角速度为0
            np.zeros(3, dtype=np.float32)
        )

        super().init_play(step_num) #启动传送带

        omni.kit.commands.execute( 
            "ChangeProperty",
            prop_path=Sdf.Path(
                f"{self.ConveyorNode_prim}.inputs:velocity"
            ),
            value=self.Conveyor_vel,
            prev=None,
        )

    #添加统一复位函数
    def reset_switch(self):
        self.switch_rigid.set_world_pose( #恢复开关主体位置、朝向
            position=self.init_switch_pose,
            orientation=self.init_switch_orien,
        )
        #清除速度，防止残余速度影响物理模拟
        self.switch_rigid.set_linear_velocity(
            np.zeros(3, dtype=np.float32)
        )

        self.switch_rigid.set_angular_velocity(
            np.zeros(3, dtype=np.float32)
        )


    def check_success_callback(self, step_size) -> None:
        current_pose,_ = self.switch_rigid.get_world_pose()
        if current_pose[2] < 0.7:
            print("switch pressed down, reset to init pose")
            self.reset_switch()
            return
            
        if self.task_checker.check_relative_position(a_path=self.Object_prim,
                                                     b_path=self.container_prim,
                                                     relation="inside",
                                                     inside_tolerance=np.array([-0.01, -0.01, -0.002, -0.01, -0.01, 0.03])):
            self.task_success_flag = True
            print("task success")
            logger.info("Task Success!")
            # Write success status to file
            self.stop()
        # return self.task_success_flag
