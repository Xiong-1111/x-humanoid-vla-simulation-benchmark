from omni.kit.scripting import BehaviorScript
import omni.kit.commands
from pxr import Sdf
import math
from omni.isaac.core import SimulationContext
from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.asset.gen.conveyor.ui")

import threading
from pynput import keyboard

print("\033[32m=======Conveyor Control Script Loaded=======\033[0m")
print("\033[32m1. pre-enable isaacsim.asset.gen.conveyor.ui extension\033[0m")
print("\033[32m2. pause key: start conveyor\033[0m")
print("\033[32m3. scroll_lock key: stop conveyor\033[0m")
print("\033[32m============================================\033[0m")

Conveyor_Velocity = 0.1 # m/s

def find_prim_path_by_name(prim_name: str, root_path: str = "/") -> str:
    """Recursively find the full path by Prim name"""
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(root_path)
    
    def _find_prim(prim):
        if prim.GetName() == prim_name:
            return prim.GetPath().pathString
        for child in prim.GetChildren():
            result = _find_prim(child)
            if result:
                return result
        return None
    
    found_path = _find_prim(prim)
    if not found_path:
        raise RuntimeError(f"Prim '{prim_name}' not found under {root_path}")
    return found_path


ConveyorNode_path = find_prim_path_by_name("ConveyorNode")
print("ConveyorNode_path", ConveyorNode_path)


class TrashcanOverControl(BehaviorScript):
    def on_init(self):
        self.convetor_vel = 0.0
        self.last_vel = 0.0
        self.init_keyboard()

    def init_keyboard(self):

        def start_keyboard_listener():
            with keyboard.Listener(on_press=self.on_press) as listener:
                listener.join()

        listener_thread = threading.Thread(target=start_keyboard_listener,
                                           daemon=True)
        listener_thread.start()

    
    def on_press(self, key):
        try:
            # print('key Pressing', key)
            if key == keyboard.Key.pause:
                self.convetor_vel = Conveyor_Velocity
            if key == keyboard.Key.scroll_lock:
                self.convetor_vel = 0.0
        except AttributeError:
            ...

    def on_destroy(self):
        pass

    def on_play(self):
        self.simulation_context = SimulationContext(
            stage_units_in_meters=1.0,
            physics_dt=1.0/120,
            rendering_dt=1.0/120
        )


    def on_pause(self):
        pass

    def on_stop(self):
        self.convetor_vel = 0.0

    def on_update(self, current_time: float, delta_time: float):
        if delta_time <= 0:
            return
        self.conveyor_control() 
    
    def conveyor_control(self):
        if self.last_vel != self.convetor_vel:
            omni.kit.commands.execute("ChangeProperty",
                                    prop_path=Sdf.Path(f"{ConveyorNode_path}.inputs:velocity"),
                                    value=self.convetor_vel,
                                    prev=None)
            self.last_vel = self.convetor_vel
        
        


                
