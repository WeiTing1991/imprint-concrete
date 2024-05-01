import itertools
from compas.geometry.primitives import vector

import compas_rrc as rrc
from compas.geometry import Frame, Vector
from compas.utilities import geometric_key



from .partial_order_plan import Action, ProductionData


class PrintProcess(object):
    def __init__(self, namespace='/rob111'):
        self.namespace = namespace
        self.material_running = True

    def setup(self):
        actions = []

        actions.append(Action('SetTool', dict(tool_name='t_A032_Print_Tool_Basic')))
        actions.append(Action('SetWorkObject', dict(wobj_name='ob_a032_ppce3dp_r111')))
        actions.append(Action('PrintText', dict(text='Tool and work object updated.')))
        actions.append(Action('CustomInstruction', dict(name='r_A032_MatOn')))
        actions.append(Action('SetAcceleration', dict(acc=100, ramp = 100)))

        return actions

    def flush(self):
        return [Action('CustomInstruction', dict(name='r_A032_Material_Flush'))]

    def print_path(self, points, path_id, is_danger_zone):
        actions = []

        # Create print instructions
        for p_index, ppoint in enumerate(points):
            instruction = "PrintToRobtarget"
            orientation = [float(o) for o in ppoint.orientation[1:-1].split(',')]
            gantry_values = [float(o) for o in ppoint.gantry[1:-1].split(',')]
            frame = Frame.from_quaternion(orientation, list(ppoint.pt))

            # calibration for two robots
            frame.point += Vector(-30, 10, 0)

            # HACK: Only first and last for simplier visualization
            if p_index == 0 or p_index == len(points) - 1:
                action = Action(instruction, dict(
                    motion_type='L',
                    frame=frame, ext_axes=gantry_values, speed=ppoint.speed, zone=ppoint.zone, flowrate_concrete=ppoint.concrete,
                    flowrate_accelerator=ppoint.accelerator, feedback_level=2,
                    path_id=path_id, is_danger_zone=is_danger_zone
                ))
                actions.append(action)

            if ppoint.material_flow == 'off' and self.material_running == True:
                self.material_running = False
                actions.append(Action('CustomInstruction', dict(name='r_A032_MatOff', path_id=path_id)))
        
            elif ppoint.material_flow == 'on' and self.material_running == False:
                self.material_running = True
                actions.append(Action('CustomInstruction', dict(name='r_A032_MatOn', path_id=path_id)))

        return actions


class PickAndPlaceProcess(object):
    def __init__(self, namespace='/rob21'):
        self.namespace = namespace
        self.gantry = [ 33412.63, -2364.1, -2759.58 ] 

        # Define pick positions
        self.offset_safety = 350 # safety postion 
        self.offset_push = (5*3) # push based on two layer height
        self.offset_x = -60

        # speed
        speed_factor = 3
        self.speed_min = 200 *speed_factor
        self.speed_mid = 300 *speed_factor
        self.speed_max = 400 *speed_factor
        # safe postion 
        self.safe_pos_joints = [11.6, -31.14, 31.04, -2.98, -90.35, 0.88]

    def setup(self):
        actions = []

        #Reset signal
        actions.append(Action('SetDigital', dict(io_name = 'doUnitR21ValveA3', value = 0)))
        actions.append(Action('PrintText', dict(text='Gripper is off')))

        #Set Work Object (for now only one)
        actions.append(Action('SetWorkObject', dict(wobj_name='ob_a032_ppce3dp_r21')))

        # Set tool
        # HACK: chnage to real work tool...
        actions.append(Action('SetTool', dict(tool_name='t_A032_T1_RFLTip')))
        actions.append(Action('PrintText', dict(text='Tool and work object updated.')))

        return actions

    def pick(self, pick_point, xaxis, yaxis):
        actions = []

        pre_pick_position = Frame((pick_point[0] + self.offset_x, pick_point[1], pick_point[2] + self.offset_safety + 200), xaxis, yaxis)
        pick_position = Frame((pick_point[0] + self.offset_x, pick_point[1], pick_point[2] + 200), xaxis, yaxis)
        
        # open the IO 
        actions.append(Action('SetDigital', dict(io_name ='doUnitR21ValveA3', value=1)))

        # Move over pre_pick postion   
        actions.append(Action('MoveToRobtarget', dict(frame=pre_pick_position, ext_axes=self.gantry, speed=self.speed_mid, zone=rrc.Zone.Z100)))

        # wait to pick
        actions.append(Action('WaitTime', dict(time = 0.5,  feedback_level=rrc.FeedbackLevel.DONE)))

        # Move to pick postion
        actions.append(Action('MoveToRobtarget', dict(frame=pick_position, ext_axes=self.gantry, speed=self.speed_min, zone=rrc.Zone.FINE)))

        # Grab glass
        actions.append(Action('SetDigital', dict(io_name ='doUnitR21ValveA3', value=0)))
        actions.append(Action('WaitTime', dict(time = 1,  feedback_level=rrc.FeedbackLevel.DONE)))
        actions.append(Action('PrintText', dict(text='Grab glass.')))

        # Move over pick postion
        actions.append(Action('MoveToRobtarget', dict(frame=pre_pick_position, ext_axes=self.gantry, speed=self.speed_mid, zone=rrc.Zone.Z100, feedback_level=rrc.FeedbackLevel.DONE)))

        # add safe position for robot 
        actions.append(Action('MoveToJoints', dict(joints = self.safe_pos_joints, ext_axes=self.gantry, speed=self.speed_max, zone=rrc.Zone.Z100, feedback_level=rrc.FeedbackLevel.DONE)))
        
        # HACK: because I'm lazy, just keep the last pre-pick position
        # but this has to be removed when we implement the safe configuration outside danger zone
        # (see comment on `def place`)
        return actions

    def place(self, place_point, xaxis, yaxis):
        actions = []

        xaxis = [0,0,1]
        yaxis = [0,1,0]

        # Define place positions
        pre_place_position = Frame((place_point[0] +200, place_point[1], place_point[2] + self.offset_safety - self.offset_x), xaxis, yaxis)
        place_position = Frame((place_point[0] +200, place_point[1], place_point[2] + self.offset_push - self.offset_x), xaxis, yaxis)
        print(place_position)

        # Move over place postion
        actions.append(Action('MoveToRobtarget', dict(frame=pre_place_position, ext_axes=self.gantry, speed=self.speed_mid, zone=rrc.Zone.Z100)))

        # Move to place postion
        actions.append(Action('MoveToRobtarget', dict(frame=place_position, ext_axes=self.gantry,  speed=self.speed_min, zone=rrc.Zone.FINE)))

        # Leave glass
        actions.append(Action('WaitTime', dict(time = 1, feedback_level=rrc.FeedbackLevel.DONE)))

        # open the IO 
        actions.append(Action('SetDigital', dict(io_name ='doUnitR21ValveA3', value=1)))

        actions.append(Action('PrintText', dict(text='Place glass.')))

        actions.append(Action('WaitTime', dict(time = 0.5, feedback_level=rrc.FeedbackLevel.DONE)))

        # Move over place postion
        actions.append(Action('MoveToRobtarget', dict(frame=pre_place_position, ext_axes=self.gantry, speed=self.speed_mid, zone=rrc.Zone.Z100)))
        
        
        # Move to a safe position
        # TODO: this should be changed to a safe position right outside the danger zone using MoveToJoints
        actions.append(Action('MoveToJoints', dict(joints = self.safe_pos_joints, ext_axes=self.gantry, speed=self.speed_max, zone=rrc.Zone.Z100, feedback_level=rrc.FeedbackLevel.DONE)))
        actions.append(Action('SetDigital', dict(io_name ='doUnitR21ValveA3', value=0)))

        return actions



class MergedProcess(object):
    def prepare_production_data(self, slice, addons, placing_points, picking_points, xaxis, yaxis, dangerous_flags_by_path):
        precision = 5

        #dictionary with add-on center points as keys and tuple of picking,placing planes as values
        addons_map = {geometric_key(a.Origin, precision): dict(addon=a, pick_point=b, place_point=c) for a, b, c in zip(addons, picking_points, placing_points)}

        #set with all add-on center points
        addons_set = set(addons_map.keys())

        #list of all print points
        all_pts = itertools.chain.from_iterable([layer.print_points for layer in slice.layers])

        #group print points by path
        pt_by_path = itertools.groupby(all_pts, lambda pt: pt.path_id)
        pt_by_path = [(k, list(v)) for k, v in pt_by_path]

        #create a production data object that will store all the actions for both processes to then export to json
        production_data = ProductionData()

        #will generate actions
        print_process = PrintProcess()
        pnp_process = PickAndPlaceProcess()

        #add a root action
        root_action_id = production_data.append_action(print_process.namespace, Action('PrintText', dict(text='Starting process')))

        #append setup instructions
        last_setup_action_ids = [] #list containing last id for seting up robot r111 and last id for seting up robot r21
        for process in (print_process, pnp_process):
            for a_index, action in enumerate(process.setup()):
                last_action_id = production_data.append_action(process.namespace, action)
                # The first action of each process will depend on the root action instead
                if a_index == 0:
                    production_data.actions[last_action_id].dependency_ids = [root_action_id]
            last_setup_action_ids.append(last_action_id)

        last_action_id = production_data.append_action(print_process.namespace, Action('PrintText', dict(text='Setup completed')), dependency_ids=last_setup_action_ids)
        join_action_id = last_action_id
        last_pnp_action_id = None

        actions_by_path = {}
        #stupid_counter = 0
        #stupid_limit = 10

        # Add Print actions
        for idx, (path_id, pt_in_path) in enumerate(pt_by_path):
            #stupid_counter += 1
            #if stupid_counter > stupid_limit: break

            #print points per path
            pt_key_set = set([geometric_key(p.pt, precision) for p in pt_in_path])
            addons_in_path = addons_set.intersection(pt_key_set)
            actions_by_path[path_id] = dict(print_actions=[], pick_actions=[], place_actions=[])

            # TODO: Change this, currently done only for pretty output in the diagram
            #is_danger_zone = bounding_box.Contains(pt_in_path[0].pt)
            is_danger_zone = dangerous_flags_by_path[idx]

            # Append print actions for this path
            print_actions = print_process.print_path(pt_in_path, path_id, is_danger_zone)

            for a_index, action in enumerate(print_actions):
                last_action_id = production_data.append_action(print_process.namespace, action)
                actions_by_path[path_id]['print_actions'].append(last_action_id)
                # # TODO: Remove, only here to get a smaller graph
                # if a_index == 5: break

        for action in print_process.flush():
            production_data.append_action(print_process.namespace, action)

        # Add PnP actions
        first_pnp_action = True
        #stupid_counter = 0

        for path_id, pt_in_path in pt_by_path:
            #stupid_counter += 1
            #if stupid_counter > stupid_limit: break

            pt_key_set = set([geometric_key(p.pt, precision) for p in pt_in_path])
            #find out if there are add-ons on this specific path
            addons_in_path = addons_set.intersection(pt_key_set)

            if addons_in_path:
                addons = [addons_map[addon_key] for addon_key in addons_in_path]

                for item in addons:
                    addon_plane, pick_point = item['place_point'], item['pick_point']
                    pick_actions = pnp_process.pick(pick_point, xaxis, yaxis)

                    for pick_action in pick_actions:
                        # Make sure the very first PnP action is dependant on the root one, not the last print action
                        dependencies = None
                        if first_pnp_action:
                            first_pnp_action = False
                            dependencies = [join_action_id]

                        last_action_id = production_data.append_action(pnp_process.namespace, pick_action, dependency_ids=dependencies)
                        actions_by_path[path_id]['pick_actions'].append(last_action_id)

                    place_actions = pnp_process.place((addon_plane.Origin[0], addon_plane.Origin[1], addon_plane.Origin[2]), xaxis, yaxis)

                    for place_action in place_actions:
                        last_action_id = production_data.append_action(pnp_process.namespace, place_action)
                        actions_by_path[path_id]['place_actions'].append(last_action_id)

        #sort dependencies between PnP and Print
        #stupid_counter = 0
        for path_id, pt_in_path in pt_by_path:
            #stupid_counter += 1
            #if stupid_counter > stupid_limit: break

            pt_key_set = set([geometric_key(p.pt, precision) for p in pt_in_path])
            addons_in_path = addons_set.intersection(pt_key_set)
            
            if last_pnp_action_id:
                last_print_action_id = actions_by_path[path_id]['print_actions'][-1]
                # Entry of Print robot to next path depends on exit of PnP robot
                production_data.actions[last_print_action_id].dependency_ids.append(last_pnp_action_id)
                last_pnp_action_id = None

            if addons_in_path:
                first_pnp_action_id = actions_by_path[path_id]['place_actions'][0]
                last_pnp_action_id = actions_by_path[path_id]['place_actions'][-1]
                last_print_action_id = actions_by_path[path_id]['print_actions'][-1]
                # Entry of PnP robot depends on exit of Print robot
                production_data.actions[first_pnp_action_id].dependency_ids.append(last_print_action_id)

        return production_data
