import os, sys, time
current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_path)
sys.path.append(os.path.normpath(os.path.join(current_path, 'proto')))
from proto.sim_adapter import *
from proto.morai.common.type_pb2 import Vector3
from proto.morai.actor.actor_enum_pb2 import *
from proto.morai.infrastructure.infrastructure_enum_pb2 import *
from proto.morai.common.enum_pb2 import *

MORAI_SIM_ADDRESS = '127.0.0.1'
MORAI_SIM_PORT = 7789

SNSR_SAVE_FRQ = 10

def get_flashing_schedule(int_id, num_way):
    intscn_schedule = {
                "intersection_id": int_id,
                "vehicle_schedule": [{"duration": 60, "color_list": ["Flashing"] * num_way} for _ in range(num_way)],
                "yellow_schedule": [{"duration": int(0)} for _ in range(num_way)],
                "pedestrian_schedule": [{"synced_tl_idx": idx, 'tl_state_idx': -1, 
                                         "before_sec": 0, "green_light_sec": 0, 
                                         "flicker_light_sec": 0} for idx in range(num_way)]
            }
    return intscn_schedule

if __name__ == "__main__":
    api = SimAdapter() 
    
    # API 연결
    api.connect(MORAI_SIM_ADDRESS, MORAI_SIM_PORT)
    
    # Map 진입
    api.start("R_KR_PG_KATRI_DP", "2023_Hyundai_Ioniq5")
    
    # 시간 설정
    api.set_env_time(18)  # 대회 환경은 12, 18
    
    # 점멸등 설정
    # api.set_intersection_schedule(get_flashing_schedule("IntTL7", 4))  # 교차로 index, 삼거리 or 사거리 
    # api.set_intersection_schedule(get_flashing_schedule("IntTL3", 3))
    
    # Simulation 정지
    api.pause()
    
    # # Synchronous Mode 설정
    api.set_synchronous_mode(True, 10)  # 1tick = 10ms
    
    # Built-in Scenario Load
    # MoraiLauncher_Win\MoraiLauncher_Win_Data\SaveFile\Scenario\R_KR_PG_KATRI\{file_name} 경로에 존재할 경우
    api.load_morai_scenario("check_mission1_scenario.json")  # file_name
    
    # Control Mode 설정
    api.set_vehicle_control_mode("Ego", VEHICLE_CONTROL_CRUISE_MODE)  # Ego Controller AV - MORAI Built-In

    # Goal Point 설정
    api.set_vehicle_destination({"unique_id": "Ego", "decision_range": 2, "position": Vector3(x=float(140), y=float(1520), z=float(-0.66))})
    
    # Gear 설정
    api.set_vehicle_gear("Ego", GEAR_MODE_D)
    
    # 추종 속도 설정
    api.set_vehicle_ego_cruise(EGO_CRUISE_TYPE_CONSTANT, 30)  # 30kph 
    
    # Sensor 설정
    # MoraiLauncher_Win\MoraiLauncher_Win_Data\SaveFile\Sensor\25.S4.hmgcomp08\{file_name} 경로에 존재할 경우
    api.load_sensor_file("hmg_config.json")
    
    # Simulation 시작
    api.resume()
    curtime = api.get_timestamp()
    while True:
        res = api.tick(1)  # tick 1개 진행
            
        if res.frame_count % SNSR_SAVE_FRQ == 0:
            # api.save_sensor_data(True, str(res.frame_count), "HMG_Challenge")  # custom name option, file_name, dir_name
            ego_info = api.get_vehicle_actor_state("Ego")
            
            ego_pos = [ego_info.transform.location.x, ego_info.transform.location.y, ego_info.transform.location.z]  # e, n, u in m
            ego_ori = [ego_info.transform.rotation.x, ego_info.transform.rotation.y, ego_info.transform.rotation.z]  # r, p, y in deg [-180 ~ 180]
            ego_enu_vel = [ego_info.global_velocity.x, ego_info.global_velocity.y, ego_info.global_velocity.z]  # v_e, v_n, v_u in kph
            ego_vel = [ego_info.velocity.x, ego_info.velocity.y, ego_info.velocity.z]  # v_x, v_y, v_z in kph
            ego_ang_vel = [ego_info.angular_velocity.x, ego_info.angular_velocity.y, ego_info.angular_velocity.z]  # r, p, y rate in rad/s
            ego_acc = [ego_info.acceleration.x, ego_info.acceleration.y, ego_info.acceleration.z]  # a_x, a_y, a_z in m/s^2
            ego_throttle = ego_info.vehicle_state.throttle  # 0 ~ 1
            ego_brake = ego_info.vehicle_state.brake  # 0 ~ 1
            ego_steer = ego_info.vehicle_state.steer  # -1 ~ 1, c.c.w. is positive
            ego_link_id = ego_info.vehicle_state.current_link_info.id.value  # link id
            # ego_tl_id = ego_info.vehicle_state.tl_id.value  # tl id
            # ego_tl_color = ego_info.vehicle_state.tl_color  # tl color
            
            # ego_bbox = [ego_info.bounding_box.location.x, ego_info.bounding_box.location.y, ego_info.bounding_box.location.z,  # c_x, c_y, c_z in m
            #                ego_info.bounding_box.rotation.x, ego_info.bounding_box.rotation.y, ego_info.bounding_box.rotation.z,  # r, p, y in deg [-180 ~ 180] 
            #                          2*ego_info.bounding_box.extent.x, 2*ego_info.bounding_box.extent.y, 2*ego_info.bounding_box.extent.z]  # w, l, h in m
            
            # objs_info = api.get_all_actors_state(vehicle=True, pedestrian=True, obstacle=True)
            # for obj_info in objs_info.states:
            #     unique_id = obj_info.actor_info.id.value
            #     object_type = obj_info.actor_info.object_type  # OBJECT_TYPE_VEHICLE = 1, OBJECT_TYPE_PEDESTRIAN = 2, OBJECT_TYPE_OBSTACLE = 3
            #     if object_type not in [ObjectType.OBJECT_TYPE_VEHICLE, ObjectType.OBJECT_TYPE_PEDESTRIAN, ObjectType.OBJECT_TYPE_OBSTACLE]:
            #         continue
                
            #     option_name = api.get_option_name(unique_id, object_type)
            #     obj_bbox = [obj_info.bounding_box.location.x, obj_info.bounding_box.location.y, obj_info.bounding_box.location.z,  # c_x, c_y, c_z in m
            #                 obj_info.bounding_box.rotation.x, obj_info.bounding_box.rotation.y, obj_info.bounding_box.rotation.z,  # r, p, y in deg [-180 ~ 180] 
            #                 2*obj_info.bounding_box.extent.x, 2*obj_info.bounding_box.extent.y, 2*obj_info.bounding_box.extent.z]  # w, l, h in m

            #     print(f"Frame: {res.frame_count}, Object ID: {unique_id}, Type: {object_type}, Option Name: {option_name}, Object Bounding Box: {obj_bbox}")

            # if ego_info.vehicle_state.tl_id.value != "Not Detected":
            #     # cur_tl_info = api.get_traffic_light_info_by_uid(ego_info.vehicle_state.tl_id.value)
            #     api.set_traffic_light_state({"traffic_light_id": ego_info.vehicle_state.tl_id.value, 
            #                                  "light_color": TL_COLOR_SG, 
            #                                  "is_impulse": False, "set_sibling": False})
            
