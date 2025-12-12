import os, sys, time, math, random
current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_path)
sys.path.append(os.path.normpath(os.path.join(current_path, 'proto')))
from proto.sim_adapter import *

MORAI_SIM_ADDRESS = '127.0.0.1'
MORAI_SIM_PORT = 7789

"""
MORAI gRPC API 사용예제
자세한 설명 아래 gRPC Manual Link를 참조한다.
https://help-morai-sim.scrollhelp.site/ko/sim-api-guide/Working-version/grpc-api-docs
https://morai-autonomous.github.io/grpc-docs/


MORAI SIM을 실행(로딩 완료)후 본 예제코드를 실행함.

본 예제는 아래와 같은 내용을 포함하고 있음.
- Simulator 관련
    - SetSimulationStart : Simulation Start(Map, Vehicle 선택)
    - SetSimPause : Simulation 일시정지
    - SetSimResume : Simulation 일시정지 해제

- 차량 관련
    - SetVehiclePosition : Ego 차량 위치 및 자세
    - SetVehicleControlMode : Ego 차량 제어 모드
    - SetVehicleVelocity: Ego 차량 속도
    - 경로설정 , ControlMode가 VEHICLE_CONTROL_CRUISE_MODE일 때 적용
        - SetVehicleRoute : Mgeo Link idx 기준으로 NPC Vehicle 경로 설정
        - SetVehicleDestination : NPC 차량 위치와 Destination 기준으로 최단경로 설정

- Object 관련
    - SetSpawnVehicle : NPC 차량 생성
    - SetSpawnPedestrian : NPC 보행자 새성
    - SetSpawnObstacle : Object 생성
    Spawn완료 후 SimPause, SimResume을 해줘야 Simulation이 Play함.
   
- Sensor 관련
    - SetSensorFile : Sensor 세팅 파일 Load
    - SetAddSensor : 차량에 Sensor 추가

- Scenario 관련
    - SetLoadMoraiScenario : Scenario 파일 Load
    
"""
class MORAI_gRPC:

    def __init__(self):        
        self.adaptor = SimAdapter()
        self.adaptor.connect(MORAI_SIM_ADDRESS, MORAI_SIM_PORT)


    def main(self):                
        # 1. 시뮬레이션 일시정지 (안전한 생성을 위해)
        self.SetSimPause()

        print("Ego 차량 상태를 조회합니다...")
        ego_transform = self.get_ego_state_transform()

        # NPC_ID 변수를 loop에서도 쓰기 위해 초기화
        NPC_ID = None 

        if ego_transform:
            # --- NPC ID 생성 ---
            unique_time_id = str(int(time.time() * 1000))
            NPC_ID = 'NPC_' + unique_time_id 
            print(f"새로 생성할 NPC의 고유 ID: {NPC_ID}")
            
            MODEL_NAME = '2015_Kia_K5'
            LONGITUDINAL_DISTANCE = 50.0 
            LATERAL_RANGE = 3.0 
            
            # 목적지 좌표 (직진 제어를 할 경우 무시될 수 있음)
            DEST_X, DEST_Y, DEST_Z = 87.33, 1190.58, 0.0

            # 2. Spawn 위치 계산
            npc_transform = self.calculate_forward_transform(
                ego_transform, 
                longitudinal_distance=LONGITUDINAL_DISTANCE,
                lateral_range=LATERAL_RANGE
            )

            # 3. NPC 차량 생성 (multi_ego=True 상태)
            self.SetSpawnVehicle(
                npc_id=NPC_ID, 
                model_name=MODEL_NAME,
                location=npc_transform.location, 
                rotation=npc_transform.rotation
            )
            
            # 4. 목적지 설정 (참고용, 직진 제어시엔 경로 무시됨)
            self.SetVehicleDestination_for_NPC(
                npc_id=NPC_ID,
                pos_x=DEST_X, 
                pos_y=DEST_Y, 
                pos_z=DEST_Z
            )
        
        # 5. 시뮬레이션 재개
        self.SetSimResume()
        
        # ==========================================
        # [추가됨] 차량 제어 루프 (Control Loop)
        # ==========================================
        if NPC_ID is not None:
            print(f">>> {NPC_ID} 차량 제어를 시작합니다. (Ctrl+C로 종료)")
            try:
                while True:
                    # 0.05초(20Hz)마다 직진 명령 전송
                    # SetStraightMove 함수는 앞서 정의한 것을 사용한다고 가정
                    self.SetStraightMove(npc_id=NPC_ID, target_velocity=100) 
                    
                    time.sleep(0.05) # 통신 주기 유지

            except KeyboardInterrupt:
                print("제어 루프를 종료합니다.")

    

    def SetSimulationStart(self):
        """
        Simulation Start,  Map Vehicle 선택 후 맵 진입
        """
        from proto.morai.simulation.start_param_pb2 import StartParam

        request = StartParam()
        request.map_and_vehicle.map_name = 'R_KR_PG_K-City'
        request.map_and_vehicle.ego_vehicle_model = '2023_Hyundai_Ioniq5'
        try:
            response = self.adaptor._simulation_stub.Start(request)
            print(f'SetMapVehicle Response : {response.description}')
        except Exception as e :
            print(f'SetMapVehicle Error : {e}')

    def SetSensorFile(self):
        """
        sensor.config file Load
        """
        from proto.morai.common.type_pb2 import StringValue
        sensor_file_name = 'SensorInfo_2023_Hyundai_Ioniq5'    

        request = StringValue()    
        request.value = sensor_file_name
        try:
            response = self.adaptor._sensor_stub.LoadSensorFile(request)
            print(f'SetSensorFile Response : {response.description}')
        except Exception as e :
            print(f'SetSensorFile Error : {e}')
            

    def SetSimPause(self):
        from proto.morai.common.type_pb2 import Empty
        request = Empty()    
        
        try:
            response = self.adaptor._simulation_stub.Pause(request)
            print(f'SetSimPause Response : {response.description}')
        except Exception as e :
            print(f'SetSimPause Error : {e}')


    def SetSimResume(self):
        from proto.morai.common.type_pb2 import Empty
        request = Empty()    
        
        try:
            response = self.adaptor._simulation_stub.Resume(request)
            print(f'SetSimResume Response : {response.description}')
        except Exception as e :
            print(f'SetSimResume Error : {e}')


# 기존 Spawn 함수 (작성하신 코드 유지)
    def SetSpawnVehicle(self, npc_id, model_name, location, rotation):
        """
        주어진 위치(location)와 회전값(rotation)에 NPC Vehicle을 Spawn 합니다.
        """
        from morai.actor.actor_spawn_pb2 import VehicleSpawnParam
        from morai.common.enum_pb2 import ObjectType

        request = VehicleSpawnParam()
        # [주의] ID 필드명이 버전에 따라 unique_id일 수 있습니다. (확인 필요)
        # 예: request.spawn_info.actor_info.unique_id = str(npc_id) 
        request.spawn_info.actor_info.id.value = str(npc_id) 
        request.spawn_info.actor_info.object_type = ObjectType.OBJECT_TYPE_VEHICLE
        request.spawn_info.actor_info.client_key = 'Morai_Example'
        
        request.spawn_info.transform.location.x = location.x
        request.spawn_info.transform.location.y = location.y
        request.spawn_info.transform.location.z = location.z
        request.spawn_info.transform.rotation.x = rotation.x
        request.spawn_info.transform.rotation.y = rotation.y
        request.spawn_info.transform.rotation.z = rotation.z

        request.spawn_info.model_name = model_name
        request.spawn_info.label = f'Sur-{npc_id}'
        
        # Multi-Ego 설정
        request.multi_ego = True 
        request.velocity = 50 # 초기 속도 설정 (생성 순간의 속도)

        try:
            response = self.adaptor._actor_stub.SpawnVehicle(request)
            print(f'-> NPC 차량(ID:{npc_id}) Spawn 완료.')
        except Exception as e :
            print(f'SetSpawnVehicle Error : {e}')


    # [추가] 앞으로 가도록 명령을 보내는 함수
    def SetStraightMove(self, npc_id, target_velocity):
        """
        해당 NPC ID 차량에게 직진 명령(Steer=0, Velocity=target)을 전송합니다.
        """
        from morai.actor.actor_control_pb2 import VehicleCtrlCmd
        from morai.actor.actor_enum_pb2 import LongCmdType
        from morai.common.enum_pb2 import ObjectType 

        cmd = VehicleCtrlCmd()
        
        # 1. ID 설정 (unique_id 대신 id.value 사용)
        cmd.actor_info.id.value = str(npc_id)
        cmd.actor_info.object_type = ObjectType.OBJECT_TYPE_VEHICLE
        cmd.actor_info.client_key = 'Morai_Example'

        # 2. 제어 모드 및 값 설정
        cmd.long_cmd_type = 2
        cmd.velocity = target_velocity
        cmd.steer = 0.0
        cmd.brake = 0.0
        
        # 4. 명령 전송 (함수 이름 수정됨)
        try:
            # [수정 전] self.adaptor._actor_stub.SendVehicleCtrlCmd(cmd)
            # [수정 후] SetVehicleCtrlCmd 로 변경
            self.adaptor._actor_stub.SetVehicleCtrlCmd(cmd)
            
        except Exception as e:
            # 에러 메시지가 반복 출w력되면 로그가 지저분해지므로, 필요시에만 출력하거나 pass
            # print(f"Control Error: {e}")
            pass
    

    def get_ego_state_transform(self):
        """
        Ego 차량의 현재 위치(Transform) 정보를 조회하여 반환합니다.
        """
        # 경로가 설정되었으므로 'proto.'를 제거하고 'morai.'부터 임포트합니다.
        from morai.common.object_info_pb2 import ObjectInfo
        from morai.common.enum_pb2 import ObjectType
        
        request = ObjectInfo()    
        request.id.value = 'Ego'
        request.object_type = ObjectType.OBJECT_TYPE_VEHICLE
        request.client_key = 'Morai_Example'
        
        try:
            response = self.adaptor._actor_stub.GetActorState(request)
            return response.transform
        except Exception as e:
            print(f'GetEgoState Error : {e}')
            return None
        

    def calculate_forward_transform(self, ego_transform, longitudinal_distance=30.0, lateral_range=6.0):
        """
        Ego 차량의 Transform을 기준으로
        1. 종방향(앞쪽)으로 longitudinal_distance 만큼 이동
        2. 횡방향(좌우)으로 [-lateral_range, +lateral_range] 범위 내에서 랜덤 Offset 적용
        한 지점의 Transform을 계산하여 반환합니다.
        """
        from morai.common.type_pb2 import Transform 
        
        if ego_transform is None:
            return None

        # 1. Ego 차량의 현재 Yaw 각도 (라디안)
        yaw_rad = math.radians(ego_transform.rotation.z)
        
        # 2. 횡방향(Lateral) 랜덤 Offset 계산
        # -6.0m ~ +6.0m 범위에서 무작위 값 선택 (예: -2.5m, 4.1m 등)
        random_lateral_offset = random.uniform(-lateral_range, lateral_range)
        
        # 3. Spawn 위치 계산
        
        # 종방향 벡터 (Yaw 방향, cos(yaw), sin(yaw))
        # 횡방향 벡터 (Yaw에 90도를 더한 방향, cos(yaw + 90°), sin(yaw + 90°))
        # 참고: cos(θ + 90°) = -sin(θ), sin(θ + 90°) = cos(θ)
        
        # X 좌표 계산: 종방향 이동 + 횡방향 이동
        spawn_x = (
            ego_transform.location.x + 
            longitudinal_distance * math.cos(yaw_rad) +  # 종방향 (앞뒤)
            random_lateral_offset * math.cos(yaw_rad + math.pi/2) # 횡방향 (좌우, +90도)
            # 또는: random_lateral_offset * (-math.sin(yaw_rad))
        )
        
        # Y 좌표 계산: 종방향 이동 + 횡방향 이동
        spawn_y = (
            ego_transform.location.y + 
            longitudinal_distance * math.sin(yaw_rad) +  # 종방향 (앞뒤)
            random_lateral_offset * math.sin(yaw_rad + math.pi/2) # 횡방향 (좌우, +90도)
            # 또는: random_lateral_offset * (math.cos(yaw_rad))
        )
        
        # 4. 계산된 Transform 객체 생성
        calculated_transform = Transform()
        
        calculated_transform.location.x = spawn_x
        calculated_transform.location.y = spawn_y
        calculated_transform.location.z = ego_transform.location.z 
        
        # 회전값은 Ego 차량과 동일한 방향을 유지 (NPC가 Ego를 바라보지 않게)
        calculated_transform.rotation.x = ego_transform.rotation.x
        calculated_transform.rotation.y = ego_transform.rotation.y
        calculated_transform.rotation.z = ego_transform.rotation.z
        
        return calculated_transform

    def SetVehicleDestination_for_NPC(self, npc_id, pos_x, pos_y, pos_z):
        """
        NPC 차량에 Destination 기준으로 최단 경로를 세팅합니다.
        (고유 ID를 사용하기 위해 기존 SetVehicleDestination을 일반화)
        """
        # 이 함수 내의 임포트는 SetVehicleDestination과 동일합니다.
        from morai.actor.actor_set_pb2 import VehicleDestination
        from morai.common.enum_pb2 import ObjectType        

        request = VehicleDestination()                
        # >>> 여기서 고유 NPC_ID를 사용합니다.
        request.actor_info.id.value = npc_id 
        request.actor_info.object_type = ObjectType.OBJECT_TYPE_VEHICLE
        request.actor_info.client_key = 'Morai_Example'
        request.decision_range = 1
        request.position.x = pos_x
        request.position.y = pos_y
        request.position.z = pos_z

        try:
            response = self.adaptor._actor_stub.SetVehicleDestination(request)
            print(f'SetVehicleDestination_for_NPC Response : {response.description}')
        except Exception as e :
            print(f'SetVehicleDestination_for_NPC Error : {e}')


    def SetSpawnPedestrian(self):
        """
        NPC Pedestrian Spawn
        중복된 id.value 값이 사용되면 Simulator 사용에 문제가 될 수 있으니 주의필요.
        """
        from proto.morai.actor.actor_spawn_pb2 import PedestrianSpawnParam
        from proto.morai.common.enum_pb2 import ObjectType

        request = PedestrianSpawnParam()
        request.spawn_info.actor_info.id.value = '4'
        request.spawn_info.actor_info.object_type = ObjectType.OBJECT_TYPE_PEDESTRIAN
        request.spawn_info.actor_info.client_key = 'Morai_Example'
        request.spawn_info.transform.location.x = 24.92
        request.spawn_info.transform.location.y = 1110.74
        request.spawn_info.transform.location.z = 0.78
        request.spawn_info.transform.rotation.x = 0.897
        request.spawn_info.transform.rotation.y = 1.474
        request.spawn_info.transform.rotation.z = 1.751

        request.spawn_info.model_name = 'Man1'
        request.spawn_info.label = 'Ped-4'
        request.velocity = 100 #km/h
        request.active_dist = 30 #m
        request.move_dist = 100 #m
        request.start_action = True

        try:
            response = self.adaptor._actor_stub.SpawnPedestrian(request)
            print(f'SetSpawnPedestrian Response : {response.description}')
        except Exception as e :
            print(f'SetSpawnPedestrian Error : {e}')



    def SetSpawnObstacle(self, transform):
        """
        NPC Obstacle Spawn
        중복된 id.value 값이 사용되면 Simulator 사용에 문제가 될 수 있으니 주의필요.
        
        """
        from proto.morai.actor.actor_spawn_pb2 import ObstacleSpawnParam
        from proto.morai.common.enum_pb2 import ObjectType

        request = ObstacleSpawnParam()
        request.spawn_info.actor_info.id.value = '5'
        request.spawn_info.actor_info.object_type = ObjectType.OBJECT_TYPE_OBSTACLE
        request.spawn_info.actor_info.client_key = 'Morai_Example'
        request.spawn_info.transform.location.x = 24.92
        request.spawn_info.transform.location.y = 1108.74
        request.spawn_info.transform.location.z = 0.78
        request.spawn_info.transform.rotation.x = 0.897
        request.spawn_info.transform.rotation.y = 1.474
        request.spawn_info.transform.rotation.z = 1.751
        request.spawn_info.model_name = 'CargoBox' 
        request.spawn_info.label = ''
        request.scale.x = 1.0
        request.scale.y = 1.0
        request.scale.z = 1.0

        try:
            response = self.adaptor._actor_stub.SpawnObstacle(request)
            print(f'SetSpawnObstacle Response : {response.description}')
        except Exception as e :
            print(f'SetSpawnObstacle Error : {e}')


    def SetDestroyActor(self):
        """
        생성된 Actor(Object, NPC vehicle, pedestrian, obstacle..)를 삭제.

        """
        from proto.morai.common.object_info_pb2 import ObjectInfo
        from proto.morai.common.enum_pb2 import ObjectType

        request = ObjectInfo()
        request.id.value = '5'
        request.object_type = ObjectType.OBJECT_TYPE_OBSTACLE
        request.client_key = 'Morai_Example'        

        try:
            response = self.adaptor._actor_stub.DestroyActor(request)
            print(f'SetDestroyActor Response : {response.description}')
        except Exception as e :
            print(f'SetDestroyActor Error : {e}')
            

    def SetDestroyAllActors(self):
        """
        생성된 모든 Actor(Object, NPC vehicle, pedestrian, obstacle..) 삭제.
        """        
        from proto.morai.common.type_pb2 import StringValue

        request = StringValue()

        request.value = 'Morai_Example'
        try:
            response = self.adaptor._actor_stub.DestroyAllActors(request)
            print(f'SetDestroyAllActors Response : {response.description}')
        except Exception as e :
            print(f'SetDestroyAllActors Error : {e}')


    def SetVehicleRoute(self):
        """
        MGeo 기반 NPC차량 주행경로 설정.
        list_idx_list에 현재 NPC Vehicle이 위치한 Mgeo Link Index부터 시작해서
        원하는 경로(link)의 idx를 입력한다.
        마지막 지점에 도착하면 NPC 차량의 제어가 정상적이지 않을 수 있으니 
        도착지점에서 적절한 처리를 해줘야 함. (e.g SetDestory 로 해당 NPC 차량을 삭제 처리 등.)
        """
        from proto.morai.actor.actor_set_pb2 import VehicleRoute
        from proto.morai.common.enum_pb2 import ObjectType
        from proto.morai.map.link_info_pb2 import LinkInfo

        request = VehicleRoute()
        request.actor_info.id.value = '3'
        request.actor_info.object_type = ObjectType.OBJECT_TYPE_VEHICLE
        request.actor_info.client_key = 'Morai_Example'
        request.decision_range = 1

        list_idx_list = ['A219BS010380','A219BS010813', 'A219BS010398','A219BS010663','A219BS010395']
        
        for idx in list_idx_list:
            link = LinkInfo()
            link.id.value = idx
            request.links.append(link)

        try:
            response = self.adaptor._actor_stub.SetVehicleRoute(request)
            print(f'SetVehicleRoute Response : {response.description}')
        except Exception as e :
            print(f'SetVehicleRoute Error : {e}')



    def SetVehicleDestination(self):
        """
        Vehicle 차량 위치와, Destination 기준으로 최단 경로 세팅         
        """
        from proto.morai.actor.actor_set_pb2 import VehicleDestination
        from proto.morai.common.enum_pb2 import ObjectType        

        request = VehicleDestination()                
        request.actor_info.id.value = '3'
        request.actor_info.object_type = ObjectType.OBJECT_TYPE_VEHICLE
        request.actor_info.client_key = 'Morai_Example'
        request.decision_range = 1
        request.position.x = 87.33
        request.position.y = 1190.58
        request.position.z = 0

        try:
            response = self.adaptor._actor_stub.SetVehicleDestination(request)
            print(f'SetVehicleDestination Response : {response.description}')
        except Exception as e :
            print(f'SetVehicleDestination Error : {e}')



        
    def SetVehiclePosition(self):
        """
        Ego 차량 위치 Setting        
        """
        from proto.morai.common.enum_pb2 import ObjectType        
        from proto.morai.actor.actor_set_pb2 import SetTransformParam
        request = SetTransformParam()
        request.actor_info.id.value = 'Ego'
        request.actor_info.object_type = ObjectType.OBJECT_TYPE_VEHICLE
        request.actor_info.client_key = 'Morai_Example'
        request.transform.location.x = 28.22
        request.transform.location.y = 1117.90
        request.transform.location.z = 0.79
        request.transform.rotation.x = 0.78
        request.transform.rotation.y = 1.351
        request.transform.rotation.z = 3.320

        try:
            response = self.adaptor._actor_stub.SetTransform(request)
            print(f'SetVehiclePosition Response : {response.description}')
        except Exception as e :
            print(f'SetVehiclePosition Error : {e}')
        



    def SetVehicleControlMode(self):
        from proto.morai.actor.actor_set_pb2 import VehicleControlModeParam
        from proto.morai.actor.actor_enum_pb2 import VehicleControlMode
        from proto.morai.common.enum_pb2 import ObjectType
        """
        Ego 차량의 ControlMode 선택.
        request mode 
        VehicleControlMode.VEHICLE_CONTROL_KEYBOARD = 키보드 제어
        VehicleControlMode.VEHICLE_CONTROL_AUTO_MODE = 외부 제어(알고리즘)
        VehicleControlMode.VEHICLE_CONTROL_CRUISE_MODE = MORAI 내부 제어
        """
        
        request = VehicleControlModeParam()
        request.actor_info.id.value = 'Ego'
        request.actor_info.object_type = ObjectType.OBJECT_TYPE_VEHICLE
        request.actor_info.client_key = 'Morai_Example'
        request.mode = VehicleControlMode.VEHICLE_CONTROL_CRUISE_MODE

        try:
            response = self.adaptor._actor_stub.SetVehicleControlMode(request)
            print(f'SetVehicleControlMode Response : {response.description}')
        except Exception as e :
            print(f'SetVehicleControlMode Error : {e}')
        
        


    def SetVehicleVelocity(self):
        """
        Ego 차량 Velocity setting
        request.velocity에 원하는 값(km/h)을 입력한다.
        """
        from proto.morai.actor.actor_set_pb2 import SetVelocityParam
        from proto.morai.common.enum_pb2 import ObjectType
        request = SetVelocityParam()    
        request.actor_info.id.value = 'Ego'
        request.actor_info.object_type = ObjectType.OBJECT_TYPE_VEHICLE
        request.actor_info.client_key = 'Morai_Example'
        request.velocity = 0  #km/h
        try:
            response = self.adaptor._actor_stub.SetVelocity(request)
            print(f'SetVehicleVelocity Response : {response.description}')
        except Exception as e :
            print(f'SetVehicleVelocity Error : {e}')    
    



    def GetEgoState(self):
        from proto.morai.common.object_info_pb2 import ObjectInfo
        from proto.morai.common.enum_pb2 import ObjectType
        """
        Ego 차량의 상태 조회.
        request.id.value = 'Ego'
        request.object_type = ObjectType.OBJECT_TYPE_VEHICLE
        
        Sur-2 차량 상태 조회 
        request.id.value = '2'
        request.object_type = ObjectType.OBJECT_TYPE_VEHICLE

        Obj-3 장애물 상태 조회
        request.id.value = '3'
        request.object_type = ObjectType.OBJECT_TYPE_OBSTACLE

        Ped-4 사람 상태 조회
        request.id.value = '4'
        request.object_type = ObjectType.OBJECT_TYPE_PEDESTRIAN
        """
        request = ObjectInfo()    
        request.id.value = 'Ego'
        request.object_type = ObjectType.OBJECT_TYPE_VEHICLE
        request.client_key = 'Morai_Example'
        
        try:
            response = self.adaptor._actor_stub.GetActorState(request)
            print(f'GetEgoState : {response}')
        except Exception as e :
            print(f'GetEgoState Error : {e}')    


    def SetAddSensor(self):
        from proto.morai.sensor.add_sensor_param_pb2 import AddSensorParam
        """
        SetAddSensor는 이미 센서가 설치되어 있으면 다음 index로 새로운 센서가 배치된다.
        계속해서 해당 def 호출된다면 많은 센서가 배치되어서 Simulator 성능저하가 발생함.
        """        
        request = AddSensorParam()
        request.vehicle_id.id.value = "Ego"
        request.vehicle_id.object_type = 1
        request.vehicle_id.client_key = 'Morai_Example'
        request.sensor_type = 1
        request.transform.location.x = 2.2
        request.transform.location.y = 0
        request.transform.location.z = 1.1
        request.transform.rotation.x = 0
        request.transform.rotation.y = 0
        request.transform.rotation.z = 0        
        try:
            response = self.adaptor._sensor_stub.AddSensor(request)
            print(f'SetAddSensor : {response}')
        except Exception as e :
            print(f'SetAddSensor Error : {e}')    


    def SetLoadMoraiScenario(self):
        from proto.morai.common.type_pb2 import StringValue
        request = StringValue()
        request.value = 'test'

        try:
            response = self.adaptor._scenario_stub.LoadMoraiScenario(request)
            print(f'SetLoadMoraiScenario : {response}')
        except Exception as e :
            print(f'SetLoadMoraiScenario Error : {e}')    

        
if __name__ == '__main__':

    example = MORAI_gRPC()    
    example.main()
