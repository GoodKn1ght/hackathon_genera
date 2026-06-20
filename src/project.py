#!/usr/bin/env python3
import asyncio
import sys
import os
import signal
import threading
import time
import math

# Додаємо шлях до SDK
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import cv2
import rclpy
from drone_sdk import Drone

from led_detector import LEDDetector
from pid import PIDController

SWARM_SIZE = 3
ALTITUDE = 5.0  
FLIGHT_DURATION = 60

COLOR_MATRIX = {
    1: [1.0, 0.0, 0.0, 1.0],  # Дрон 1 шукає Червоного лідера (0)
    2: [0.0, 1.0, 0.0, 1.0],  # Дрон 2 шукає Зеленого дрона (1)
    3: [0.0, 0.0, 0.8, 1.0],  # Дрон 3 шукає Синього дрона (2)
}

drone_targets = {}     
drone_pitch_deg = {}   

def camera_loop(drones: list, shutdown: threading.Event):
    windows = {}
    detectors = {}
    pids_yaw = {}
    pids_range = {}
    pids_alt = {}
    
    last_seen_time = {}
    last_speeds = {}

    cam_width = 640
    cam_height = 480
    
    target_cx = cam_width / 2.0  
    static_cy = cam_height / 2.0
    
    fy_pixels = 554.26  
    target_area = 300.0  
    
    # [НОВЕ] Рахуємо корінь бажаної площі один раз для оптимізації
    sqrt_target_area = math.sqrt(target_area)

    for d in drones:
        d.start_camera()
        win = f'Drone {d.drone_id}'
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        windows[d.drone_id] = {'name': win, 'sized': False}
        
        if d.drone_id > 0:
            rgba = COLOR_MATRIX.get(d.drone_id, [0.0, 1.0, 0.0, 1.0])
            detectors[d.drone_id] = LEDDetector(rgba_color=rgba, min_area=25)
            
            # [НОВЕ] Більш м'які Kp, але сильніші Kd для демпфування (щоб дрон не пролітав ціль)
            pids_yaw[d.drone_id] = PIDController(kp=0.05, ki=0.00, kd=0.02, max_output=25.0, min_output=-25.0)
            
            # [НОВЕ] Оскільки ми перейшли на квадратні корені, помилка тепер маленька (від 1 до 20). 
            # Тому Kp для дальності став більшим (0.3), але сам рух буде ідеально лінійним.
            pids_range[d.drone_id] = PIDController(kp=0.30, ki=0.001, kd=0.08, max_output=2.5, min_output=-0.5)
            
            # [НОВЕ] Дуже ніжний контроль висоти, щоб не було мікро-стрибків
            pids_alt[d.drone_id] = PIDController(kp=0.01, ki=0.00, kd=0.005, max_output=1.0, min_output=-1.0)
            
            drone_targets[d.drone_id] = [0.0, 0.0, 0.0]  
            last_seen_time[d.drone_id] = 0.0
            last_speeds[d.drone_id] = [0.0, 0.0, 0.0]

    while not shutdown.is_set() and rclpy.ok():
        current_time = time.time()
        
        for d in drones:
            d.spin()
            frame = d.camera_frame()
            
            if frame is not None:
                if d.drone_id in detectors:
                    cx, cy, area = detectors[d.drone_id].detect(frame, d.drone_id)
                    
                    if cx is not None and cy is not None:
                        # 1. Yaw
                        yaw_error_deg = pids_yaw[d.drone_id].update(current_value=target_cx, target_value=cx)
                        area_error = sqrt_target_area - math.sqrt(area)
                        raw_vx = pids_range[d.drone_id].update(current_value=0.0, target_value=area_error)
                        current_pitch = drone_pitch_deg.get(d.drone_id, 0.0)
                        pitch_rad = math.radians(current_pitch)
                        dynamic_target_cy = static_cy + (fy_pixels * math.tan(pitch_rad))
                        dynamic_target_cy = max(20.0, min(460.0, dynamic_target_cy))
                        raw_vz = pids_alt[d.drone_id].update(current_value=cy, target_value=dynamic_target_cy)
                        alpha = 0.2 
                        prev_vx, prev_yaw, prev_vz = last_speeds[d.drone_id]
                        
                        smooth_vx = prev_vx + alpha * (raw_vx - prev_vx)
                        smooth_yaw = prev_yaw + alpha * (yaw_error_deg - prev_yaw)
                        smooth_vz = prev_vz + alpha * (raw_vz - prev_vz)
                        
                        good_speeds = [smooth_vx, smooth_yaw, smooth_vz]
                        
                        last_speeds[d.drone_id] = good_speeds
                        last_seen_time[d.drone_id] = current_time
                        drone_targets[d.drone_id] = good_speeds
                        
                    else:
                        time_lost = current_time - last_seen_time[d.drone_id]
                        if time_lost < 0.4:
                            drone_targets[d.drone_id] = last_speeds[d.drone_id]
                        elif time_lost < 2.0:
                            decay = max(0.0, 1.0 - ((time_lost - 0.4) / 1.6))
                            drone_targets[d.drone_id] = [
                                last_speeds[d.drone_id][0] * decay,
                                last_speeds[d.drone_id][1] * decay,
                                last_speeds[d.drone_id][2] * decay
                            ]
                        else:
                            pids_yaw[d.drone_id].clear()
                            pids_range[d.drone_id].clear()
                            pids_alt[d.drone_id].clear()
                            drone_targets[d.drone_id] = [0.0, 6.0, 0.0]
                            last_speeds[d.drone_id] = [0.0, 0.0, 0.0] # Скидаємо інерцію при повній втраті

                entry = windows[d.drone_id]
                if not entry['sized']:
                    h, w = frame.shape[:2]
                    cv2.resizeWindow(entry['name'], w, h)
                    entry['sized'] = True
                cv2.imshow(entry['name'], frame)
                
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == 27:
            shutdown.set()
            break

    for win in windows.values():
        cv2.destroyWindow(win['name'])
    for d in drones:
        d.stop_camera()


async def leader_mission(drone: Drone, stop_event: asyncio.Event, shutdown: asyncio.Event):
    """Асинхронний траєкторний маршрут з плавними поворотами (Слалом / S-подібна крива)"""
    try:
        print("Leader (Drone 0): waiting 20s for EKF2 & GPS lock...")
        await asyncio.sleep(20)
        
        print("Leader (Drone 0): arming & takeoff")
        await drone.arm()
        await drone.takeoff(altitude_m=ALTITUDE)
        
        await asyncio.sleep(10)
        await drone.start_offboard()
        
        # Фіксуємо базовий напрямок руху
        base_yaw = await drone.heading()
        print("Leader (Drone 0): Starting smooth slalom trajectory...")
        
        start_time = time.time()
        while not stop_event.is_set() and not shutdown.is_set():
            # Рахуємо чистий час польоту від початку місії
            t = time.time() - start_time
            
            # --- ПАРАМЕТРИ ПЛАВНОЇ ТРАЄКТОРІЇ ---
            # Амплітуда відхилення носа вліво/вправо (в градусах)
            amplitude_deg = 40.0  
            period = 10.0  
            omega = (2.0 * math.pi) / period
            yaw_offset = amplitude_deg * math.sin(omega * t)
            current_heading = (base_yaw + yaw_offset) % 360.0
            v_forward = 1.3  
            heading_rad = math.radians(current_heading)
            vn = v_forward * math.cos(heading_rad)
            ve = v_forward * math.sin(heading_rad)
            vz = -0.2
            await drone.set_velocity(vn, ve, vz, yaw_deg=current_heading)
            await asyncio.sleep(0.1)
        print("Leader (Drone 0): Місію завершено. Зависання.")
        while not stop_event.is_set() and not shutdown.is_set():
            await drone.set_velocity(0.0, 0.0, 0.0, yaw_deg=base_yaw)
            await asyncio.sleep(0.1)
            
    except Exception as e:
        print(f"Leader Error: {e}")


async def follower_mission(drone: Drone, drone_id: int, stop_event: asyncio.Event, shutdown: asyncio.Event):
    try:
        print(f"Follower {drone_id}: waiting 20s for EKF2 lock...")
        await asyncio.sleep(20)
        
        print(f"Follower {drone_id}: arming & takeoff")
        await drone.arm()
        await drone.takeoff(altitude_m=ALTITUDE)
        
        await asyncio.sleep(10)
        await drone.start_offboard()
        print(f"Follower {drone_id}: Offboard active. Visual tracking engaged.")

        while not stop_event.is_set() and not shutdown.is_set():
            hdg = await drone.heading()
            
            try:
                roll_rad, pitch_rad, yaw_rad = await drone.attitude()
                drone_pitch_deg[drone_id] = math.degrees(pitch_rad)
            except Exception as e:
                pass 

            speeds = drone_targets.get(drone_id, [0.0, 0.0, 0.0])
            vx, yaw_error_deg, vz = speeds
            
            target_heading = (hdg + yaw_error_deg) % 360.0
            scale = max(0.3, 1.0 - (abs(yaw_error_deg) / 35.0))
            vx *= scale

            target_heading_rad = math.radians(target_heading)
            vn = vx * math.cos(target_heading_rad)
            ve = vx * math.sin(target_heading_rad)
            
            await drone.set_velocity(vn, ve, -vz, yaw_deg=target_heading)
            await asyncio.sleep(0.1)
            
    except Exception as e:
        print(f"Follower {drone_id} Error: {e}")


async def main():
    rclpy.init()
    shutdown_event = threading.Event()
    shutdown_async = asyncio.Event()
    stop_event = asyncio.Event()

    def on_sig(*_):
        shutdown_event.set()
        shutdown_async.set()
        stop_event.set()
    signal.signal(signal.SIGINT, on_sig)

    drones = [Drone(drone_id=i) for i in range(SWARM_SIZE)]

    print('Connecting all drones...')
    for d in drones:
        await d.connect()
        print(f'  Drone {d.drone_id}: connected')

    cam_thread = threading.Thread(target=camera_loop, args=(drones, shutdown_event), daemon=True)
    cam_thread.start()
    await asyncio.sleep(1.0)

    print('Activating LED beacons for the whole swarm...')
    for d in drones:
        d.set_leds('1111')

    tasks = []
    tasks.append(asyncio.create_task(leader_mission(drones[0], stop_event, shutdown_async)))
    for i in range(1, SWARM_SIZE):
        tasks.append(asyncio.create_task(follower_mission(drones[i], i, stop_event, shutdown_async)))

    print(f'Mission running. Target flight duration: {FLIGHT_DURATION}s')
    
    timer = 0
    while timer < FLIGHT_DURATION and not shutdown_event.is_set():
        await asyncio.sleep(1.0)
        timer += 1

    print('Mission complete. Landing sequence...')
    stop_event.set()
    await asyncio.gather(*tasks, return_exceptions=True)
    
    for d in drones:
        try:
            await d.set_velocity(0.0, 0.0, 0.0, yaw_deg=0.0)
            await d.stop_offboard()
            await d.land()
        except:
            pass
            
    await asyncio.sleep(15)
    for d in drones:
        try:
            await d.disarm()
        except:
            pass
            
    shutdown_event.set()
    print('Swarm safely disarmed. Process finished.')

if __name__ == '__main__':
    asyncio.run(main())