#!/usr/bin/env python3
"""
WebSocket server zintegrowany z ROS2 jako node.
Łączy WebSocket API z ROS2 ecosystem poprzez ros2_servo_bridge.
"""

import asyncio
import json
import logging
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Vector3

from websockets import ConnectionClosedError, ConnectionClosedOK
from websockets.asyncio.server import broadcast, serve

from .work_cycle import WorkCycle
from .servo_model import Servo

logging.basicConfig(level=logging.INFO)

# Global state
USERS = set()
VALUE = 0
WORKCYCLE = WorkCycle()

# Servos list
servos: list[Servo] = [
    Servo(0, 90),  # base rotation servo - Z axis
    Servo(1, 90),  # shoulder servo - Y axis
    Servo(2, 90),  # elbow servo - Y axis
    Servo(3, 90),  # gripper servo
]


class WebSocketROS2Node(Node):
    """Node ROS2 który integruje WebSocket server z ROS2"""
    
    def __init__(self):
        super().__init__('websocket_server_node')
        
        # Publisher do sterowania serwami
        self.servo_publisher = self.create_publisher(
            Vector3,
            'servo_command',
            10
        )
        
        self.get_logger().info('WebSocket ROS2 Node initialized')
    
    def publish_servo_command(self, channel: int, angle: float):
        """Publikuje komendę servo na topic /servo_command"""
        msg = Vector3()
        msg.x = float(channel)
        msg.y = float(angle)
        msg.z = 0.0
        
        self.servo_publisher.publish(msg)
        self.get_logger().info(f'Published: channel={channel}, angle={angle}°')


# Global node instance
ros_node: WebSocketROS2Node = None


def workcycle_load_event():
    return json.dumps({"type": "work_cycle_loaded", "history": WORKCYCLE.getHistory()})


def servo_value_event(servo_number, value):
    return json.dumps({"type": "servo_state", "servo_id": f"{servo_number}", "value": value})


def users_event():
    return json.dumps({"type": "users", "count": len(USERS)})


def value_event():
    return json.dumps({"type": "value", "value": VALUE})


async def execute_servo_step(step):
    """Wykonuje krok servo - wysyła komendy do ROS2 przez topic"""
    for servo_key, angle in step["servos"].items():
        servo_id = int(servo_key.split("_")[1])
        logging.info(f"Executing servo {servo_id} -> {angle}°")
        
        # Zaktualizuj lokalne servo
        for servo in servos:
            if servo.id == servo_id:
                servo.setValue(angle)
                break
        
        # Wyślij komendę do ROS2 przez node
        if ros_node:
            await asyncio.get_event_loop().run_in_executor(
                None,
                ros_node.publish_servo_command,
                servo_id,
                angle
            )
        
        # Broadcast do wszystkich klientów WebSocket
        broadcast(USERS, servo_value_event(servo_id, angle))


async def execute_work_cycle_once():
    """Wykonuje jedną iterację work cycle"""
    try:
        logging.info("Starting work cycle execution (once)")
        await WORKCYCLE.executeOnce(execute_servo_step)
        logging.info("Work cycle execution completed")
        broadcast(USERS, json.dumps({"type": "work_cycle_status", "status": "completed_once"}))
    except Exception as e:
        logging.error(f"Error executing work cycle: {e}")
        broadcast(USERS, json.dumps({"type": "work_cycle_status", "status": "error", "message": str(e)}))


async def execute_work_cycle_cyclic():
    """Wykonuje work cycle w trybie cyklicznym"""
    try:
        logging.info("Starting cyclic work cycle execution")
        await WORKCYCLE.executeCyclic(execute_servo_step)
        logging.info("Cyclic work cycle stopped")
    except Exception as e:
        logging.error(f"Error in cyclic work cycle: {e}")


async def event_handler(websocket):
    global USERS, VALUE
    try:
        # Register user
        USERS.add(websocket)
        broadcast(USERS, users_event())
        # Send current state to user
        await websocket.send(value_event())
        
        # Send current servo states
        for servo in servos:
            await websocket.send(servo_value_event(servo.id, servo.value))
        
        # Manage state changes
        async for message in websocket:
            logging.info(f"Received message: {message}")
            event = json.loads(message)
            match event["action"]:
                case "plus":
                    VALUE += 1
                    broadcast(USERS, value_event())
                case "minus":
                    VALUE -= 1
                    broadcast(USERS, value_event())
                case "set_servo":
                    servo_id = event["servo_id"]
                    angle = event["value"]
                    
                    # Znajdź lub utwórz servo
                    servo = next((s for s in servos if s.id == servo_id), None)
                    if servo is None:
                        servo = Servo(servo_id, angle)
                        servos.append(servo)
                    else:
                        servo.setValue(angle)
                    
                    # Wyślij do ROS2
                    if ros_node:
                        await asyncio.get_event_loop().run_in_executor(
                            None,
                            ros_node.publish_servo_command,
                            servo_id,
                            angle
                        )
                    
                    broadcast(USERS, servo_value_event(servo.id, servo.value))
                    
                case "get_available_servos":
                    for servo in servos:
                        await websocket.send(servo_value_event(servo.id, servo.value))
                        
                case "work_cycle":
                    operation = event["operation"]
                    match operation:
                        case "load_history":
                            WORKCYCLE.loadFromFile("work_cycle.json")
                            broadcast(USERS, workcycle_load_event())
                        case "save_history":
                            WORKCYCLE.saveToFile("work_cycle.json")
                        case "add_step":
                            WORKCYCLE.addStep(servos)
                            WORKCYCLE.saveToFile("work_cycle.json")
                            broadcast(USERS, workcycle_load_event())
                        case "add_delay_step":
                            delay_ms = event.get("delay_ms", 1000)
                            WORKCYCLE.addDelayStep(delay_ms)
                            WORKCYCLE.saveToFile("work_cycle.json")
                            broadcast(USERS, workcycle_load_event())
                        case "remove_step":
                            step_id = event["step_id"]
                            WORKCYCLE.removeStep(step_id)
                            WORKCYCLE.saveToFile("work_cycle.json")
                            broadcast(USERS, workcycle_load_event())
                        case "clear_history":
                            WORKCYCLE.clearHistory()
                            WORKCYCLE.saveToFile("work_cycle.json")
                            broadcast(USERS, workcycle_load_event())
                        case "run_once":
                            asyncio.create_task(execute_work_cycle_once())
                            await websocket.send(json.dumps({"type": "work_cycle_status", "status": "started_once"}))
                        case "start_cyclic":
                            WORKCYCLE.setCyclicMode(True)
                            asyncio.create_task(execute_work_cycle_cyclic())
                            await websocket.send(json.dumps({"type": "work_cycle_status", "status": "cyclic_started"}))
                        case "stop_cyclic":
                            WORKCYCLE.setCyclicMode(False)
                            await websocket.send(json.dumps({"type": "work_cycle_status", "status": "cyclic_stopped"}))
                        case _:
                            logging.error(f"unsupported work cycle event {operation}")
                case _:
                    logging.error(f"unsupported event: {event}")
                    
    except ConnectionClosedError:
        logging.info("Client disconnected unexpectedly (no close frame)")
    except ConnectionClosedOK:
        logging.info("Client disconnected gracefully")
    except Exception as e:
        logging.error(f"Unexpected error in connection handler: {e}", exc_info=True)
    finally:
        # Unregister user
        USERS.discard(websocket)
        broadcast(USERS, users_event())


async def run_websocket_server():
    """Uruchamia WebSocket server"""
    logging.info("Starting WebSocket server on ws://localhost:8765")
    async with serve(event_handler, "0.0.0.0", 8765) as server:
        await server.serve_forever()


def ros_spin_thread(node, executor):
    """Funkcja wątku dla ROS2 spin"""
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()


def main(args=None):
    global ros_node
    
    # Inicjalizacja ROS2
    rclpy.init(args=args)
    ros_node = WebSocketROS2Node()
    
    # Executor ROS2 w osobnym wątku
    executor = MultiThreadedExecutor()
    executor.add_node(ros_node)
    
    ros_thread = threading.Thread(target=ros_spin_thread, args=(ros_node, executor), daemon=True)
    ros_thread.start()
    
    logging.info("ROS2 node started in background thread")
    
    # Wczytaj work cycle z pliku
    WORKCYCLE.loadFromFile("work_cycle.json")
    logging.info(f"Loaded work cycle with {len(WORKCYCLE.getHistory())} steps")
    
    # Uruchom WebSocket server w głównym wątku (asyncio)
    try:
        asyncio.run(run_websocket_server())
    except KeyboardInterrupt:
        logging.info("Shutting down...")
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()
        ros_thread.join(timeout=2.0)


if __name__ == '__main__':
    main()
