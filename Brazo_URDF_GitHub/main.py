import argparse
import math
import time
from pathlib import Path

import pybullet as p
import pybullet_data
import serial
from serial.tools import list_ports


JOINT_NAMES = (
    "joint_1",
    "joint_2",
    "joint_gripper",
    "joint_dedo_izq",
    "joint_dedo_der",
)
BAUD_RATE = 115200
TIME_STEP = 1 / 240


def parse_arguments():
    parser = argparse.ArgumentParser(description="Control del brazo URDF con ESP32.")
    parser.add_argument("--port", help="Puerto serie de la ESP32, por ejemplo COM4.")
    parser.add_argument("--baudrate", type=int, default=BAUD_RATE)
    parser.add_argument("--demo", action="store_true", help="Usa controles PyBullet sin ESP32.")
    parser.add_argument("--list-ports", action="store_true", help="Muestra los puertos serie disponibles.")
    return parser.parse_args()


def connect_serial(port, baudrate):
    if port:
        selected_port = port
    else:
        available_ports = list(list_ports.comports())
        if len(available_ports) != 1:
            if available_ports:
                print("Puertos disponibles:")
                for item in available_ports:
                    print(f"  {item.device}: {item.description}")
            print("No se seleccionó un único puerto; inicia en modo demo o indica --port.")
            return None
        selected_port = available_ports[0].device

    connection = serial.Serial(selected_port, baudrate, timeout=0, write_timeout=0)
    connection.reset_input_buffer()
    connection.arm_line_buffer = bytearray()
    print(f"UART conectada: {selected_port} a {baudrate} baudios")
    return connection


def read_joint_targets(connection, limits, current_targets):
    if connection.in_waiting:
        connection.arm_line_buffer.extend(connection.read(connection.in_waiting))

    while b"\n" in connection.arm_line_buffer:
        raw_line, _, remaining = connection.arm_line_buffer.partition(b"\n")
        connection.arm_line_buffer = bytearray(remaining)
        line = raw_line.decode("ascii", errors="ignore").strip()
        fields = line.split(",")
        if len(fields) != len(JOINT_NAMES) + 1 or fields[0] != "J":
            continue
        try:
            positions = [float(value) for value in fields[1:]]
        except ValueError:
            continue
        if all(math.isfinite(value) for value in positions):
            return [
                min(max(value, lower), upper)
                for value, (lower, upper) in zip(positions, limits)
            ]
    return current_targets


def main():
    args = parse_arguments()
    available_ports = list(list_ports.comports())
    if args.list_ports:
        for item in available_ports:
            print(f"{item.device}: {item.description}")
        if not available_ports:
            print("No se encontraron puertos serie.")
        return

    connection = None
    if not args.demo:
        try:
            connection = connect_serial(args.port, args.baudrate)
        except serial.SerialException as error:
            if args.port:
                raise SystemExit(f"No se pudo abrir {args.port}: {error}") from error
            print(f"No se pudo abrir el puerto serie ({error}); se inicia modo demo.")

    physics_client = p.connect(p.GUI)
    if physics_client < 0:
        if connection:
            connection.close()
        raise SystemExit("No se pudo abrir la ventana de PyBullet.")

    try:
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(TIME_STEP)
        p.loadURDF("plane.urdf")
        urdf_path = Path(__file__).with_name("brazo.urdf")
        robot_id = p.loadURDF(str(urdf_path), [0, 0, 0], useFixedBase=True)

        joint_indices = {}
        for index in range(p.getNumJoints(robot_id)):
            joint_info = p.getJointInfo(robot_id, index)
            joint_name = joint_info[1].decode("utf-8")
            joint_indices[joint_name] = index
            print(f"Articulación {index}: {joint_name}")

        missing_joints = set(JOINT_NAMES) - joint_indices.keys()
        if missing_joints:
            raise RuntimeError(f"Faltan articulaciones en el URDF: {', '.join(sorted(missing_joints))}")

        limits = []
        debug_sliders = {}
        for joint_name in JOINT_NAMES:
            index = joint_indices[joint_name]
            joint_info = p.getJointInfo(robot_id, index)
            lower, upper = joint_info[8], joint_info[9]
            limits.append((lower, upper))
            p.setJointMotorControl2(
                robot_id,
                index,
                p.POSITION_CONTROL,
                targetPosition=0,
                force=joint_info[10],
                maxVelocity=joint_info[11],
            )
            if connection is None:
                debug_sliders[joint_name] = p.addUserDebugParameter(
                    joint_name, lower, upper, 0
                )

        targets = [0.0] * len(JOINT_NAMES)
        p.resetDebugVisualizerCamera(1.2, 45, -25, [0, 0, 0.45])
        if connection:
            print("Control UART activo. Cierra la ventana PyBullet o pulsa Ctrl+C para salir.")
        else:
            print("Modo demo: mueve los deslizadores de articulación en la ventana PyBullet.")

        while p.isConnected():
            if connection:
                targets = read_joint_targets(connection, limits, targets)
            else:
                targets = [p.readUserDebugParameter(debug_sliders[name]) for name in JOINT_NAMES]

            for joint_name, target in zip(JOINT_NAMES, targets):
                index = joint_indices[joint_name]
                joint_info = p.getJointInfo(robot_id, index)
                p.setJointMotorControl2(
                    robot_id,
                    index,
                    p.POSITION_CONTROL,
                    targetPosition=target,
                    force=joint_info[10],
                    maxVelocity=joint_info[11],
                )
            p.stepSimulation()
            time.sleep(TIME_STEP)
    except KeyboardInterrupt:
        print("\nSimulación detenida.")
    finally:
        if connection and connection.is_open:
            connection.close()
        if p.isConnected():
            p.disconnect()


if __name__ == "__main__":
    main()