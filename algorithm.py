import numpy as np
from pyrep import PyRep
from pyrep.robots.arms.widowx import WidowX
from pyrep.objects.shape import Shape
from pyrep.const import PrimitiveShape

# Загрузка среды и создание экземпляра робота WidowX
pr = PyRep()
pr.launch('widowx_scene.ttt')
robot = WidowX()

# Функция определения расстояния между манипулятором и объектами в среде
def distance_to_object(obj):
    robot_pos = robot.get_tip().get_position()
    obj_pos = obj.get_position()
    return np.sqrt(np.sum((robot_pos - obj_pos)**2))

# Функция обнаружения и перемещения куба
def pick_and_place(cube, target_pos):
    # Определение положения куба и целевой точки
    cube_pos = cube.get_position()
    target = Shape.create(type=PrimitiveShape.CUBOID, size=[0.05, 0.05, 0.05], color=[1.0, 1.0, 0.0])
    target.set_position(target_pos)

    # Нахождение расстояния между кубом и манипулятором
    distance = distance_to_object(cube)

    # Перемещение манипулятора над кубом
    robot.arm.set_joint_target_positions([cube_pos[0], cube_pos[1], cube_pos[2] + 0.1, 0, 0, 0])
    pr.step()
    robot.arm.wait_until_arrived()

    # Захват куба
    robot.gripper.close()
    pr.step()
    robot.gripper.wait_until_closed()

    # Перемещение куба в целевую точку
    robot.arm.set_joint_target_positions([target_pos[0], target_pos[1], target_pos[2] + 0.1, 0, 0, 0])
    pr.step()
    robot.arm.wait_until_arrived()

    # Освобождение куба
    robot.gripper.open()
    pr.step()
    robot.gripper.wait_until_open()

    # Возврат манипулятора в начальное положение
    robot.arm.set_joint_target_positions(robot.arm.get_joint_positions())
    pr.step()
    robot.arm.wait_until_arrived()

    # Удаление целевой точки
    target.remove()

# Поиск куба рядом с манипулятором
visible_objects = [Shape('Cuboid{}'.format(i)) for i in range(10)]
nearby_cubes = [obj for obj in visible_objects if obj.get_position()[2] < 0.1]
cube = None
for obj in nearby_cubes:
    if obj.get_color()[1] >= 0.9:
        cube = obj
        break

if cube is not None:
    # Целевая точка для перемещения куба
    target_pos = [-0.2, 0.2, 0.025]

    # Выполнение операции Pick and Place
    pick_and_place(cube, target_pos)

# Остановка среды V-REP
pr.stop()
pr.shutdown()
