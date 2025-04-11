import pybullet as p
import pybullet_data
import time
import math

def create_wall(x_center, y_center, half_ext_x, half_ext_y, height=0.4):
    collision_shape = p.createCollisionShape(
        shapeType=p.GEOM_BOX,
        halfExtents=[half_ext_x, half_ext_y, height / 2.0]
    )
    visual_shape = p.createVisualShape(
        shapeType=p.GEOM_BOX,
        halfExtents=[half_ext_x, half_ext_y, height / 2.0],
        rgbaColor=[0.8, 0.8, 0.8, 1]
    )
    body_id = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=collision_shape,
        baseVisualShapeIndex=visual_shape,
        basePosition=[x_center, y_center, height / 2.0]
    )
    return body_id

def create_cube_block(pos_x, pos_y, size=0.2, mass=0.00):
    collision_shape = p.createCollisionShape(
        shapeType=p.GEOM_BOX,
        halfExtents=[size/2, size/2, size/2]
    )
    visual_shape = p.createVisualShape(
        shapeType=p.GEOM_BOX,
        halfExtents=[size/2, size/2, size/2],
        rgbaColor=[0, 1, 0, 1]  # green
    )
    body_id = p.createMultiBody(
        baseMass=mass,
        baseCollisionShapeIndex=collision_shape,
        baseVisualShapeIndex=visual_shape,
        basePosition=[pos_x, pos_y, size/2]
    )
    return body_id

def create_point_agent(pos_x, pos_y, radius=0.05, mass=0.0, color=[1, 0, 0, 1]):
    collision_shape = p.createCollisionShape(
        shapeType=p.GEOM_SPHERE,
        radius=radius
    )
    visual_shape = p.createVisualShape(
        shapeType=p.GEOM_SPHERE,
        radius=radius,
        rgbaColor=color
    )
    body_id = p.createMultiBody(
        baseMass=mass,
        baseCollisionShapeIndex=collision_shape,
        baseVisualShapeIndex=visual_shape,
        basePosition=[pos_x, pos_y, radius]
    )
    return body_id

def distance_between_bodies(bodyA, bodyB):
    """
    Returns Euclidean distance between the base positions of bodyA and bodyB.
    """
    posA, _ = p.getBasePositionAndOrientation(bodyA)
    posB, _ = p.getBasePositionAndOrientation(bodyB)
    dx = posA[0] - posB[0]
    dy = posA[1] - posB[1]
    dz = posA[2] - posB[2]
    return math.sqrt(dx*dx + dy*dy + dz*dz)

def create_fixed_constraint(bodyA, bodyB, worldPos):
    """
    Creates a fixed constraint (JOINT_FIXED) between two bodies at a specified
    pivot position in world coordinates.
    
    This effectively “sticks” them together so that if one moves, the other
    follows, until the constraint is removed.
    """
    cid = p.createConstraint(
        parentBodyUniqueId=bodyA, parentLinkIndex=-1,
        childBodyUniqueId=bodyB,  childLinkIndex=-1,
        jointType=p.JOINT_FIXED, jointAxis=[0,0,0],
        parentFramePosition=worldPos,
        childFramePosition=worldPos
    )
    # Raise the maxForce
    p.changeConstraint(cid, maxForce=1000000)

    return cid

def main():
    # Connect to PyBullet in GUI mode
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    # Create a plane (floor)
    plane_id = p.loadURDF("plane.urdf")
    p.setGravity(0, 0, -9.8)

    # Create outer boundary walls (6x6 area, from -3 to +3 in x,y)
    create_wall(x_center=0,    y_center= 3.0, half_ext_x=3.0, half_ext_y=0.05)  # Top
    create_wall(x_center=0,    y_center=-3.0, half_ext_x=3.0, half_ext_y=0.05)  # Bottom
    create_wall(x_center=-3.0, y_center= 0.0, half_ext_x=0.05, half_ext_y=3.0)  # Left
    create_wall(x_center= 3.0, y_center= 0.0, half_ext_x=0.05, half_ext_y=3.0)  # Right

    # Create dividing wall with a slit in the middle
    create_wall(x_center=0, y_center= 1.7, half_ext_x=0.05, half_ext_y=1.3)  # Upper
    create_wall(x_center=0, y_center=-1.7, half_ext_x=0.05, half_ext_y=1.3)  # Lower

    # Create the cube block
    cube_id = create_cube_block(pos_x=-2, pos_y=0, size=0.2, mass=1.0)

    # Create two agents (small spheres)
    agent1_id = create_point_agent(pos_x=-2.5, pos_y= 0.5, radius=0.05, mass=0.1, color=[1,0,0,1])  # red
    agent2_id = create_point_agent(pos_x=-2.5, pos_y=-0.5, radius=0.05, mass=0.1, color=[0,0,1,1])  # blue

    p.setRealTimeSimulation(0)

    # Tracking whether each agent is currently grabbing the cube
    agent1_grabbing = False
    agent2_grabbing = False

    # Store constraint IDs for each agent if they grab the cube
    agent1_constraint_id = None
    agent2_constraint_id = None

    # Parameters
    grab_distance_threshold = 1.3  # max distance for an agent to be able to grab the cube
    agent_speed = 0.50             # speed for each agent (adjust as desired)

    try:
        while True:
            keys = p.getKeyboardEvents()

            # ------------------------------------------------------------
            # Agent 1 movement: Arrow keys
            # ------------------------------------------------------------
            agent1_vx, agent1_vy = 0.0, 0.0
            if keys.get(p.B3G_UP_ARROW) == p.KEY_IS_DOWN:
                agent1_vy += agent_speed
            if keys.get(p.B3G_DOWN_ARROW) == p.KEY_IS_DOWN:
                agent1_vy -= agent_speed
            if keys.get(p.B3G_LEFT_ARROW) == p.KEY_IS_DOWN:
                agent1_vx -= agent_speed
            if keys.get(p.B3G_RIGHT_ARROW) == p.KEY_IS_DOWN:
                agent1_vx += agent_speed

            # ------------------------------------------------------------
            # Agent 2 movement: W, A, S, D
            # ------------------------------------------------------------
            agent2_vx, agent2_vy = 0.0, 0.0
            if keys.get(ord('w')) == p.KEY_IS_DOWN or keys.get(ord('W')) == p.KEY_IS_DOWN:
                agent2_vy += agent_speed
            if keys.get(ord('s')) == p.KEY_IS_DOWN or keys.get(ord('S')) == p.KEY_IS_DOWN:
                agent2_vy -= agent_speed
            if keys.get(ord('a')) == p.KEY_IS_DOWN or keys.get(ord('A')) == p.KEY_IS_DOWN:
                agent2_vx -= agent_speed
            if keys.get(ord('d')) == p.KEY_IS_DOWN or keys.get(ord('D')) == p.KEY_IS_DOWN:
                agent2_vx += agent_speed

            # ------------------------------------------------------------
            # Set the base velocity for each agent
            # ------------------------------------------------------------
            p.resetBaseVelocity(agent1_id, linearVelocity=[agent1_vx, agent1_vy, 0], angularVelocity=[0, 0, 0])
            p.resetBaseVelocity(agent2_id, linearVelocity=[agent2_vx, agent2_vy, 0], angularVelocity=[0, 0, 0])

            # ------------------------------------------------------------
            # Agent 1 Grab/Release: 'r' or 'R'
            # ------------------------------------------------------------
            if (keys.get(ord('r')) ==3 or keys.get(ord('R')) ==3):
                if not agent1_grabbing:
                    # If not currently grabbing, check distance
                    dist = distance_between_bodies(agent1_id, cube_id)
                    if dist < grab_distance_threshold:
                        # Close enough to grab
                        agent1_grabbing = True
                        print(f"Distance between agent1 and cube:{dist}, and agent1 is grabbing")

                        # Optional: choose pivot in world coords (here, use mid-point between agent and cube)
                        agent_pos, _ = p.getBasePositionAndOrientation(agent1_id)
                        cube_pos, _  = p.getBasePositionAndOrientation(cube_id)

                        pivot_x = (agent_pos[0] + cube_pos[0]) * 0.5
                        pivot_y = (agent_pos[1] + cube_pos[1]) * 0.5
                        pivot_z = (agent_pos[2] + cube_pos[2]) * 0.5

                        agent1_constraint_id = create_fixed_constraint(
                            agent1_id, cube_id, [pivot_x, pivot_y, pivot_z]
                        )
                else:
                    # Currently grabbing => release
                    if agent1_constraint_id is not None:
                        p.removeConstraint(agent1_constraint_id)
                        agent1_constraint_id = None
                    agent1_grabbing = False
                    print(f" agent1 released BOX")


            # ------------------------------------------------------------
            # Agent 2 Grab/Release: 'h' or 'H'
            # ------------------------------------------------------------
            if (keys.get(ord('h')) == 3 or keys.get(ord('H')) ==3 ):
                if not agent2_grabbing:
                    # If not currently grabbing, check distance
                    dist = distance_between_bodies(agent2_id, cube_id)
                    if dist < grab_distance_threshold:
                        agent2_grabbing = True
                        agent_pos, _ = p.getBasePositionAndOrientation(agent2_id)
                        cube_pos, _  = p.getBasePositionAndOrientation(cube_id)

                        pivot_x = (agent_pos[0] + cube_pos[0]) * 0.5
                        pivot_y = (agent_pos[1] + cube_pos[1]) * 0.5
                        pivot_z = (agent_pos[2] + cube_pos[2]) * 0.5

                        agent2_constraint_id = create_fixed_constraint(
                            agent2_id, cube_id, [pivot_x, pivot_y, pivot_z]
                        )
                else:
                    # Currently grabbing => release
                    if agent2_constraint_id is not None:
                        p.removeConstraint(agent2_constraint_id)
                        agent2_constraint_id = None
                    agent2_grabbing = False

            # Step simulation
            p.stepSimulation()
            time.sleep(1./240.)

    except KeyboardInterrupt:
        pass

    p.disconnect()

if __name__ == "__main__":
    main()
